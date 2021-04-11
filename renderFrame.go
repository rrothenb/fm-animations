package main

import (
	"flag"
	"fmt"
	"golang.org/x/text/language"
	"golang.org/x/text/message"
	"image"
	"image/png"
	"math"
	"math/rand"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/hunterloftis/pbr/pkg/material"
	"github.com/hunterloftis/pbr/pkg/render"
	"github.com/hunterloftis/pbr/pkg/rgb"
	"github.com/hunterloftis/pbr/pkg/surface"
)

func strength(x float64) float64 {
	return math.Sin(x)*.75 + .85
}

func texture(u, v, t float64) float64 {
	return math.Sin(
		3*u + 5*v + strength(.1+2*t)*math.Sin(
			2*u+strength(.2+3*t)*math.Sin(3*u)) + strength(.3+5*t)*math.Sin(
			7*v+strength(.4+7*t)*math.Sin(5*v)) + strength(.5+11*t)*math.Sin(
			11*u+13*v) + strength(.6+13*t)*math.Sin(17*u-5*v) + strength(.7+17*t)*math.Sin(23*v-11*u))
}

func radius(u, v, t float64) float64 {
	return 1.0 + .075*math.Pow(math.Pow(texture(u, v, t), 2), 3)
}

type SLR2 struct {
	Width  float64
	Height float64
	Lens   float64
	FStop  float64
	Focus  float64

	trans       *geom.Mtx
	position    geom.Vec
	target      geom.Vec
	subframes   bool
	subframeRow int
	subframeCol int
}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2(subframe int) *SLR2 {
	s := &SLR2{
		Width:     0.036, // 36mm (full frame sensor width)
		Height:    0.024, // 24mm (full frame sensor height)
		Lens:      0.050, // 50mm focal length
		FStop:     4,
		Focus:     1,
		position:  geom.Vec{0, 0, 0},
		target:    geom.Vec{0, 0, -5},
		subframes: subframe > 0,
	}
	if s.subframes {
		s.Lens = s.Lens * 2
		s.subframeRow = 2 - (subframe-1)/6
		s.subframeCol = (subframe-1)%6 - 3
		fmt.Printf("row: %v, col: %v\n", s.subframeRow, s.subframeCol)
	}
	s.transform()
	return s
}

// LookAt orients a Camera to face a target.
func (s *SLR2) LookAt(target geom.Vec) *SLR2 {
	s.target = target
	s.transform()
	return s
}

// MoveTo moves a Camera to a position given by x, y, and z coordinates.
func (s *SLR2) MoveTo(pos geom.Vec) *SLR2 {
	s.position = pos
	s.transform()
	return s
}

func (s *SLR2) Ray(x, y, width, height float64, rnd *rand.Rand) *geom.Ray {
	targetDist := s.target.Minus(s.position).Len()
	u := x / width
	v := y / height
	aSense := s.Width / s.Height
	aImage := width / height
	if aImage > aSense { // wider image; crop vertically
		r := aSense / aImage
		v = (1-r)*0.5 + v*r
	} else if aSense > aImage { // taller image; crop horizontally
		r := aImage / aSense
		u = (1-r)*0.5 + u*r
	}
	focusDist := targetDist * s.Focus
	sensorPt := s.sensorPoint(u, v, focusDist)
	straight, _ := geom.Vec{}.Minus(sensorPt).Unit()
	focalPt := geom.Vec(straight).Scaled(focusDist) // TODO: is this creating a curved focal plane? need to project along the center axis?
	lensPt := s.aperturePoint(rnd)
	refracted, _ := focalPt.Minus(lensPt).Unit()
	ray := geom.NewRay(lensPt, refracted)
	return s.trans.MultRay(ray)
}
func (s *SLR2) transform() {
	s.trans = geom.LookMatrix(s.position, s.target)
	if s.subframes {
		s.trans = s.trans.Mult(geom.Shift(geom.Vec{s.Height/8 + s.Height/4*float64(s.subframeCol), s.Height/8 + s.Height/4*float64(s.subframeRow), 0}))
	}
}

func (s *SLR2) invisible(point geom.Vec) bool {
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	if projectedPoint.Z > 0.0 {
		return true
	}
	if projectedPoint.Z < -s.position.Len() {
		return true
	}
	if s.subframes {
		subframeSize := -projectedPoint.Z/4/3
		xOffset := float64(s.subframeCol)*subframeSize
		yOffset := float64(s.subframeRow)*subframeSize
		if projectedPoint.X < math.Min(xOffset, 0) - subframeSize/2 || projectedPoint.X > math.Max(xOffset, 0) + subframeSize/2 {
			return true
		}
		if projectedPoint.Y < math.Min(yOffset, 0) - subframeSize/2 || projectedPoint.Y > math.Max(yOffset, 0) + subframeSize/2 {
			return true
		}
	} else {
		if projectedPoint.X < projectedPoint.Z/4 || projectedPoint.X > -projectedPoint.Z/4 {
			return true
		}
		if projectedPoint.Y < projectedPoint.Z/4 || projectedPoint.Y > -projectedPoint.Z/4 {
			return true
		}
	}
	return false
}

func (s *SLR2) sensorPoint(u, v, focusDist float64) geom.Vec {
	z := 1 / ((1 / s.Lens) - (1 / focusDist))
	x := (u - 0.5) * s.Width
	y := (v - 0.5) * s.Height
	return geom.Vec{-x, y, z}
}

// https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
func (s *SLR2) aperturePoint(rnd *rand.Rand) geom.Vec {
	d := s.Lens / s.FStop
	t := 2 * math.Pi * rnd.Float64()
	r := math.Sqrt(rnd.Float64()) * d * 0.5
	x := r * math.Cos(t)
	y := r * math.Sin(t)
	return geom.Vec{x, y, 0}
}

func writePng(filename string, im image.Image) error {
	out, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer out.Close()
	return png.Encode(out, im)
}

func Iterative(scene *render.Scene, file string, width, height, depth int, direct bool, maxSeconds int) error {
	kill := make(chan os.Signal, 2)
	signal.Notify(kill, os.Interrupt, syscall.SIGTERM)

	frame := scene.Render(width, height, depth, direct)
	defer frame.Stop()
	ticker := time.NewTicker(6 * time.Second) // 10 .s = 1 minute, 100 .s = 1 hr
	defer ticker.Stop()

	start := time.Now().UnixNano()
	max := 0
	fmt.Printf("\nRendering %v (Ctrl+C to end)", file)

	for frame.Active() {
		select {
		case <-kill:
			frame.Stop()
		case <-ticker.C:
			if sample, n := frame.Sample(); n > max {
				max = n
				fmt.Print(".")
				if err := writePng(file, sample.Image()); err != nil {
					return err
				}
				if time.Now().UnixNano()-start > int64(maxSeconds)*1e9 {
					frame.Stop()
				}
			}
		}
	}

	stop := time.Now().UnixNano()
	sample, _ := frame.Sample()
	total := sample.Total()
	p := message.NewPrinter(language.English)
	secs := float64(stop-start) / 1e9
	sps := math.Round(float64(total) / secs) // TODO: rename to pixels/sec for clarity
	p.Printf("\n%v samples in %.1f seconds (%.0f samples/sec)\n", total, secs, sps)

	return nil
}

type FMEnv struct {
	t float64
}

func (e *FMEnv) At(dir geom.Dir) rgb.Energy {
	u := math.Atan2(dir.X, -dir.Z)
	v := math.Acos(dir.Y)
	a := (1 - math.Pow(1-math.Pow(texture(u, v, e.t), 2), math.Pow(1-v/math.Pi, 7)*50)) * 400
	return rgb.Energy{
		X: a,
		Y: a,
		Z: a,
	}
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	return geom.Vec{
		math.Sin(v/2.0) * math.Cos(u) * a,
		math.Sin(v/2.0) * math.Sin(u) * a,
		math.Cos(v/2.0) * a,
	}
}

func index2radians(index, n int) float64 {
	return float64(index) / float64(n) * math.Pi * 2
}

func renderSurfaces(frameNumber int, subframe int, pixels int, maxSubdivisions int, maxTime int, dt float64, surfaces []render.Surface) {
	t := float64(frameNumber) * dt
	cameraLoc := geom.Vec{math.Sin(t), math.Sin(t), math.Cos(t)}.Scaled(.3).Plus(geom.Vec{0.0, 0.0, -.1})
	unitCameraLoc, _ := cameraLoc.Unit()
	focusPoint := unitCameraLoc.Scaled(.075)
	c := NewSLR2(subframe).MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	n := int(float64(pixels) / distance / 2)
	if n > maxSubdivisions {
		n = maxSubdivisions
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v, maxTime: %v\n", cameraLoc.Len(), distance, n, maxTime)
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			if c.invisible(topRight) {
				continue
			}
			topLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex, n), t, radius).Scaled(.075)
			botRight := uv2xyz(index2radians(uIndex, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			botLeft := uv2xyz(index2radians(uIndex+1, n), index2radians(vIndex+1, n), t, radius).Scaled(.075)
			fm := texture(index2radians(uIndex, n), index2radians(vIndex, n), t)
			fm4 := math.Pow(fm, 4)
			roughness := fm4*.09 + .01
			metalness := (1-fm4)*.8 + .1
			specularity := fm4*.3 + .1
			color := rgb.Energy{
				1,
				.5 + .05*fm,
				.25 + .15*math.Sin(4*fm)}.Scaled(fm4).Plus(rgb.Energy{.8, .8, .8}.Scaled(1 - fm4))
			m := &material.Uniform{
				Color:       color,
				Metalness:   metalness,
				Roughness:   roughness,
				Specularity: specularity,
			}
			if vIndex != 0 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botLeft, topLeft, m))
			}
			if vIndex != n-1 {
				surfaces = append(surfaces,
					surface.NewTriangle(topRight, botRight, botLeft, m))
			}
		}
	}
	fmt.Println(len(surfaces), n*n*2)

	s := surface.NewTree(surfaces...)
	e := &FMEnv{t}

	scene := render.NewScene(c, s, e)
	framePath := fmt.Sprintf("images/%04v.png", frameNumber)
	if subframe > 0 {
		framePath = fmt.Sprintf("images/%04v.%02v.png", frameNumber, subframe)
	}
	err := Iterative(scene, framePath, pixels, pixels, 8, true, maxTime)
	if err != nil {
		fmt.Fprintf(os.Stderr, "\nError: %v\n", err)
	}
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	subframe := flag.Int("subframe", 0, "Specify subframe")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxTime := flag.Int("maxtime", 0, "Max time to render")
	maxFrames := flag.Int("maxframes", 5000, "Max frames")
	flag.Parse()
	fmt.Printf("subframe: %v, frame: %v, pixels: %v, maxSubdivisions: %v, maxTime: %v, maxFrames: %v\n", *subframe, *frame, *pixels, *maxSubdivisions, *maxTime, *maxFrames)
	dt := math.Pi * 2 / float64(*maxFrames)
	var surfaces []render.Surface
	renderSurfaces(*frame, *subframe, *pixels, *maxSubdivisions, *maxTime, dt, surfaces)
}
