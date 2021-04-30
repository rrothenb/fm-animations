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
	"sync"

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
	subframeRow float64
	subframeCol float64
}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2(subframe int) *SLR2 {
	s := &SLR2{
		Width:     0.024,
		Height:    0.024,
		Lens:      0.050, // 50mm focal length
		FStop:     4,
		Focus:     1,
		position:  geom.Vec{0, 0, 0},
		target:    geom.Vec{0, 0, -5},
		subframes: subframe > 0,
	}
	if s.subframes {
		s.subframeRow = float64(5 - (subframe-1)/10)
		s.subframeCol = float64((subframe-1)%10 - 4)
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
	if s.subframes {
		// top right is 0,0 offset (row is reversed)
		u = u/10 + (s.subframeCol + 4)/10
		v = v/10 + (5 - s.subframeRow)/10
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
	if projectedPoint.X < projectedPoint.Z/4 || projectedPoint.X > -projectedPoint.Z/4 {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z/4 || projectedPoint.Y > -projectedPoint.Z/4 {
		return true
	}
	/*
	if s.subframes {
		subframeSize := -projectedPoint.Z/6/5
		xOffset := float64(s.subframeCol)*subframeSize
		yOffset := float64(s.subframeRow)*subframeSize
		if projectedPoint.X < xOffset - subframeSize - subframeSize*2.5 || projectedPoint.X > xOffset + subframeSize*2.5 {
			return true
		}
		if projectedPoint.Y < yOffset - subframeSize - subframeSize*2.5 || projectedPoint.Y > yOffset + subframeSize*2.5 {
			return true
		}
	}
	 */
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

func index2radians(index float64, n int) float64 {
	return index / float64(n) * math.Pi * 2
}

type Interpolated struct {
	u *material.Uniform
	v *material.Uniform
	w *material.Uniform
}

func (interpolated *Interpolated) interpolate(u, v float64) *material.Uniform {
	w := 1 - u - v
	return &material.Uniform{
		Color:       interpolated.u.Color.Scaled(u).Plus(interpolated.v.Color.Scaled(v)).Plus(interpolated.w.Color.Scaled(w)),
		Metalness:   interpolated.u.Metalness*u+interpolated.v.Metalness*v+interpolated.w.Metalness*w,
		Roughness:   interpolated.u.Roughness*u+interpolated.v.Roughness*v+interpolated.w.Roughness*w,
		Specularity: interpolated.u.Specularity*u+interpolated.v.Specularity*v+interpolated.w.Specularity*w,
	}
}

func (interpolated *Interpolated) At(u, v float64, in, norm geom.Dir, rnd *rand.Rand) (geom.Dir, render.BSDF) {
	return interpolated.interpolate(u, v).At(u, v, in, norm, rnd)
}

func (interpolated *Interpolated) Light() rgb.Energy {
	return rgb.Black
}

func (interpolated *Interpolated) Transmit() rgb.Energy {
	return rgb.Black
}

func uvIndexToMaterial(uIndex, vIndex, n int, t float64) *material.Uniform {
	fm := texture(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t)
	fm4 := math.Pow(fm, 4)
	roughness := fm4*.09 + .01
	metalness := (1-fm4)*.8 + .1
	specularity := fm4*.3 + .1
	color := rgb.Energy{
		1,
		.5 + .05*fm,
		.25 + .15*math.Sin(4*fm)}.Scaled(fm4).Plus(rgb.Energy{.8, .8, .8}.Scaled(1 - fm4))
	return &material.Uniform{
		Color:       color,
		Metalness:   metalness,
		Roughness:   roughness,
		Specularity: specularity,
	}
}

func uvIndexToNormal(uIndex, vIndex, n int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)+.1, n), t, radius).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)-.1, n), t, radius).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

// new Object/Surface - like Sphere

type Square struct {
	left surface.Triangle
	right surface.Triangle
}

type Shape struct {
	vertices sync.Map
	normals sync.Map
	materials sync.Map
	squares sync.Map
	outerBound *surface.Sphere
	innerBound *surface.Sphere
	intersections int
	triangles []surface.Triangle
}

func NewShape() *Shape {
	u := 17
	v := 31
	topRight := uv2xyz(index2radians(float64(u), 32), index2radians(float64(v), 32), 0, radius).Scaled(.075)
	topLeft := uv2xyz(index2radians(float64(u+1), 32), index2radians(float64(v), 32), 0, radius).Scaled(.075)
	botRight := uv2xyz(index2radians(float64(u), 32), index2radians(float64(v+1), 32), 0, radius).Scaled(.075)
	botLeft := uv2xyz(index2radians(float64(u+1), 32), index2radians(float64(v+1), 32), 0, radius).Scaled(.075)
	var surfaces []surface.Triangle
	surfaces = append(surfaces, *surface.NewTriangle(topRight, botLeft, topLeft))
	surfaces = append(surfaces, *surface.NewTriangle(topRight, botRight, botLeft))
	return &Shape {
		outerBound: surface.UnitSphere().Scale(geom.Vec{.075*(1+.075),.075*(1+.075),.075*(1+.075)}),
		innerBound: surface.UnitSphere().Scale(geom.Vec{.075,.075,.075}),
		triangles: surfaces,
	}
}

func (s *Shape) Light() rgb.Energy {
	return rgb.Black
}

func (s *Shape) Transmit() rgb.Energy {
	return rgb.Black
}

func (s *Shape) Bounds() *geom.Bounds {
	return s.outerBound.Bounds()
}

func (s *Shape) Intersect(ray *geom.Ray, max float64) (obj render.Object, dist float64) {
	//obj, dist = s.outerBound.Intersect(ray, max)
	obj, dist = surface.NewTree(&s.triangles[0], &s.triangles[1]).Intersect(ray, max)
	if obj != nil {
		s.intersections++
		intersection, _ := ray.Origin.Plus(ray.Dir.Scaled(dist)).Unit()
		u := (math.Atan2(-intersection.Y, -intersection.X)/math.Pi/2+.5)*32
		v := math.Acos(intersection.Z)/math.Pi*32
		// both indices go from 0 to n when u/v go from 0 to 2pi
		// v goes from -pi/2 to pi/2
		// subframe 1 -x, y | x, y
		// subframe 10 x, y | -x, y
		// subframe 91 -x, -y | x, -y
		// subframe 100 x, -y | -x, -y
		// x is opposite around back - that makes sense
		// x,y make u
		// z makes v
		// u of 0 and n are both 3:00 at frame 0
		// 0,1 - .2, .03, .99 - 1.4, .18
		// 0,31 - .1, 0, -1 - 1.5, 3
		// 4,1 - .1,.15,.99
		// 8,1 - 0, .2, .99
		// 16,1 - -.15,0, .99
		// 28,1 - .08, -.06, .99
		// v above goes from 0 to pi, u goes from
		if s.intersections < 10 {
			fmt.Println(intersection, u, v)
			//panic("hello")
		}
		obj = s
	}
	return
}

func (s *Shape) At(pt geom.Vec, in geom.Dir, rnd *rand.Rand) (normal geom.Dir, bsdf render.BSDF) {
	return s.outerBound.At(pt, in, rnd)
}

func (s *Shape) Lights() []render.Object {
	return nil
}

func renderSurfaces(frameNumber int, subframe int, pixels int, maxSubdivisions int, maxTime int, dt float64, surfaces []render.Surface, info bool) {
	t := float64(frameNumber) * dt
	// this should speed up at t around pi
	cameraT := (t - math.Pi)/math.Pi
	cameraT = math.Pow(cameraT*cameraT, .1)*math.Pi + math.Pi
	if t < math.Pi {
		cameraT = -cameraT
	}
	cameraLoc := geom.Vec{math.Sin(cameraT), math.Sin(cameraT), math.Cos(cameraT)}.Scaled(.3).Plus(geom.Vec{0.0, 0.0, -.1})
	unitCameraLoc, _ := cameraLoc.Unit()
	focusPoint := unitCameraLoc.Scaled(.075)
	c := NewSLR2(subframe).MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	n := int(float64(pixels) / distance * 3)
	if n > maxSubdivisions {
		n = maxSubdivisions
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v, maxTime: %v\n", cameraLoc.Len(), distance, n, maxTime)
	/*
	vertices := make([][]geom.Vec, n+1)
	materials := make([][]*material.Uniform, n+1)
	normals := make([][]*geom.Dir, n+1)
	for uIndex := 0; uIndex <= n; uIndex++ {
		vertices[uIndex] = make([]geom.Vec, n+1)
		materials[uIndex] = make([]*material.Uniform, n+1)
		normals[uIndex] = make([]*geom.Dir, n+1)
		for vIndex := 0; vIndex <= n; vIndex++ {
			vertices[uIndex][vIndex] = uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
			if c.invisible(vertices[uIndex][vIndex]) {
				continue
			}
			materials[uIndex][vIndex] = uvIndexToMaterial(uIndex, vIndex, n, t)
			normals[uIndex][vIndex] = uvIndexToNormal(uIndex, vIndex, n, t)
		}
	}
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := vertices[uIndex][vIndex]
			topLeft := vertices[uIndex+1][vIndex]
			botRight := vertices[uIndex][vIndex+1]
			botLeft := vertices[uIndex+1][vIndex+1]
			if c.invisible(topRight) || c.invisible(topLeft) || c.invisible(botRight) || c.invisible(botLeft) {
				continue
			}
			topRightMaterial := materials[uIndex][vIndex]
			topRightNormal := normals[uIndex][vIndex]
			topLeftMaterial := materials[uIndex+1][vIndex]
			topLeftNormal := normals[uIndex+1][vIndex]
			botRightMaterial := materials[uIndex][vIndex+1]
			botRightNormal := normals[uIndex][vIndex+1]
			botLeftMaterial := materials[uIndex+1][vIndex+1]
			botLeftNormal := normals[uIndex+1][vIndex+1]
			if vIndex != 0 {
				m := &Interpolated{u: topRightMaterial, v: botLeftMaterial, w: topLeftMaterial}
				triangle := surface.NewTriangle(topRight, botLeft, topLeft, m)
				triangle.Texture[0] = geom.Vec{1,0,0}
				triangle.Texture[1] = geom.Vec{0,1,0}
				triangle.SetNormals(*topRightNormal, *botLeftNormal, *topLeftNormal)
				surfaces = append(surfaces, triangle)
			}
			if vIndex != n-1 {
				m := &Interpolated{u: topRightMaterial, v: botRightMaterial, w: botLeftMaterial}
				triangle := surface.NewTriangle(topRight, botRight, botLeft, m)
				triangle.Texture[0] = geom.Vec{1,0,0}
				triangle.Texture[1] = geom.Vec{0,1,0}
				triangle.SetNormals(*topRightNormal, *botRightNormal, *botLeftNormal)
				surfaces = append(surfaces, triangle)
			}
		}
	}
	 */
	surfaces = append(surfaces,NewShape())
	fmt.Println(len(surfaces), n*n*2)

	if info {
		return
	}

	s := surface.NewTree(surfaces...)
	e := &FMEnv{t}

	scene := render.NewScene(c, s, e)
	framePath := fmt.Sprintf("images/%04v.png", frameNumber)
	if subframe > 0 {
		framePath = fmt.Sprintf("images/%04v.%03v.png", frameNumber, subframe)
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
	info := flag.Bool("info", false, "Only print out number triangles")
	flag.Parse()
	fmt.Printf("subframe: %v, frame: %v, pixels: %v, maxSubdivisions: %v, maxTime: %v, maxFrames: %v\n", *subframe, *frame, *pixels, *maxSubdivisions, *maxTime, *maxFrames)
	dt := math.Pi * 2 / float64(*maxFrames)
	var surfaces []render.Surface
	renderSurfaces(*frame, *subframe, *pixels, *maxSubdivisions, *maxTime, dt, surfaces, *info)
}
