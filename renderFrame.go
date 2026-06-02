//go:build ignore
// +build ignore

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
	"text/template"

	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/hunterloftis/pbr/pkg/material"
	"github.com/hunterloftis/pbr/pkg/render"
	"github.com/hunterloftis/pbr/pkg/rgb"
	"github.com/hunterloftis/pbr/pkg/surface"
	"github.com/Opioid/rgbe"
)

type Face struct {
	A int
	B int
	C int
}

type Vertex struct {
	Vertex geom.Vec
	Normal geom.Dir
}

type MeshType struct {
	Vertices []Vertex
	Faces []Face
	Thing int
	Stuff string
}

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

func (s *SLR2) invisibleToSubframe(point geom.Vec) bool {
	if s.invisible(point) {
		return true
	}
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	subframeSize := -projectedPoint.Z/6/5
	xOffset := float64(s.subframeCol)*subframeSize
	yOffset := float64(s.subframeRow)*subframeSize
	if projectedPoint.X < xOffset - subframeSize - subframeSize*1.5 || projectedPoint.X > xOffset + subframeSize*1.5 {
		return true
	}
	if projectedPoint.Y < yOffset - subframeSize - subframeSize*1.5 || projectedPoint.Y > yOffset + subframeSize*1.5 {
		return true
	}
	return false
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
	if projectedPoint.Y < projectedPoint.Z/7 || projectedPoint.Y > -projectedPoint.Z/7 {
		return true
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

func index2radians(index float64, n int) float64 {
	return index / float64(n) * math.Pi * 2
}

type FMMaterial struct {
	uTexture float32
	vTexture float32
	wTexture float32
}

func NewFMMaterial(uTexture, vTexture, wTexture float64) *FMMaterial{
	return &FMMaterial{float32(uTexture), float32(vTexture),float32(wTexture)}
}

func (m *FMMaterial) At(u, v float64, in, norm geom.Dir, rnd *rand.Rand) (geom.Dir, render.BSDF) {
	w := 1 - u - v
	fm := float64(m.uTexture)*u+float64(m.vTexture)*v+float64(m.wTexture)*w
	fm4 := math.Pow(fm, 4)
	roughness := fm4*.09 + .01
	metalness := (1-fm4)*.8 + .1
	specularity := fm4*.3 + .1
	color := rgb.Energy{
		1,
		.5 + .05*fm,
		.25 + .15*math.Sin(4*fm)}.Scaled(fm4).Plus(rgb.Energy{.8, .8, .8}.Scaled(1 - fm4))
	mat := material.Uniform{
		Color:       color,
		Metalness:   metalness,
		Roughness:   roughness,
		Specularity: specularity,
	}
	return mat.At(u, v, in, norm, rnd)
}

func (m *FMMaterial) Light() rgb.Energy {
	return rgb.Black
}

func (m *FMMaterial) Transmit() rgb.Energy {
	return rgb.Black
}

func uvIndexToNormal(uIndex, vIndex, n int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)+.1, n), t, radius).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)-.1, n), t, radius).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, subframe int, pixels int, maxSubdivisions int, maxTime int, dt float64, surfaces []render.Surface, info bool, desiredTriangles int, mitsuba bool) {
	t := float64(frameNumber) * dt
	// this should speed up at t around pi
	cameraT := 0.0
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
	if desiredTriangles > 0 {
		numTriangles := 0
		for uIndex := 0; uIndex <= 500; uIndex++ {
			for vIndex := 0; vIndex <= 500; vIndex++ {
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
				if subframe > 0 {
					if !c.invisibleToSubframe(vertex) {
						numTriangles++
					} else if vIndex%6 == 0 && uIndex%6 == 0 {
						if !c.invisible(vertex) {
							numTriangles++
						}
					}
				} else {
					if !c.invisible(vertex) {
						numTriangles++
					}
				}
			}
		}
		fmt.Println(numTriangles)
		n = int(math.Sqrt(float64(desiredTriangles)/float64(numTriangles*2))*500)
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v, maxTime: %v\n", cameraLoc.Len(), distance, n, maxTime)
	vertices := make([][]geom.Vec, n+1)
	textures := make([][]float64, n+1)
	normals := make([][]*geom.Dir, n+1)
	vertexIndicies := make([][]int, n+1)
	numVerticies := 0
	verticiesArray := []Vertex{}
	faces := []Face{}
	for uIndex := 0; uIndex <= n; uIndex++ {
		vertices[uIndex] = make([]geom.Vec, n+1)
		textures[uIndex] = make([]float64, n+1)
		normals[uIndex] = make([]*geom.Dir, n+1)
		vertexIndicies[uIndex] = make([]int, n+1)
		for vIndex := 0; vIndex <= n; vIndex++ {
			vertices[uIndex][vIndex] = uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
			if c.invisible(vertices[uIndex][vIndex]) {
				continue
			}
			textures[uIndex][vIndex] = texture(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t)
			normals[uIndex][vIndex] = uvIndexToNormal(uIndex, vIndex, n, t)
			vertexIndicies[uIndex][vIndex] = numVerticies
			numVerticies++
			verticiesArray = append(verticiesArray, Vertex{vertices[uIndex][vIndex], *normals[uIndex][vIndex]})
		}
	}
	fmt.Printf("\nnumVerticies: %v, len(verticiesArray): %v\n", numVerticies, len(verticiesArray))
	// if subframe loop but jumping by 10 instead of 1 and include stuff not visible to subframe but still visible
	if subframe > 0 {
		for vIndex := 0; vIndex < n-6; vIndex+=6 {
			for uIndex := 0; uIndex < n-6; uIndex+=6 {
				topRight := vertices[uIndex][vIndex]
				topLeft := vertices[uIndex+6][vIndex]
				botRight := vertices[uIndex][vIndex+6]
				botLeft := vertices[uIndex+6][vIndex+6]
				if c.invisible(topRight) || c.invisible(topLeft) || c.invisible(botRight) || c.invisible(botLeft) {
					continue
				}
				if !c.invisibleToSubframe(topRight) && !c.invisibleToSubframe(topLeft) && !c.invisibleToSubframe(botRight) && !c.invisibleToSubframe(botLeft) {
					continue
				}
				topRightTexture := textures[uIndex][vIndex]
				topRightNormal := normals[uIndex][vIndex]
				topLeftTexture := textures[uIndex+6][vIndex]
				topLeftNormal := normals[uIndex+6][vIndex]
				botRightTexture := textures[uIndex][vIndex+6]
				botRightNormal := normals[uIndex][vIndex+6]
				botLeftTexture := textures[uIndex+6][vIndex+6]
				botLeftNormal := normals[uIndex+6][vIndex+6]
				if vIndex != 0 {
					m := NewFMMaterial(topRightTexture, botLeftTexture, topLeftTexture)
					triangle := surface.NewTriangle(topRight, botLeft, topLeft, m)
					triangle.Texture[0] = geom.Vec{1,0,0}
					triangle.Texture[1] = geom.Vec{0,1,0}
					triangle.SetNormals(*topRightNormal, *botLeftNormal, *topLeftNormal)
					surfaces = append(surfaces, triangle)
				}
				m := NewFMMaterial(topRightTexture, botRightTexture, botLeftTexture)
				triangle := surface.NewTriangle(topRight, botRight, botLeft, m)
				triangle.Texture[0] = geom.Vec{1,0,0}
				triangle.Texture[1] = geom.Vec{0,1,0}
				triangle.SetNormals(*topRightNormal, *botRightNormal, *botLeftNormal)
				surfaces = append(surfaces, triangle)
			}
		}
	}
	e := &FMEnv{t}
	envmapArray := []float32{}
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			u := float64(uIndex)/float64(n)*2*math.Pi
			v := float64(vIndex)/float64(n)*math.Pi
			envmapValue := float32(1 - math.Pow(1-math.Pow(texture(u, v, e.t), 2), math.Pow(1-v/math.Pi, 7)*50))*255
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
			topRight := vertices[uIndex][vIndex]
			topLeft := vertices[uIndex+1][vIndex]
			botRight := vertices[uIndex][vIndex+1]
			botLeft := vertices[uIndex+1][vIndex+1]
			if subframe > 0 {
				if c.invisibleToSubframe(topRight) || c.invisibleToSubframe(topLeft) || c.invisibleToSubframe(botRight) || c.invisibleToSubframe(botLeft) {
					continue
				}
			} else {
				if c.invisible(topRight) || c.invisible(topLeft) || c.invisible(botRight) || c.invisible(botLeft) {
					continue
				}
			}
			topRightTexture := textures[uIndex][vIndex]
			topRightNormal := normals[uIndex][vIndex]
			topLeftTexture := textures[uIndex+1][vIndex]
			topLeftNormal := normals[uIndex+1][vIndex]
			botRightTexture := textures[uIndex][vIndex+1]
			botRightNormal := normals[uIndex][vIndex+1]
			botLeftTexture := textures[uIndex+1][vIndex+1]
			botLeftNormal := normals[uIndex+1][vIndex+1]
			if vIndex != 0 {
				m := NewFMMaterial(topRightTexture, botLeftTexture, topLeftTexture)
				triangle := surface.NewTriangle(topRight, botLeft, topLeft, m)
				triangle.Texture[0] = geom.Vec{1,0,0}
				triangle.Texture[1] = geom.Vec{0,1,0}
				triangle.SetNormals(*topRightNormal, *botLeftNormal, *topLeftNormal)
				surfaces = append(surfaces, triangle)
				faces = append(faces, Face{vertexIndicies[uIndex][vIndex], vertexIndicies[uIndex+1][vIndex+1], vertexIndicies[uIndex+1][vIndex]})
			}
			if vIndex != n-1 {
				m := NewFMMaterial(topRightTexture, botRightTexture, botLeftTexture)
				triangle := surface.NewTriangle(topRight, botRight, botLeft, m)
				triangle.Texture[0] = geom.Vec{1,0,0}
				triangle.Texture[1] = geom.Vec{0,1,0}
				triangle.SetNormals(*topRightNormal, *botRightNormal, *botLeftNormal)
				surfaces = append(surfaces, triangle)
				faces = append(faces, Face{vertexIndicies[uIndex][vIndex], vertexIndicies[uIndex][vIndex+1], vertexIndicies[uIndex+1][vIndex+1]})
			}
		}
	}
	fmt.Println(len(surfaces), n*n*2)

	if info {
		return
	}

	if mitsuba {
		fmt.Println("Mitsuba!")
		t, _ := template.New("some template").Parse(`
ply
format ascii 1.0
element vertex {{ .Vertices | len }}
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
element face {{ .Faces | len }}
property list uint8 int32 vertex_index
end_header
{{ range .Vertices }}{{.Vertex.X}} {{.Vertex.Y}} {{.Vertex.Z}} {{.Normal.X}} {{.Normal.Y}} {{.Normal.Z}}
{{ end }}
{{ range .Faces }}3 {{.A}} {{.B}} {{.C}}
{{ end }}
`)
		f, _ := os.Create("mitsuba.ply")
		mesh := MeshType{}
		mesh.Vertices = verticiesArray
		mesh.Faces = faces
		t.Execute(f, mesh)
		envmap, _ := os.Create("mitsuba.rgbe")
		rgbe.Encode(envmap, n, n, envmapArray)
		return
	}

	s := surface.NewTree(surfaces...)

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
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	mitsuba := flag.Bool("mitsuba", false, "Only generate files for Mitsuba")
	flag.Parse()
	fmt.Printf("subframe: %v, frame: %v, pixels: %v, maxSubdivisions: %v, maxTime: %v, maxFrames: %v\n", *subframe, *frame, *pixels, *maxSubdivisions, *maxTime, *maxFrames)
	dt := math.Pi * 2 / float64(*maxFrames)
	var surfaces []render.Surface
	renderSurfaces(*frame, *subframe, *pixels, *maxSubdivisions, *maxTime, dt, surfaces, *info, *desiredTriangles, *mitsuba)
}
