//go:build ignore
// +build ignore

package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"text/template"
	"encoding/binary"
	"bufio"

	"github.com/Opioid/rgbe"
	"github.com/hunterloftis/pbr/pkg/geom"
)

type MeshType struct {
	NumVertices int
	NumFaces    int
}

var sin = math.Sin
var cos = math.Cos
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi

func strength(x float64) float64 {
	return sin(x)*.75 + .85
}

func texture(u, v, t float64) float64 {
	return sin(
		3*u + 5*v + strength(.1+2*t)*sin(
			2*u+strength(.2+3*t)*sin(3*u)) + strength(.3+5*t)*sin(
			7*v+strength(.4+7*t)*sin(5*v)) + strength(.5+11*t)*sin(
			11*u+13*v) + strength(.6+13*t)*sin(17*u-5*v) + strength(.7+17*t)*sin(23*v-11*u))
}

func radius(u, v, t float64) float64 {
	return 1.0 + .075*pow(pow(texture(u, v, t), 2), 3)
}

type SLR2 struct {
	Width  float64
	Height float64
	Lens   float64
	FStop  float64
	Focus  float64

	trans    *geom.Mtx
	position geom.Vec
	target   geom.Vec
}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
		Width:    0.024,
		Height:   0.024,
		Lens:     0.050, // 50mm focal length
		FStop:    4,
		Focus:    1,
		position: geom.Vec{0, 0, 0},
		target:   geom.Vec{0, 0, -5},
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
	if projectedPoint.X < projectedPoint.Z/2 || projectedPoint.X > -projectedPoint.Z/2 {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z/6 || projectedPoint.Y > -projectedPoint.Z/6 {
		return true
	}
	return false
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	return geom.Vec{
		sin(v/2.0) * cos(u) * a,
		sin(v/2.0) * sin(u) * a,
		cos(v/2.0) * a,
	}
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, n int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)+.1, n), t, radius).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)-.1, n), t, radius).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	// this should speed up at t around pi
	cameraT := 0.0
	cameraLoc := geom.Vec{sin(cameraT), sin(cameraT), cos(cameraT)}.Scaled(.3).Plus(geom.Vec{0.0, 0.0, -.1})
	unitCameraLoc, _ := cameraLoc.Unit()
	focusPoint := unitCameraLoc.Scaled(.075)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
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
				if !c.invisible(vertex) {
					numTriangles++
				}
			}
		}
		fmt.Println(numTriangles)
		n = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)) * 500)
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v\n", cameraLoc.Len(), distance, n)
	vertexIndicies := make([][]int32, n+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= n; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, n+1)
		for vIndex := 0; vIndex <= n; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
			if c.invisible(vertex) {
				vertexIndicies[uIndex][vIndex] = -1
				continue
			}
			normal := uvIndexToNormal(uIndex, vIndex, n, t)
			vertexIndicies[uIndex][vIndex] = int32(numVerticies)
			numVerticies++
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Z))
		}
	}
	envmapArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			u := float64(uIndex) / float64(n) * 2 * pi
			v := float64(vIndex) / float64(n) * pi
			envmapValue := float32(1-pow(1-pow(texture(u, v, t), 2), pow(1-v/pi, 7)*50)) * 255
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)

			topRight := vertexIndicies[uIndex][vIndex]
			topLeft := vertexIndicies[uIndex+1][vIndex]
			botRight := vertexIndicies[uIndex][vIndex+1]
			botLeft := vertexIndicies[uIndex+1][vIndex+1]
			if topRight == -1 || topLeft == -1 || botRight == -1 || botLeft == -1 {
				continue
			}
			if vIndex != 0 {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
				binary.Write(PlyDataBuffered, binary.LittleEndian, topLeft)
				numFaces++
			}
			if vIndex != n-1 {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
				numFaces++
			}
		}
	}

	fmt.Println("Mitsuba!")
	PlyDataBuffered.Flush()
	tmpl, _ := template.New("some template").Parse(`
ply
format binary_little_endian 1.0
element vertex {{ .NumVertices }}
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
element face {{ .NumFaces }}
property list uint8 int32 vertex_index
end_header
`)
	plyHeaderPath := fmt.Sprintf("data/%v.header.ply", frameNumber)
	envPath := fmt.Sprintf("data/%v.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, n, n, envmapArray)
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 5000, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
