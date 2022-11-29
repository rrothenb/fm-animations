package main

import (
	"flag"
	"fmt"
	"math"
	"os"
	"text/template"
	"encoding/binary"
	"bufio"

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
var abs = math.Abs
var min = math.Min
var max = math.Max
var floor = math.Floor

func sign(x float64) float64 {
	if x < 0 {
		return -1
	} else {
		return 1
	}
}
func spow(x, y float64) float64 {
	return sign(x)*pow(abs(x), y)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2-1
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

var zAxis = geom.Dir{0, 0, 1}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
		Width:    0.048,
		Height:   0.027,
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

func LookMatrix(o geom.Vec, to geom.Vec) *geom.Mtx {
	f, _ := o.Minus(to).Unit() // forward
	r, _ := zAxis.Cross(f)     // right
	u, _ := f.Cross(r)         // up
	orient := geom.NewMat(
		r.X, u.X, f.X, 0,
		r.Y, u.Y, f.Y, 0,
		r.Z, u.Z, f.Z, 0,
		0, 0, 0, 1,
	)
	return geom.Shift(o).Mult(orient)
}

func (s *SLR2) transform() {
	s.trans = LookMatrix(s.position, s.target)
}

func (s *SLR2) invisible(point geom.Vec) bool {
	return false
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	//fmt.Printf("\npoint: %#v\nprojectedPoint: %#v\ncameraSpaceTransform: %#v\n", point, projectedPoint, cameraSpaceTransform)
	factor := .01
	aspectRatio := s.Width/s.Height
	if projectedPoint.X < projectedPoint.Z*factor*aspectRatio || projectedPoint.X > -projectedPoint.Z*factor*aspectRatio {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
	return false
	if projectedPoint.Z > 0.0 {
		return true
	}
	return false
	if projectedPoint.Z < -s.position.Len() {
		return true
	}
	if projectedPoint.X < projectedPoint.Z/2 || projectedPoint.X > -projectedPoint.Z/2 {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z/2 || projectedPoint.Y > -projectedPoint.Z/2 {
		return true
	}
	return false
}

func circle(x float64) geom.Vec {
	return geom.Vec{sin(x), cos(x), 0}
}

func sphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v/2.0),
	}
}

func torusKnot(t, R, r float64, pInt, qInt int, path func(x float64) geom.Vec) geom.Vec {
	p := float64(pInt)
	q := float64(qInt)
	pathPoint := path(q*t)
	return geom.Vec{(R+r*cos(p*t))*pathPoint.X, (R+r*cos(p*t))*pathPoint.Y, r*sin(p*t)+pathPoint.Z}
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .45, 2, 3, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .35, 3, 2, outerKnot)
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	u = u + 0
	delta := .01
	center := path(v)
	normal, _ := path(v+delta).Minus(path(v-delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r*cos(u-.5*sin(2*u))).Plus(sinVec.Scaled(r*sin(u+.5*sin(2*u)))).Plus(center)
}

func strength(x float64) float64 {
	return pow(2, sin(x)*2)
}

func radius(u, v, a float64) float64 {

	return 1.0 + .1*a*shapeTexture(u, v, a)
}

func shapeTexture(u, v, a float64) float64 {
	return sin(sin(4*u)+sin(4*v))
}

func sphereish(u, v, a float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-a*spow(sin(2*u), 1)),
		sin(v/2.0+a*spow(sin(v),1)) * sin(u+a*spow(sin(2*u), 1)),
		cos(v/2.0-a*spow(sin(v),1)),
	}
}

func texture(u, v, t float64) float64 {
	a := .5
	minT := pi*.975
	maxT := pi*1.025
	t = minT + t/2/pi*(maxT-minT)
	return sin(
		10*u + a*strength(.1+2*t)*sin(
			2*u+a*strength(.2+3*t)*sin(3*u)) + a*strength(.3+5*t)*sin(
			2*v+a*strength(.4+7*t)*sin(-v)) + a*strength(.5+11*t)*sin(
			5*u-v) + a*strength(.6+13*t)*sin(7*u-v) + a*strength(.7+17*t)*sin(v-3*u))
}

func fabricPath(t float64) geom.Vec {
	//return geom.Vec{sin(29*t), cos(31*t), .1*cos(2*29*31*t)*cos(29*t)}
	return geom.Vec{sin(29*t), sin(31*t), sin((29*31-2)*t)*.1}
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	return pathWrapper(u, v, .05, fabricPath).By(geom.Vec{1, 1, .1})
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, n int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, n), index2radians(float64(vIndex), n), t, radius)
	right := uv2xyz(index2radians(float64(uIndex)+.1, n), index2radians(float64(vIndex), n), t, radius)
	up := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)+.1, n), t, radius)
	down := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)-.1, n), t, radius)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	cameraLoc := geom.Vec{0, 0, 9}
	focusPoint := geom.Vec{0, 0, 0}
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
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius)
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
	plyDataPath := fmt.Sprintf("testbed.data.ply")
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= n; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, n+1)
		for vIndex := 0; vIndex <= n; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t, radius)
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
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex), n)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex), n)/pi/2))
		}
	}
	numFaces := 0
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			topRight := vertexIndicies[uIndex][vIndex]
			topLeft := vertexIndicies[uIndex+1][vIndex]
			botRight := vertexIndicies[uIndex][vIndex+1]
			botLeft := vertexIndicies[uIndex+1][vIndex+1]
			if topRight == -1 || topLeft == -1 || botRight == -1 || botLeft == -1 {
				continue
			}
			binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
			binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
			binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
			binary.Write(PlyDataBuffered, binary.LittleEndian, topLeft)
			numFaces++
			binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
			binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
			binary.Write(PlyDataBuffered, binary.LittleEndian, botRight)
			binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
			numFaces++
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
property float32 u
property float32 v
element face {{ .NumFaces }}
property list uint8 int32 vertex_index
end_header
`)
	plyHeaderPath := fmt.Sprintf("testbed.header.ply")
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 8, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
