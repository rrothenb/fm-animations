//go:build testbed
// +build testbed

package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"os"
	"text/template"

	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/rrothenb/fm-animations/utils"
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

var tGlobal = 0.0
var frameGlobal = 0

func sign(x float64) float64 {
	if x < 0 {
		return -1
	} else {
		return 1
	}
}
func spow(x, y float64) float64 {
	return sign(x) * pow(abs(x), y)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2 - 1
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
	aspectRatio := s.Width / s.Height
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

func square(x float64) geom.Vec {
	return geom.Vec{sin(x + .3*sin(2*x)), cos(x - .3*sin(2*x)), 0}
}

func sphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v / 2.0),
	}
}

func torusKnot(t, R, r float64, pInt, qInt int, path func(x float64) geom.Vec) geom.Vec {
	p := float64(pInt)
	q := float64(qInt)
	center := path(q * t).Scaled(R)
	delta := .01
	normal, _ := path(q*t + delta).Scaled(R).Minus(path(q*t - delta).Scaled(R)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * cos(p*t)).Plus(sinVec.Scaled(r * sin(p*t)).Plus(center))
}

var factor = .51

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.48125, 3, 2, circle)
}

func middleKnot(t float64) geom.Vec {
	return torusKnot(t, 1, pow(factor, 2), 3, 2, outerKnot)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, pow(factor, 3), 2, 3, middleKnot)
}

func lastKnot(t float64) geom.Vec {
	return torusKnot(t, 1, pow(factor, 5), 3, 2, innerKnot)
}

func lastKnotForRealz(t float64) geom.Vec {
	return torusKnot(t, 1, pow(factor, 5), 3, 2, lastKnot)
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v + delta).Minus(path(v - delta)).Unit()
	sinVec, ok := normal.Cross(geom.Dir{0, 0, 1})
	if !ok { // tangent parallel to z: avoid 0/0 -> NaN ring, use a different up
		sinVec, _ = normal.Cross(geom.Dir{1, 0, 0})
	}
	cosVec, _ := normal.Cross(sinVec)
	a := -.2
	return cosVec.Scaled(r * cos(u-a*sin(2*u))).Plus(sinVec.Scaled(r * sin(u+a*sin(2*u)))).Plus(center)
}

func strength(x float64) float64 {
	return pow(2, sin(x)*2)
}

func texture(a, u, v, t float64) float64 {
	return sin(
		u + v + a*strength(.1+2*t)*sin(
			u+a*strength(.2+3*t)*sin(u)) + a*strength(.3+5*t)*sin(
			2*v+a*strength(.4+7*t)*sin(v)) + a*strength(.5+11*t)*sin(
			u+2*v) + a*strength(.6+13*t)*sin(3*u-v) + a*strength(.7+17*t)*sin(5*v-3*u))
}

func radius(u, v, a float64) float64 {

	return 1.0 + .1*a*shapeTexture(u, v, a)
}

func shapeTexture(u, v, a float64) float64 {
	return sin(sin(4*u) + sin(4*v))
}

func sphereish(u, v, a, b, c float64) geom.Vec {
	return geom.Vec{
		spow(sin(v/2.0), a) * spow(cos(u), 1/b),
		spow(sin(v/2.0), a) * spow(sin(u), b),
		spow(cos(v/2.0), c),
	}
}

func yzTexture(a, y, z, t float64) float64 {
	y = y * pi
	z = z * pi
	return sin(
		pow(texture(a, y, z, t), 2) +
			pow(texture(a, -y, z, t), 2) +
			pow(texture(a, y, -z, t), 2) +
			pow(texture(a, -y, -z, t), 2) +
			pow(texture(a, z, y, t), 2) +
			pow(texture(a, -z, y, t), 2) +
			pow(texture(a, z, -y, t), 2) +
			pow(texture(a, -z, -y, t), 2))
}

func xyzTexture(a, x, y, z, t float64) float64 {
	return pow(spow(sin(
		pow(texture(a, y, z, t), 2)+
			pow(texture(a, -y, z, t), 2)+
			pow(texture(a, y, -z, t), 2)+
			pow(texture(a, -y, -z, t), 2)+
			pow(texture(a, z, y, t), 2)+
			pow(texture(a, -z, y, t), 2)+
			pow(texture(a, z, -y, t), 2)+
			pow(texture(a, -z, -y, t), 2)), pow(2, sin(2*t)))/2+.5, pow(2, sin(3*t))) * spow(x, 4)
}

func displacement(loc geom.Vec, t float64) geom.Vec {
	a := pow(2, sin(2*t)-3)
	xTexture := shape(yzTexture(a, loc.Y, loc.Z, t), sin(7*t), sin(11*t)) * spow(loc.X, 4)
	yTexture := shape(yzTexture(a, loc.X, loc.Z, t), sin(7*t), sin(11*t)) * spow(loc.Y, 4)
	zTexture := shape(yzTexture(a, loc.Y, loc.X, t), sin(7*t), sin(11*t)) * spow(loc.Z, 4)
	return geom.Vec{xTexture, yTexture, zTexture}
}

func fabricPath(t float64) geom.Vec {
	//return geom.Vec{sin(29*t), cos(31*t), .1*cos(2*29*31*t)*cos(29*t)}
	return geom.Vec{sin(11 * t), sin(7 * t), sin((11*7-2)*t) * .15}
}

func threadPath(t float64) geom.Vec {
	return torusKnot(t, 1, .01, 7, 2, fabricPath)
}

func symmetricalKnot(t float64) geom.Vec {
	a := sin(2 * tGlobal)
	b := sin(3 * tGlobal)
	c := sin(5 * tGlobal)
	return circle(t).Plus(geom.Vec{cos(t - a*sin(2*t)), cos(t - b*sin(2*t)), cos(t - c*sin(2*t))}.Scaled(.75))
}

func shape(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

func sqrsin(t float64) float64 {
	return sin(t + .6*sin(2*t))
}

func sqrcos(t float64) float64 {
	return cos(t - .6*sin(2*t))
}

func box(u, v, t float64) geom.Vec {
	return geom.Vec{
		sqrsin(v/2.0) * sqrcos(u),
		sqrsin(v/2.0) * sqrsin(u),
		sqrcos(v / 2.0),
	}
}

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{
		sin(float64(xN) * t),
		sin(float64(yN) * t),
		cos(float64(zN) * t)}
}

func irrationalLissajousKnot(t float64) geom.Vec {
	xF := -cos(2*tGlobal)*5 + 16
	yF := -cos(3*tGlobal)*5 + 18
	zF := xF*yF - 2
	return geom.Vec{
		cos(xF * t),
		sin(yF * t),
		sin(zF*t) * .1}
}

func irrationalXYZTexture(a, f, x, y, z, t float64) float64 {
	a1 := f*sin(2*t)*.5 + .5
	a2 := f*sin(3*t)*.5 + .5
	a3 := f*sin(5*t)*.5 + .5
	a4 := a*sin(7*t)*.5 + .5
	a5 := f*sin(11*t)*.5 + .5
	a6 := f*sin(13*t)*.5 + .5
	a7 := f*sin(17*t)*.5 + .5
	a8 := a*sin(19*t)*.5 + .5
	a9 := f*sin(23*t)*.5 + .5
	a10 := f*sin(29*t)*.5 + .5
	a11 := f*sin(31*t)*.5 + .5
	return sin(a1*x + a2*y + a3*z + a4*sin(a5*x+a6*y+a7*z+a8*sin(a9*x+a10*y+a11*z)))
}
func rationalUVTexture(a, fU, fV, u, v, t float64) float64 {
	a1 := floor(fU*sin(2*t)*.5 + .5)
	a2 := floor(fV*sin(3*t)*.5 + .5)
	a3 := a*sin(5*t)*.5 + .5
	a4 := floor(fU*sin(7*t)*.5 + .5)
	a5 := floor(fV*sin(11*t)*.5 + .5)
	a6 := a*sin(13*t)*.5 + .5
	a7 := floor(fU*sin(17*t)*.5 + .5)
	a8 := floor(fV*sin(19*t)*.5 + .5)
	a9 := a*sin(23*t)*.5 + .5
	a10 := floor(fU*sin(29*t)*.5 + .5)
	a11 := floor(fV*sin(31*t)*.5 + .5)
	return sin(a1*u + a2*v + a3*sin(a4*u+a5*v+a6*sin(a7*u+a8*v+a9*sin(a10*u+a11*v))))
}

func symmetricalXYZTexture(a, f, x, y, z, t float64) float64 {
	return pow(irrationalXYZTexture(a, f, x, y, z, t), 2) +
		pow(irrationalXYZTexture(a, f, -x, y, z, t), 2) +
		pow(irrationalXYZTexture(a, f, x, -y, z, t), 2) +
		pow(irrationalXYZTexture(a, f, x, y, -z, t), 2)
}

func symmetricalUVTexture(a, fU, fV, u, v, t float64) float64 {
	return pow(rationalUVTexture(a, fU, fV, u, v, t), 2) +
		pow(rationalUVTexture(a, fU, fV, -u, v, t), 2) +
		pow(rationalUVTexture(a, fU, fV, u, -v, t), 2) +
		pow(rationalUVTexture(a, fU, fV, -u, -v, t), 2) +
		pow(rationalUVTexture(a, fU, fV, v, u, t), 2) +
		pow(rationalUVTexture(a, fU, fV, -v, u, t), 2) +
		pow(rationalUVTexture(a, fU, fV, v, -u, t), 2) +
		pow(rationalUVTexture(a, fU, fV, -v, -u, t), 2)
}

func knot(t float64) geom.Vec {
	return torusKnot(t, 1, .45-.001, 3, 2, circle)
}

func cube(u, v, t float64) geom.Vec {
	a := .333 * float64((frameGlobal%2)*2-1)
	b := .333 * float64(((frameGlobal/2)%2)*2-1)
	return geom.Vec{
		sin(v/2.0+a*sin(v+b*sin(v))) * cos(u-a*sin(2*u+b*sin(2*u))),
		sin(v/2.0+a*sin(v+b*sin(v))) * sin(u+a*sin(2*u+b*sin(2*u))),
		cos(v/2.0 - a*sin(v+b*sin(v))),
	}
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	return pathWrapper(u, v, 0.48125, outerKnot)
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t, radius)
	right := uv2xyz(index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t, radius)
	up := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t, radius)
	down := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t, radius)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	fmt.Println("radius", .475+t/2/pi/100)
	rMax, rCurv, rSep := utils.MaxTubeRadius(outerKnot, 2*pi, 4000)
	fmt.Printf("max tube radius for outerKnot: %.4f (curvature %.4f, separation %.4f)\n", rMax, rCurv, rSep)
	fmt.Println("frame", frameNumber)
	tGlobal = t
	frameGlobal = frameNumber
	cameraLoc := geom.Vec{0, 0, 9}
	focusPoint := geom.Vec{0, 0, 0}
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	nU := int(float64(pixels) / distance * 3)
	nV := nU
	minZ := 0.0
	if desiredTriangles > 0 {
		numTriangles := 0
		totalWidth := 0.0
		totalHeight := 0.0
		for uIndex := 0; uIndex <= 500; uIndex++ {
			for vIndex := 0; vIndex <= 500; vIndex++ {
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius)
				if !c.invisible(vertex) {
					numTriangles++
					vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t, radius)
					vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t, radius)
					totalWidth += vertex.Minus(vertexLeft).Len()
					totalHeight += vertex.Minus(vertexBelow).Len()
					minZ = math.Min(minZ, vertex.Z)
				}
			}
		}
		fmt.Println(numTriangles, minZ)
		ratio := totalWidth / totalHeight
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, nU: %v, nV: %v\n", cameraLoc.Len(), distance, nU, nV)
	vertexIndicies := make([][]int32, nU+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("testbed.data.ply")
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= nU; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		for vIndex := 0; vIndex <= nV; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, radius)
			if c.invisible(vertex) {
				vertexIndicies[uIndex][vIndex] = -1
				continue
			}
			normal := uvIndexToNormal(uIndex, vIndex, nU, nV, t)
			vertexIndicies[uIndex][vIndex] = int32(numVerticies)
			numVerticies++
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex), nU)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex), nV)/pi/2))
		}
	}
	numFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
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
