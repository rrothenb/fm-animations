//go:build series27
// +build series27

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
	"github.com/hunterloftis/pbr/pkg/surface"
	"github.com/hunterloftis/pbr/pkg/material"
)

type MeshType struct {
	NumVertices int
	NumFaces    int
}

var maxX = 0.0
var maxY = 0.0
var maxZ = 0.0
var minX = 0.0
var minY = 0.0
var minZ = 0.0

func sin(x float64) float64 {
	return math.Sin(x*2*pi)
}

func cos(x float64) float64 {
	return math.Cos(x*2*pi)
}

func xyzTexture(u, v, tPercent float64, texture func (u, v, tPercent float64) float64, shape func (u, v, tPercent float64) geom.Vec) float64 {
	loc := shape(u, v, tPercent).Scaled(.5).Plus(geom.Vec{.5, .5, .5})
	return (texture(loc.X, loc.Y, tPercent)+texture(loc.X, loc.Z, tPercent)+texture(loc.Z, loc.Y, tPercent))/3
}

var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var min = math.Min
var max = math.Max
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

// inputs are in range 0 to 1 for most of these

func strength(x float64) float64 {
	return sin(x)+1.1
}

func texture(u, v, t float64) float64 {
	perturbation := perturbation(u, v, t)
	return sin(
		3*u + 3*v +
			strength(.1+2*t)*sin(5*u+strength(.2+3*t)*sin(3*u)) +
			strength(.1+2*t)*sin(5*v+strength(.2+3*t)*sin(3*v)) +
			strength(.3+5*t)*sin(2*u-3*v) +
			strength(.3+5*t)*sin(2*v-3*u) +
			strength(.4+7*t)*perturbation)
}

func baseShape(u, v, t float64) geom.Vec {
	return pathWrapper(u, v, .01, knot)
}

func radius(u, v, t float64) float64 {
	return .95 + .3*xyzTexture(u, v, t, perturbation, baseShape)
}

func perturbation(u, v, t float64) float64 {
	return sin(
		u*v*(u-v)*(v-u)*(u+v) +
			.15*strength(.1+2*t)*sin(3*u-2*v+.15*strength(.2+3*t)*sin(2*u)+.15*strength(.3+5*t)*sin(11*v+.15*strength(.4+7*t)*sin(2*u+7*v))) +
			.15*strength(.1+2*t)*sin(5*v-3*u+.15*strength(.2+3*t)*sin(7*v)+.15*strength(.3+5*t)*sin(3*u+.15*strength(.4+7*t)*sin(13*u+11*v))))
}

func blendTexture(u, v, t float64) float64 {
	return 1-pushout(pow(perturbation(u, v, t)/2+.5, 2), .1)
}

func metalBlendTexture(u, v, t float64) float64 {
	return 1-pushout(pow(perturbation(u, v, t)/2+.5, 2), .5)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2-1
}

func pushout(x, n float64) float64 {
	return spow(x*2-1, n)/2+.5
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
		Width:    0.036,
		Height:   0.036,
		Lens:     0.050, // 50mm focal length
		FStop:    4,
		Focus:    1,
		position: geom.Vec{0, 0, 0},
		target:   geom.Vec{0, 0, -5},
	}
	fmt.Println("constructor")
	s.transform()
	return s
}

// LookAt orients a Camera to face a target.
func (s *SLR2) LookAt(target geom.Vec) *SLR2 {
	s.target = target
	fmt.Println("LookAt")
	s.transform()
	return s
}

// MoveTo moves a Camera to a position given by x, y, and z coordinates.
func (s *SLR2) MoveTo(pos geom.Vec) *SLR2 {
	s.position = pos
	fmt.Println("MoveTo")
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
	fmt.Printf("\nf: %#v\nr: %#v\nu: %#v\norient: %#v\n", f, r, u, orient)
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
	factor := .4
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
	if projectedPoint.Z < -s.position.Len() {
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

func foldedSphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u) * (1.25 - pow(cos(5*v/2.0), 2)),
		sin(v/2.0) * sin(u) * (1.25 - pow(cos(5*v/2.0), 2)),
		cos(5*v/2.0),
	}
}

func unitStrength(x float64) float64 {
	return .5-cos(x)/2
}

func squareSphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+sin(2*t+.8)*sin(v)) * cos(u-sin(3*t+.8)*sin(2*u)),
		sin(v/2.0+sin(2*t+.8)*sin(v)) * sin(u+sin(3*t+.8)*sin(2*u)),
		cos(v/2.0-sin(5*t+.8)*sin(v)),
	}
}

func foldedGeneralizedSphere(u, v, tPercent float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+unitStrength(2*tPercent)/2*sin(v)) * cos(u-unitStrength(3*tPercent)/2*sin(2*u)) * (1 - pow(cos(7*v/2.0), 2)),
		sin(v/2.0+unitStrength(2*tPercent)/2*sin(v)) * sin(u+unitStrength(3*tPercent)/2*sin(2*u)) * (1 - pow(cos(7*v/2.0), 2)),
		cos(7*v/2.0),
	}
}

func layeredSphere(u, v, tPercent float64, layers int, power float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * spow(sin(float64(layers)*v), power) * cos(u),
		sin(v/2.0) * spow(sin(float64(layers)*v), power) * sin(u),
		cos(v/2.0),
	}
}

// maybe torusKnot should have a path input and for a regular torus knot it's a circle but for a cable know it's a torusKnot
func torusKnot(tPercent, R, r float64, pInt, qInt int, path func(x float64) geom.Vec) geom.Vec {
	p := float64(pInt)
	q := float64(qInt)
	pathPoint := path(q*tPercent)
	return geom.Vec{(R+r*cos(p*tPercent))*pathPoint.X, (R+r*cos(p*tPercent))*pathPoint.Y, r*sin(p*tPercent)+pathPoint.Z}
}

func lissajousKnot(tPercent float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{sin(float64(xN)*tPercent), sin(float64(yN)*tPercent), cos(float64(zN)*tPercent)}
}

func unitLissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	point, _ := lissajousKnot(t, xN, yN, zN).Unit()
	return geom.Vec(point)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .75+.1*strength(2*t), 4, 7, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5+.025*strength(3*t), 29, 11, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	t = pushout(t, .8)
	return circle(t).Scaled(.875+sin(t)*.01)
}

func focusPath(t float64) geom.Vec {
	t = pushout(t, .8)
	return cameraPath(t+.15+sin(t)*.05)
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v+delta).Minus(path(v-delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r*cos(u)).Plus(sinVec.Scaled(r*sin(u))).Plus(center)
}

func knot(t float64) geom.Vec {
	return torusKnot(t, .8, .1, 99, 29, circle)
}

func normalize(x, min, max float64) float64 {
	return (x-min)/(max-min)
}

func myLayeredSphere(u, v, t float64) geom.Vec {
	return layeredSphere(u, v, t, 7, pow(10, sin(7*t)))
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	loc := pathWrapper(u, v, .01*radius(u, v, t), knot)
	return loc.Plus(geom.Vec{
		spow(perturbation(loc.Y, loc.Z, t), pow(10,sin(4*t)*.5-.5)),
		spow(perturbation(loc.X, loc.Z, t), pow(10,sin(3*t)*.5-.5)),
		spow(perturbation(loc.Y, loc.X, t), pow(10,sin(5*t)*.5-.5)),
	}.Scaled(.003+sin(7*t)*.001))
}

func index2percent(index float64, n int) float64 {
	return index / float64(n)
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2percent(float64(uIndex)-.1, nU), index2percent(float64(vIndex), nV), t, radius)
	right := uv2xyz(index2percent(float64(uIndex)+.1, nU), index2percent(float64(vIndex), nV), t, radius)
	up := uv2xyz(index2percent(float64(uIndex), nU), index2percent(float64(vIndex)+.1, nV), t, radius)
	down := uv2xyz(index2percent(float64(uIndex), nU), index2percent(float64(vIndex)-.1, nV), t, radius)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	// angle := t*360 - 180
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	distance := cameraLoc.Minus(focusPoint).Len() - .23
	fmt.Printf("\ncameraLoc: %v\nfocusPoint: %v\ndistance: %v\nt: %#v\n", cameraLoc, focusPoint, distance, t, c)
	nU := int(float64(pixels) / distance * 3)
	if nU > maxSubdivisions {
		nU = maxSubdivisions
	}
	nV := nU
	numTriangles := 0
	totalWidth := 0.0
	totalHeight := 0.0
	minDistance := 100.0
	maxDistance := 0.0
	minU := 500
	minV := 500
	maxU := 0
	maxV := 0
	closestPoint := geom.Vec{0,0,0}
	for uIndex := 0; uIndex <= 500; uIndex++ {
		for vIndex := 0; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2percent(float64(uIndex), 500), index2percent(float64(vIndex), 500), t, radius)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2percent(float64(uIndex-1), 500), index2percent(float64(vIndex), 500), t, radius)
				vertexBelow := uv2xyz(index2percent(float64(uIndex), 500), index2percent(float64(vIndex-1), 500), t, radius)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				minX = math.Min(minX, vertex.X)
				minY = math.Min(minY, vertex.Y)
				minZ = math.Min(minZ, vertex.Z)
				maxX = math.Max(maxX, vertex.X)
				maxY = math.Max(maxY, vertex.Y)
				maxZ = math.Max(maxZ, vertex.Z)
				if (minU > uIndex) {
					minU = uIndex
				}
				if (minV > vIndex) {
					minV = vIndex
				}
				if (maxU < uIndex) {
					maxU = uIndex
				}
				if (maxV < vIndex) {
					maxV = vIndex
				}
				if (cameraLoc.Minus(closestPoint).Len() > cameraLoc.Minus(vertex).Len()) {
					closestPoint = vertex
				}
			}
		}
	}
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	dir, _ := focusPoint.Minus(cameraLoc).Unit()
	_, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v\n", minDistance, maxDistance, distance, cameraLoc.Len())
	fmt.Printf("maxX: %v, maxY: %v, maxZ: %v\n", maxX, maxY, maxZ)
	fmt.Printf("minX: %v, minY: %v, minZ: %v\n", minX, minY, minZ)
	ratio := totalWidth/totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	fmt.Println(nU, nV)
	for nV > 30000 {
		ratio = ratio*10
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
	fmt.Println(nU, nV)
	startUIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * float64(minU))
	endUIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * float64(maxU))
	startVIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * float64(minV))
	endVIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * float64(maxV))
	fmt.Printf("distance from center: %v, distance from focal point: %v, nU: %v, nV: %v\n", cameraLoc.Len(), distance, nU, nV)
	vertexIndicies := make([][]int32, nU+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	fmt.Printf("startUIndex: %v, endUIndex: %v, startVIndex: %v, endVIndex: %v\n", startUIndex, endUIndex, startVIndex, endUIndex)
	for uIndex := startUIndex; uIndex <= endUIndex; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		for vIndex := startVIndex; vIndex <= endVIndex; vIndex++ {
			vertex := uv2xyz(index2percent(float64(uIndex), nU), index2percent(float64(vIndex), nV), t, radius)
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
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2percent(float64(uIndex-startUIndex), endUIndex-startUIndex)))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2percent(float64(vIndex-startVIndex), endVIndex-startVIndex)))
		}
	}
	envmapArray := []float32{}
	blendArray := []float32{}
	metalBlendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			blendValue := float32(blendTexture(index2percent(float64(uIndex), nU), index2percent(float64(vIndex), nV), t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			metalBlendValue := float32(metalBlendTexture(index2percent(float64(uIndex), nU), index2percent(float64(vIndex), nV), t))
			metalBlendArray = append(metalBlendArray, metalBlendValue, metalBlendValue, metalBlendValue)

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
	for vIndex := 0; vIndex < 3000; vIndex++ {
		for uIndex := 0; uIndex < 3000; uIndex++ {
			u := float64(uIndex) / float64(3000)
			v := float64(vIndex) / float64(3000)/2
			envmapValue := 0*float32(pow(sin(v/2),20)) + .5*float32((1-pow(1-pow(perturbation(u, v, t), 10), pow(1-v*2, 5)*10))) + .5*float32((1-pow(1-pow(perturbation(1-u, v, t), 10), pow(1-v*2, 5)*10)))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
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
	plyHeaderPath := fmt.Sprintf("data/%v.header.ply", frameNumber)
	envPath := fmt.Sprintf("data/%v.rgbe", frameNumber)
	blendPath := fmt.Sprintf("data/%v.blend.rgbe", frameNumber)
	metalBlendPath := fmt.Sprintf("data/%v.metal.blend.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, 3000, 3000, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	metalBlend, _ := os.Create(metalBlendPath)
	rgbe.Encode(metalBlend, endUIndex-startUIndex, endVIndex-startVIndex, metalBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera geom.Vec
		LookAt geom.Vec
		Distance float64
		Angle float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".000001"/>
        <float name="fov" value="30"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="independent">
            <integer name="sample_count" value="256"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="2500"/>
            <integer name="height" value="2500"/>
            <rfilter type="gaussian"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="0, 1, 0" angle="{{ .Angle }}"/>
        </transform>
    </emitter>
</scene>
`)
	sensorTemplate.Execute(sensorFile,sensor{cameraLoc, focusPoint, distance, 0})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 16, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 100, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := 1 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
