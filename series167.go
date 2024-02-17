package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"os"
	"text/template"

	"github.com/Opioid/rgbe"
	"github.com/hunterloftis/pbr/pkg/geom"
	// "github.com/hunterloftis/pbr/pkg/surface"
	// "github.com/hunterloftis/pbr/pkg/material"
)

type MeshType struct {
	NumVertices int
	NumFaces    int
}

var sin = math.Sin
var cos = math.Cos
var tan = math.Tan
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var min = math.Min
var max = math.Max
var t = 0.0
var frameNumber = 0

var biggity = 1.0

var maxLen = 1.0

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[i-1])
}

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

func strength(x float64) float64 {
	return sin(x)*2 + 2
}

func strength2(x float64) float64 {
	return cos(x)/2 + .5
}

func subtexture2(x, y, z, t float64) float64 {
	return sin(13*x - 31*y - 23*z + 5*strength(4*t+.1)*subtexture3(x, y, z, t))
}

func subtexture3(x, y, z, t float64) float64 {
	return sin(17*x + 23*y + 19*z)
}

func roughnessTexture(x, y, z, t float64) float64 {
	return spow(sin(5*x+7*y-3*z+4*strength(2+5*t+.2)*subtexture1(x, y, z, t)), .1)*.4 + .41
}

func subtexture1(x, y, z, t float64) float64 {
	subtexture1 := spow(sin(5*x-7*y+3*strength(3+7*t+.3)*subtexture2(x, y, z, t)), .5)
	//fmt.Printf("subtexture1: %v\n", subtexture1)
	return subtexture1
}

func radius(u, v, t float64) float64 {
	return 1.0 + .1*strength2(7*t)*pow(spow(shapeTexture(u, v, t), pow(10, sin(2*t)))/2+.5, pow(10, sin(3*t)))*sin(v)
}

func uvTexture(u, v, t float64, texture func(x, y, z, t float64) float64, shape func(u, v, t float64) geom.Vec) float64 {
	loc := shape(u, v, t)
	return texture(loc.X, loc.Y, loc.Z, t)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2 - 1
}

func pushout(x, n float64) float64 {
	return spow(x*2-1, n)/2 + .5
}

type SLR2 struct {
	AspectRatio float64
	Lens        float64
	FStop       float64
	Focus       float64
	FOV         float64

	trans    *geom.Mtx
	position geom.Vec
	target   geom.Vec
}

var zAxis = geom.Dir{0, 0, 1}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
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
	factor := tan(s.FOV * 1.1 / 360 * pi)
	if projectedPoint.X < projectedPoint.Z*factor*s.AspectRatio || projectedPoint.X > -projectedPoint.Z*factor*s.AspectRatio {
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
	thickness := shape(blendTexture(u, v, t)*2-1, sin(19*t), 0)*.2 + .1
	r := 1.0 + (spow(cos(v/2-.7*sin(v)), pow(10, -cos(17*t)))/2+.5)*thickness
	w := 1.0 - sin(5*t)*spow(sin(v/2), pow(3, sin(3*t)+1))*.9
	w2 := 1.0 + pow(sin(v), 4)*(sin(13*t)*.75+.75)
	loc := geom.Vec{
		sin(u+(sin(7*t)*.15+.15)*sin(2*u)) * sin(v/2) * w * w2 * r,
		cos(u-(sin(7*t)*.15+.15)*cos(2*u)) * sin(v/2) * w * w2 * r,
		-cos(v-(.3+sin(2*t)*.2)*sin(2*v)) * r,
	}
	return loc
}

func blendTexture(u, v, t float64) float64 {
	thickness := .1
	r := 1.0 + (spow(cos(v/2-.7*sin(v)), pow(10, -cos(17*t)))/2+.5)*thickness
	w := 1.0 - sin(5*t)*spow(sin(v/2), pow(3, sin(3*t)+1))*.9
	w2 := 1.0 + pow(sin(v), 4)*(sin(13*t)*.75+.75)
	loc := geom.Vec{
		sin(u+(sin(7*t)*.15+.15)*sin(2*u)) * sin(v/2) * w * w2 * r,
		cos(u-(sin(7*t)*.15+.15)*cos(2*u)) * sin(v/2) * w * w2 * r,
		-cos(v-(.3+sin(2*t)*.2)*sin(2*v)) * r,
	}
	normLoc, _ := loc.Unit()
	norminess := cos(11*t)/2 + .5
	return displacement(normLoc.Scaled(norminess).Plus(loc.Scaled(1-norminess)), t).Len() / maxLen
}

// maybe torusKnot should have a path input and for a regular torus knot it's a circle but for a cable know it's a torusKnot
func torusKnot(t, R, r float64, pInt, qInt int, path func(x float64) geom.Vec) geom.Vec {
	p := float64(pInt)
	q := float64(qInt)
	pathPoint := path(q * t)
	return geom.Vec{(R + r*cos(p*t)) * pathPoint.X, (R + r*cos(p*t)) * pathPoint.Y, r*sin(p*t) + pathPoint.Z}
}

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{sin(float64(xN) * t), sin(float64(yN) * t), cos(float64(zN) * t)}
}

func unitLissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	point, _ := lissajousKnot(t, xN, yN, zN).Unit()
	return geom.Vec(point)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .25, 2, 3, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .15, 3, 2, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	loc, _ := circle(t).Plus(geom.Vec{0, 0, .6 - cos(2*t)*.6}).Unit()
	return loc.Scaled(3 + biggity*3).Minus(geom.Vec{0, 0, 1})
}

func focusPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 0}
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v + delta).Minus(path(v - delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * cos(u)).Plus(sinVec.Scaled(r * sin(u))).Plus(center)
}

func knot(t float64) geom.Vec {
	return unitLissajousKnot(t, 19, 20, 21)
}

func shapeTexture(u, v, t float64) float64 {
	loc := sphere(u, v, t).Scaled(5)
	loc2 := sphere(
		u+.4*strength(2*t+1.1)*sin(loc.X+loc.Y),
		v+.4*strength(3*t+1.2)*sin(loc.Z-loc.X+.4*strength(5*t+1.3)*sin(loc.Z+loc.Y)),
		t).Scaled(5)
	return sin(
		.4 * strength(7*t+1.4) * sin(loc.X+loc2.Y+.4*strength(11*t+1.5)*sin(loc2.Z-loc.Y+.4*strength(13*t+1.6)*sin(loc.Z+loc2.X))))
}

func texture(a, u, v, t float64) float64 {
	return sin(
		u + v + a*strength(.1+2*t)*sin(
			u+a*strength(.2+3*t)*sin(u)) + a*strength(.3+5*t)*sin(
			2*v+a*strength(.4+7*t)*sin(v)) + a*strength(.5+11*t)*sin(
			u+2*v) + a*strength(.6+13*t)*sin(3*u-v) + a*strength(.7+17*t)*sin(5*v-3*u))
}

func yzTexture(a, y, z, t float64) float64 {
	y = y * pi
	z = z * pi
	return sin(
		pow(cos(2*t)*texture(a, y, z, t), 2) +
			pow(cos(3*t)*texture(a, -y, z, t), 2) +
			pow(cos(4*t)*texture(a, y, -z, t), 2) +
			pow(cos(5*t)*texture(a, -y, -z, t), 2) +
			pow(cos(6*t)*texture(a, z, y, t), 2) +
			pow(cos(7*t)*texture(a, -z, y, t), 2) +
			pow(cos(8*t)*texture(a, z, -y, t), 2) +
			pow(cos(9*t)*texture(a, -z, -y, t), 2))
}
func shape(x, a, b float64) float64 {
	return pow(spow(x, pow(2, a))/2+.5, pow(2, b))
}

func displacement(loc geom.Vec, t float64) geom.Vec {
	a := 1.25 + sin(7*t)*.25
	xTexture := shape(yzTexture(a, loc.Y, loc.Z, t), sin(7*t), sin(11*t)) * spow(loc.X, 4)
	yTexture := shape(yzTexture(a, loc.X, loc.Z, t), sin(7*t), sin(11*t)) * spow(loc.Y, 4)
	zTexture := shape(yzTexture(a, loc.Y, loc.X, t), sin(7*t), sin(11*t)) * spow(loc.Z, 4)
	return geom.Vec{xTexture, yTexture, zTexture}
}

func uv2xyz(u, v, t float64) geom.Vec {
	return sphere(u, v, t)
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t)
	right := uv2xyz(index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t)
	up := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t)
	down := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frame int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	frameNumber = frame
	width := int(aspectRatio * float64(height))
	t = float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FOV = 40
	c.AspectRatio = aspectRatio
	distance := cameraLoc.Minus(focusPoint).Len()
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
	maxX := 0.0
	maxY := 0.0
	maxZ := 0.0
	minU := 500
	minV := 500
	maxU := 0
	maxV := 0
	currentMaxLen := 0.0
	for uIndex := 0; uIndex <= 500; uIndex++ {
		for vIndex := 0; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t)
			if !c.invisible(vertex) {
				u := index2radians(float64(uIndex-1), 500)
				v := index2radians(float64(vIndex), 500)
				currentMaxLen = math.Max(currentMaxLen, blendTexture(u, v, t))
			}
		}
	}
	maxLen = currentMaxLen
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 0; uIndex <= 500; uIndex++ {
		for vIndex := 0; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				u := index2radians(float64(uIndex-1), 500)
				v := index2radians(float64(vIndex), 500)
				vertexLeft := uv2xyz(u, v, t)
				vertexBelow := uv2xyz(u, v, t)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, math.Abs(vertex.X))
				maxY = math.Max(maxY, math.Abs(vertex.Y))
				maxZ = math.Max(maxZ, math.Abs(vertex.Z))
				if minU > uIndex {
					minU = uIndex
				}
				if minV > vIndex {
					minV = vIndex
				}
				if maxU < uIndex {
					maxU = uIndex
				}
				if maxV < vIndex {
					maxV = vIndex
				}
				if cameraLoc.Minus(closestPoint).Len() > cameraLoc.Minus(vertex).Len() {
					closestPoint = vertex
				}
			}
		}
	}
	biggity = math.Max(maxX, math.Max(maxY, maxZ))
	cameraLoc = cameraPath(t)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("biggity: %v, minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", biggity, minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
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
	for uIndex := startUIndex; uIndex <= endUIndex; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		for vIndex := startVIndex; vIndex <= endVIndex; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t)
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
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex-startUIndex), endUIndex-startUIndex)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex-startVIndex), endVIndex-startVIndex)/pi/2))
		}
	}
	envmapArray := []float32{}
	roughnessArray := []float32{}
	blendArray := []float32{}
	metalBlendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			roughnessValue := float32(uvTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, roughnessTexture, sphere))
			blendValue := float32(blendTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t) * .85)
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			metalBlendValue := float32(pow(spow(shapeTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t), pow(10, sin(11*t)))/2+.5, pow(2, sin(13*t)*2)))
			metalBlendArray = append(metalBlendArray, metalBlendValue, metalBlendValue, metalBlendValue)
			roughnessArray = append(roughnessArray, roughnessValue, roughnessValue, roughnessValue)

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
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			envmapValue := float32(pow(sin(u/2)*sin(v), 10))
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
	roughnessPath := fmt.Sprintf("data/%v.roughness.rgbe", frameNumber)
	blendPath := fmt.Sprintf("data/%v.blend.rgbe", frameNumber)
	metalBlendPath := fmt.Sprintf("data/%v.metal.blend.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	roughness, _ := os.Create(roughnessPath)
	rgbe.Encode(roughness, endUIndex-startUIndex, endVIndex-startVIndex, roughnessArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	metalBlend, _ := os.Create(metalBlendPath)
	rgbe.Encode(metalBlend, endUIndex-startUIndex, endVIndex-startVIndex, metalBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera    geom.Vec
		LookAt    geom.Vec
		Distance  float64
		FOV       float64
		Aperture  float64
		Height    int
		Width     int
		Samples   int
		RowHeight int
		FogRadius float64
		Angle     float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value=".25"/>
        <float name="aperture_radius" value="{{ .Aperture }}"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="{{ .Samples }}"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="{{ .Width }}"/>
            <integer name="height" value="{{ .Height }}"/>
            <integer name="crop_offset_y" value="$offset"/>
            <integer name="crop_height" value="{{ .RowHeight }}"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="5"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="30"/>
            <rotate value="0, 0, 1" angle="{{ .Angle }}"/>
        </transform>
    </emitter>
    <integrator type="path" />
    <bsdf type="blendbsdf" id="object_bsdf">
        <texture type="bitmap" name="weight">
            <string name="filename" value="mitsuba.blend.rgbe"/>
        </texture>
       <bsdf type="twosided">
            <bsdf type="diffuse">
            </bsdf>
        </bsdf>
         <bsdf type="twosided">
            <bsdf type="conductor">
                <string name="material" value="CuO"/>
            </bsdf>
        </bsdf>
    </bsdf>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
        <ref id="object_bsdf"/>
    </shape>
	<shape type="ply">
        <string name="filename" value="paper.ply"/>
        <transform name="to_world">
            <scale value="1,10,10"/>
            <rotate value="0, 1, 0" angle="90"/>
            <translate x="0" y="0" z="-1.05"/>
        </transform>
	</shape>
</scene>
`)
	angle := 180 - t/pi*180
	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		c.FOV,
		.000000000001,
		height,
		width,
		samples,
		height / numRows,
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		angle,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 100, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	aspectRatio := flag.Float64("aspectratio", 1.0, "Aspect ratio")
	height := flag.Int("height", 720, "Height")
	samples := flag.Int("samples", 25, "Samples")
	numRows := flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
