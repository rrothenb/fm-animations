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
var floor = math.Floor
var t = 0.0
var frameNumber = 0

var biggity = 1.0

var maxLen = 1.0
var center = geom.Vec{0, 0, 0}

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[i-1])
}

var globalT = 0.0
var nV = 0

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

func pushout(x, duty, degree float64) float64 {
	return spow(pow(x, duty)*2-1, degree)/2 + .5
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
	return torusKnot(t, 1, .45+sin(19*globalT)*.25, 2, 3, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .35+sin(17*globalT)*.25, 3, 2, outerKnot)
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

func cube(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+.6*sin(v)) * cos(u-.6*sin(2*u)),
		sin(v/2.0+.6*sin(v)) * sin(u+.6*sin(2*u)),
		cos(v/2.0 - .6*sin(v)),
	}
}

func cameraPath(t float64) geom.Vec {
	loc, _ := geom.Vec{1, 1, .5}.Unit()
	return loc.Scaled(6)
}

func focusPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 0}
}

func strength(x float64) float64 {
	return sin(x)*1.25 + 1.25
}

func textureOriginal(u, v, t float64) float64 {
	v = v * 5
	return sin(
		3*u + 5*v +
			strength(.1+2*t)*sin(2*u+strength(.2+3*t)*sin(3*u)) +
			strength(.3+5*t)*sin(7*v+strength(.4+7*t)*sin(5*v)) +
			strength(.7+11*t)*sin(5*u+7*v) +
			strength(.5+13*t)*sin(17*u)*sin(19*v))
}

func subtexture3(u, v, t float64) float64 {
	return sin(2*u + 3*v)
}

func subtexture2(u, v, t float64) float64 {
	return sin(7*u+strength(17*t)*subtexture3(u, v, t)) + sin(5*v+strength(19*t)*subtexture3(u, v, t))
}

func texture(a, u, v, t float64) float64 {
	return sin(
		u + v + a*strength(.1+2*t)*sin(
			u+a*strength(.2+3*t)*sin(u)) + a*strength(.3+5*t)*sin(
			2*v+a*strength(.4+7*t)*sin(v)) + a*strength(.5+11*t)*sin(
			u+2*v) + a*strength(.6+13*t)*sin(3*u-v) + a*strength(.7+17*t)*sin(5*v-3*u))
}

func blendTexture(u, v, t float64) float64 {
	loc := sphereish(u, v, 0, 0, 0)
	return displacement(loc, t).Len() * (spow(sin(v+.7*sin(2*v)), .5)/2 + .5)
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

func sphereish(u, v, a, b, c float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-b*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+b*sin(2*u)),
		cos(v/2.0 - c*sin(v)),
	}
}

func thingy(a, u, v, t float64) float64 {
	return sin(pow(texture(a, u, v, t), 2) + pow(texture(a, v, u, t), 2))
}

func bowl1(thickness, insideTexture, outsideTexture, u, v, t float64) geom.Vec {
	width := 1.0 + .1*strength(3*t+.3)*pow(sin(v/2), 10)
	height := sin(t)*.15 + .35 + .1*strength(2*t+.2)*pow(sin(v/2), 10)
	space := (cos(v/2-.7*sin(v))/2+.5)*(thickness+outsideTexture) + (.5-cos(v/2-.7*sin(v))/2)*insideTexture
	return geom.Vec{
		width * sin(u) * sin(v/2) * (1 + 1/height*space),
		width * cos(u) * sin(v/2) * (1 + 1/height*space),
		-height * cos(v-(sin(7*t)*.4+.5)*sin(2*v)) * (1 + 1/height*space),
	}
}

func bowl2(thickness, u, v, t float64) geom.Vec {
	// r := 1.0 + cos(v/2-.7*sin(v))*thickness/2
	r := 1.0 + (spow(cos(v/2-.7*sin(v)), .1)/2+.5)*thickness
	w := 1.0 - sin(7*t)*spow(sin(v/2), pow(3, sin(11*t)+1))*.9
	w2 := 1.0 + pow(sin(v), 4)*(sin(13*t)*.75+.75)
	return geom.Vec{
		sin(u) * sin(v/2) * w * w2 * r,
		cos(u) * sin(v/2) * w * w2 * r,
		-cos(v) * r,
	}
}

func bowl3(thickness, u, v, t float64) geom.Vec {
	percent := .5 + sin(59*t)*.25
	v2 := 2 * pi * (sin(v / 2)) * percent
	thickness = thickness * percent
	r := 1 + spow(sin(v+.7*sin(2*v)), .25)*thickness/2
	v = v2 / 2
	return geom.Vec{
		sin(v) * cos(u),
		sin(v) * sin(u),
		-cos(v),
	}.Scaled(r).Scaled(1 / percent)
}

func bowl4(u, v, t float64) geom.Vec {
	thickness := .1
	r := 1.0 + cos(v/2-.7*sin(v))*thickness/2
	w := 1.0 - sin(5*t)*spow(sin(v/2), pow(3, sin(3*t)+1))*.1
	return geom.Vec{
		sin(u+(sin(7*t)*.15+.15)*sin(2*u)) * sin(v/2) * w * r,
		cos(u-(sin(7*t)*.15+.15)*cos(2*u)) * sin(v/2) * w * r,
		-cos(v-(.3+sin(2*t)*.2)*sin(2*v)) * r,
	}
}

func sinG(x, a, b float64) float64 {
	return sin(x + a*sin(2*x) + b/2*sin(4*x))
}

func cosG(x, a, b float64) float64 {
	return cos(x - a*sin(2*x) + b/2*sin(4*x))
}

func bowl5(u, v, t float64) geom.Vec {
	v = v / 2
	a := .4 - .4*cos(6*t)
	b := .4 - .4*cos(3*t)
	c := .4 - .4*cos(4*t)
	d := .4 - .4*cos(5*t)
	e := .4 - .4*cos(2*t)
	f := .4 - .4*cos(7*t)
	return geom.Vec{
		sinG(v, a, b) * cosG(u, c, d),
		sinG(v, a, b) * sinG(u, c, d),
		cosG(v, e, f),
	}
}

func theBowl(u, v, t float64) geom.Vec {
	if frameNumber%5 == 1 {
		return bowl3(.1, u, v, t)
	} else if frameNumber%5 == 2 {
		return bowl4(u, v, t)
	} else if frameNumber%5 == 3 {
		return bowl5(u, v, t)
	} else if frameNumber%5 == 4 {
		return bowl2(.1, u, v, t)
	} else {
		return bowl1(.1, .5-cos(2*t)/2, .5-cos(3*t)/2, u, v, t)
	}
}

func uv2xyz(u, v, t float64) geom.Vec {
	loc := theBowl(u, v, t)
	texture := displacement(sphereish(u, v, 0, 0, 0), t).Len() * .1
	unitLoc, _ := loc.Minus(center).Unit()
	return loc.Plus(unitLoc.Scaled(texture * (spow(sin(v+.7*sin(2*v)), .5)/2 + .5)))
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
	globalT = t
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
	nV = nU
	numTriangles := 0
	totalWidth := 0.0
	totalHeight := 0.0
	minDistance := 100.0
	maxDistance := 0.0
	maxX := -100.0
	maxY := -100.0
	maxZ := -100.0
	minX := 100.0
	minY := 100.0
	minZ := 100.0
	minU := 500
	minV := 500
	maxU := 0
	maxV := 0
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 0; uIndex <= 500; uIndex++ {
		for vIndex := 0; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, vertex.X)
				maxY = math.Max(maxY, vertex.Y)
				maxZ = math.Max(maxZ, vertex.Z)
				minX = math.Min(minX, vertex.X)
				minY = math.Min(minY, vertex.Y)
				minZ = math.Min(minZ, vertex.Z)
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
	c.FOV = 40 + (maxX - minX) + (maxY - minY) + (maxZ - minZ)
	midX := (minX + maxX) / 2
	midY := (minY + maxY) / 2
	midZ := (minZ + maxZ) / 2
	center = geom.Vec{midX, midY, midZ}
	focusPoint = center
	//focusPoint = geom.Vec{(minX+maxX)/2, (minY+maxY)/2, (minZ+maxZ)/2}
	//cameraLoc = cameraLoc.Scaled(math.Max(maxX, math.Max(maxY, maxZ))*8)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v, minZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ, minZ)
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
	blendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			blendValue := float32(blendTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
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
			envmapValue := float32(pow(sin(u/2), 4) * pow(sin(v), 4))
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
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
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
		Weight1   int
		Weight2   int
		Weight3   int
		Weight4   int
		EnvX      float64
		EnvY      float64
		EnvZ      float64
		Rough1    float64
		Rough2    float64
		MinX      float64
		MaxX      float64
		MinY      float64
		MaxY      float64
		MinZ      float64
		MaxZ      float64
		MidX      float64
		MidY      float64
		MidZ      float64
		Red       float64
		Green     float64
		Blue      float64
		Red2      float64
		Green2    float64
		Blue2     float64
		IntIOR    float64
		Scale     float64
		G         float64
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
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="{{ .EnvX }}"/>
            <rotate value="0, 1, 0" angle="{{ .EnvY }}"/>
            <rotate value="0, 0, 1" angle="{{ .EnvZ }}"/>
        </transform>
    </emitter>
        <integrator type="volpathmis">
            <integer name="max_depth" value="16"/>
        </integrator>
    <medium id="medium1" type="homogeneous">
        <float name="scale" value="{{ .Scale }}"/>
        <rgb name="albedo" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
        <rgb name="sigma_t" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <ref id="medium1" name="interior"/>
    	<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.blend.rgbe"/>
			</texture>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight1 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight2 }}"/>
				   <bsdf type="twosided">
						<bsdf type="diffuse">
							<rgb name="reflectance" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						</bsdf>
					</bsdf>
				      <bsdf type="twosided">
						<bsdf type="roughplastic">
                			<float name="alpha" value="{{ .Rough1 }}"/>
            				<float name="int_ior" value="{{ .IntIOR }}"/>
							<rgb name="diffuse_reflectance" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						</bsdf>
				     </bsdf>
				</bsdf>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight2 }}"/>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough1 }}"/>
						<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						<rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						</bsdf>
					</bsdf>
						<bsdf type="roughdielectric">
						<float name="alpha" value="{{ .Rough1 }}"/>
            				<float name="int_ior" value="{{ .IntIOR }}"/>
						</bsdf>
				</bsdf>
			</bsdf>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight3 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight4 }}"/>
				   <bsdf type="twosided">
						<bsdf type="diffuse">
							<rgb name="reflectance" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						</bsdf>
					</bsdf>
				      <bsdf type="twosided">
						<bsdf type="roughplastic">
                			<float name="alpha" value="{{ .Rough2 }}"/>
            				<float name="int_ior" value="{{ .IntIOR }}"/>
							<rgb name="diffuse_reflectance" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						</bsdf>
				     </bsdf>
				</bsdf>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight4 }}"/>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough2 }}"/>
						<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						</bsdf>
					</bsdf>
						<bsdf type="roughdielectric">
						<float name="alpha" value="{{ .Rough2 }}"/>
            				<float name="int_ior" value="{{ .IntIOR }}"/>
						</bsdf>
				</bsdf>
			</bsdf>
		</bsdf>
    </shape>
	<shape type="rectangle">
        <transform name="to_world">
            <scale value="10"/>
            <translate x="0" y="0" z="{{ .MinZ }}"/>
        </transform>
				<bsdf type="roughplastic">
				<float name="alpha" value=".01"/>
				</bsdf>
	</shape>
</scene>
`)
	fmt.Printf("frame: %v, weights: %v, %v, %v, %v\n", frameNumber, frameNumber%2, (frameNumber/2)%2, (frameNumber/4)%2, (frameNumber/8)%2)

	sensorTemplate.Execute(
		sensorFile,
		sensor{
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
			0,
			frameNumber % 2,
			(frameNumber / 2) % 2,
			(frameNumber / 4) % 2,
			(frameNumber / 8) % 2,
			-45,
			0,
			0,
			pow(10, sin(5*t)-2),
			pow(10, cos(7*t)-2),
			minX,
			maxX,
			minY,
			maxY,
			minZ,
			maxZ,
			midX,
			midY,
			midZ,
			cos(2*t)/2 + .5,
			cos(3*t)/2 + .5,
			cos(5*t)/2 + .5,
			.5 - cos(13*t)/2,
			.5 - cos(11*t)/2,
			.5 - cos(7*t)/2,
			1.5 + sin(17*t)*.4,
			pow(10, sin(19*t)+2),
			cos(23*t) * .9,
		})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 500, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
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
