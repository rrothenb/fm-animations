//go:build ignore

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

var frameNumber = 0
var t2 = 0.0

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
	return pow(2, sin(x)+1)
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

func clay1Color(x, y, z, t float64) geom.Vec {
	r := sin(2*t)*.4 + .5
	rRange := min(.25, min(r, 1-r))
	g := sin(3*t)*.4 + .5
	gRange := min(.25, min(r, 1-r))
	b := sin(5*t)*.4 + .5
	bRange := min(.25, min(r, 1-r))
	a := .1
	return geom.Vec{
		r + rRange*sin(x+a*strength(7*t)*sin(y+a*strength(23*t)*sin(y))),
		g + gRange*sin(x+z+a*strength(11*t)*sin(z+a*strength(19*t)*sin(x))),
		b + bRange*sin(x-y+a*strength(13*t)*sin(z+a*strength(17*t)*sin(x-y))),
	}
}

func subtexture1(x, y, z, t float64) float64 {
	subtexture1 := spow(sin(5*x-7*y+3*strength(3+7*t+.3)*subtexture2(x, y, z, t)), .5)
	//fmt.Printf("subtexture1: %v\n", subtexture1)
	return subtexture1
}

func radius(u, v, t float64) float64 {
	return 1.0 + .1*strength2(13*t)*pow(spow(shapeTexture(u, v, t), pow(10, sin(17*t)))/2+.5, pow(10, sin(19*t)))*sin(v)*(spow(cos(v/2-.7*sin(v)), .1)*spow(sin(23*t), .1)/2+.5)
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

var zAxis = geom.Dir{0, 1, 0}

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
	factor := tan(s.FOV * 1.5 / 360 * pi)
	if projectedPoint.X < projectedPoint.Z*factor*s.AspectRatio || projectedPoint.X > -projectedPoint.Z*factor*s.AspectRatio {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
	if projectedPoint.Z > 0.0 {
		return true
	}
	return false
	if projectedPoint.Z < -s.position.Len() {
		return true
	}
	return false
}

func circle(x float64) geom.Vec {
	return geom.Vec{sin(x), cos(x), 0}
}

func lipTexture(u, t float64) float64 {
	return sin(u + strength(2*t)*sin(u+strength(3*t)*sin(u)) + strength(5*t)*sin(2*u) + strength(7*t)*sin(3*u))
}

func sphere(u, v, t float64) geom.Vec {
	a := .25 - cos(17*t)*.25
	b := .25 - cos(19*t)*.25
	c := .25 - cos(23*t)*.25
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-b*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+b*sin(2*u)),
		cos(v/2.0 - c*sin(v)),
	}
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
	x := .5 - cos(103*t)/2
	z := 1 - x
	y := pow(x*z, .5)
	loc, _ := geom.Vec{x, y, z}.Unit()
	return loc.Scaled(2)
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

func shapeTexture1(u, v, t float64) float64 {
	loc := sphere(u, v, t).Scaled(5)
	loc2 := sphere(
		u+.4*strength(2*t+1.1)*sin(loc.X+loc.Y),
		v+.4*strength(3*t+1.2)*sin(loc.Z-loc.X+.4*strength(5*t+1.3)*sin(loc.Z+loc.Y)),
		t).Scaled(5)
	return sin(
		.4 * strength(7*t+1.4) * sin(loc.X+loc2.Y+.4*strength(11*t+1.5)*sin(loc2.Z-loc.Y+.4*strength(13*t+1.6)*sin(loc.Z+loc2.X))))
}

func shapeTexture2(u, v, t float64) float64 {
	loc := sphere(u, v, t).Scaled(5)
	return sin(
		strength(2*t)*5*sin(loc.X)*sin(loc.Y)*sin(loc.Z) +
			(1-strength(2*t))*4*sin(loc.X-loc.Y+.4*strength(3*t)*sin(loc.Z-loc.X+.4*strength(5*t)*sin(loc.Z-loc.Z))))
}

func shapeTexture(u, v, t float64) float64 {
	a := spow(sin(11*t), .5)/2 + .5
	return a*shapeTexture1(u, v, t) + (1-a)*shapeTexture2(u, v, t)
}

func shape(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

// 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97
func texture(u, v, t float64) float64 {
	t = sin(t2) * .01
	vFreq := floor(14 + sin(101*t)*10)
	uFreq := floor((sin(97*t)/2+.5)*(vFreq-4) + 4)
	a := sin(89*t) * .5
	b := sin(83 * t)
	c := sin(79 * t)
	d := sin(73 * t)
	e := sin(71 * t)
	f := sin(67 * t)
	g := sin(61 * t)
	h := sin(59 * t)
	i := sin(53 * t)
	j := sin(47 * t)
	k := sin(43 * t)
	return pow(shape(sin(vFreq*v+c*sin(v+f*sin(u)+g*sin(v)))*sin((vFreq-uFreq)*v+uFreq*u+d*sin(v+h*sin(u)+i*sin(v))+e*sin(u+j*sin(u)+k*sin(v))), a, b), 2)
}

func and(a, b float64) float64 {
	return a * b
}
func or(a, b float64) float64 {
	return 1 - (1-a)*(1-b)
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	tuv := texture(pow(10, floor(sin(31*t)+2))*u, pow(10, floor(sin(37*t)+2))*v, t)
	tvu := texture(pow(10, floor(sin(29*t)+2))*v, pow(10, floor(sin(41*t)+2))*u, t)
	blend := sin(31*t2)*.1 + .5
	microTexture := blend*and(tuv, tvu) + (1-blend)*or(tuv, tvu)
	return sphere(u, v, t).Scaled(1 - .25*texture(u, v, t) - .0025*microTexture*(1-metalBlendTexture(u, v, t)))
}

func metalBlendTexture(u, v, t float64) float64 {
	return texture(u, v, t)

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

func hsb2rgb(hue, sat, bri float64) (r, g, b float64) {
	u := bri
	if sat == 0 {
		r, g, b = u, u, u
	} else {
		h := (hue - math.Floor(hue)) * 6
		f := h - math.Floor(h)
		p := bri * (1 - sat)
		q := bri * (1 - sat*f)
		t := bri * (1 - sat*(1-f))
		switch int(h) {
		case 0:
			r, g, b = u, t, p
		case 1:
			r, g, b = q, u, p
		case 2:
			r, g, b = p, u, t
		case 3:
			r, g, b = p, q, u
		case 4:
			r, g, b = t, p, u
		case 5:
			r, g, b = u, p, q
		}
	}
	return
}

func renderSurfaces(pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	width := int(aspectRatio * float64(height))
	t := float64(60) * dt
	t2 = float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	fov := 75.0
	c.FOV = fov
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
	minZ := 1.0
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 0; uIndex <= 500; uIndex++ {
		for vIndex := 0; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t, radius)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t, radius)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, math.Abs(vertex.X))
				maxY = math.Max(maxY, math.Abs(vertex.Y))
				maxZ = math.Max(maxZ, math.Abs(vertex.Z))
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
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	fov = 75.0 + 50*(maxDistance-1) + sin(2*t2)*10
	c.FOV = fov
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
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
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex-startUIndex), endUIndex-startUIndex)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex-startVIndex), endVIndex-startVIndex)/pi/2))
		}
	}
	envmapArray := []float32{}
	roughnessArray := []float32{}
	blendArray := []float32{}
	metalBlendArray := []float32{}
	clay1ColorArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * pi
			roughnessValue := float32(uvTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, roughnessTexture, sphere))
			// blendValue := float32(uvTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, blendTexture, sphere))
			blendValue := float32(spow((sin(2*u+1000*v+strength(61*t+1.7)*sin(5*u+873*v))+sin(3*u+901*v+strength(67*t+1.8)*sin(u-1101*v)))/2, .1))/2 + .5
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			metalBlendValue := float32(metalBlendTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
			metalBlendArray = append(metalBlendArray, metalBlendValue, metalBlendValue, metalBlendValue)
			roughnessArray = append(roughnessArray, roughnessValue, roughnessValue, roughnessValue)
			loc := sphere(u, v, t).Scaled(pi)
			clay1ColorValue := clay1Color(loc.X, loc.Y, loc.Z, t)
			clay1ColorArray = append(clay1ColorArray, float32(clay1ColorValue.X), float32(clay1ColorValue.Y), float32(clay1ColorValue.Z))

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
			envmapValue := float32(pow(sin(u/2)*sin(v), pow(2, sin(19*t)+2)))
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
	clay1ColorPath := fmt.Sprintf("data/%v.clay.1.color.rgbe", frameNumber)
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
	clay1Color, _ := os.Create(clay1ColorPath)
	rgbe.Encode(clay1Color, endUIndex-startUIndex, endVIndex-startVIndex, clay1ColorArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera    geom.Vec
		LookAt    geom.Vec
		Distance  float64
		FOV       float64
		EnvX      float64
		EnvY      float64
		EnvZ      float64
		Roughness float64
		Red       float64
		Green     float64
		Blue      float64
		Red2      float64
		Green2    float64
		Blue2     float64
		ExtIOR    float64
		Height    int
		Width     int
		Samples   int
		RowHeight int
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".0000000000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 1, 0"/>
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
    <integrator type="path" />
    <bsdf type="blendbsdf" id="object_bsdf">
        <texture type="bitmap" name="weight">
            <string name="filename" value="mitsuba.metal.blend.rgbe"/>
        </texture>
            <bsdf type="roughdielectric">
				<float name="alpha" value="{{ .Roughness }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
            </bsdf>
         <bsdf type="twosided">
            <bsdf type="conductor">
				<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
				<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
            </bsdf>
		 </bsdf>
    </bsdf>
<shape type="shapegroup" id="my_shape_group">
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
        <ref id="object_bsdf"/>
    </shape>
</shape>
<shape type="instance">
    <ref id="my_shape_group"/>
</shape>
<shape type="instance">
    <ref id="my_shape_group"/>
        <transform name="to_world">
            <scale value="3"/>
            <translate x="0" y="0" z="0"/>
        </transform>
</shape>
</scene>
`)

	red, green, blue := hsb2rgb(sin(3*t2)*.1+.5, .95, .75+cos(7*t2)*.2)
	red2, green2, blue2 := hsb2rgb(sin(5*t2)*.1+.5, .95, .75+cos(11*t2)*.2)

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		fov,
		sin(2*t2) * 175,
		sin(3*t2) * 175,
		sin(5*t2) * 175,
		pow(10, sin(29*t2)-1),
		red,
		green,
		blue,
		red2,
		green2,
		blue2,
		1.5 + sin(7*t2)*.01,
		height,
		width,
		samples,
		height / numRows,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 120, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	aspectRatio := flag.Float64("aspectratio", 1.0, "Aspect ratio")
	height := flag.Int("height", 720, "Height")
	samples := flag.Int("samples", 25, "Samples")
	numRows := flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	frameNumber = *frame
	renderSurfaces(*pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
