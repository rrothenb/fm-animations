//go:build ignore
// +build ignore

package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"image/jpeg"
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

func pushout(x, duty, degree float64) float64 {
	return spow(pow(x, duty)*2-1, degree)/2 + .5
}

func strength(n int, x float64) float64 {
	return pow(2, sin(pow(float64(n), .5)*(x+float64(n)/3)))
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

var zAxis = geom.Dir{.707, .707, 0}

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

func lipTexture(u, t float64) float64 {
	return sin(u + 2*strength(5, t)*sin(u+2*strength(7, t)*sin(u)) + 2*strength(11, t)*sin(2*u) + 2*strength(13, t)*sin(3*u))
}

func bowl(thickness, insideTexture, outsideTexture, u, v, t float64) geom.Vec {
	width := 1.0 + .1*strength(3, t)*pow(sin(v/2), 10)*pow(spow(lipTexture(u, t), pow(3, sin(2*t)))/2+.5, pow(3, sin(3*t)))
	height := sin(t)*.15 + .35 + .1*strength(2, t)*pow(sin(v/2), 10)*pow(spow(lipTexture(u, t), pow(3, sin(2*t)))/2+.5, pow(3, sin(3*t)))
	space := (cos(v/2-.7*sin(v))/2+.5)*(thickness+outsideTexture) + (.5-cos(v/2-.7*sin(v))/2)*insideTexture
	return geom.Vec{
		width * sin(u) * sin(v/2) * (1 + 1/height*space),
		width * cos(u) * sin(v/2) * (1 + 1/height*space),
		-height * cos(v-(sin(7*t)*.4+.5)*sin(2*v)) * (1 + 1/height*space),
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
	return torusKnot(t, 1, .8, 5, 7, circle)
}

func middleKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .4, 7, 5, outerKnot)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .2, 5, 7, middleKnot)
}

func cameraPath(t float64) geom.Vec {
	return geom.Vec{sin(prime(12)*t) * .75, sin(prime(13)*t) * .75, 1.75 + sin(prime(14)*t)*.25}
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
	a := sin(prime(11)*t)/2 + .2
	return cosVec.Scaled(r * cos(u-a*sin(2*u))).Plus(sinVec.Scaled(r * sin(u+a*sin(2*u)))).Plus(center)
}

func knot(t float64) geom.Vec {
	return unitLissajousKnot(t, 19, 20, 21)
}

func sphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v / 2.0),
	}
}

func shapeTexture(f, a, t float64, loc geom.Vec) float64 {
	loc = loc.Scaled(f * 2 * pi)
	loc.X = abs(loc.X)
	loc.Y = abs(loc.Y)
	loc.Z = abs(loc.Z)
	return sin(
		a*strength(7, t)*sin(a*strength(23, t)*loc.Z) +
			a*strength(7, t)*sin(a*strength(23, t)*loc.Y) +
			a*strength(7, t)*sin(a*strength(23, t)*loc.X) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y+a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z+a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y-a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z-a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)))
}

func cube(u, v, t float64) geom.Vec {
	a := sin(t)*.2 + .25
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-a*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+a*sin(2*u)),
		cos(v/2.0 - a*sin(v)),
	}
}

func shape(u, v, t float64) geom.Vec {
	return cube(u, v, t)
}

func strength2(x float64) float64 {
	return pow(2, sin(x)*2)
}

func texture2(u, v, t float64) float64 {
	a := .75
	minT := pi * .95
	maxT := pi * 1.05
	t = minT + t/2/pi*(maxT-minT)
	return sin(
		10*u + a*strength2(.1+2*t)*sin(
			2*u+a*strength2(.2+3*t)*sin(3*u)) + a*strength2(.3+5*t)*sin(
			2*v+a*strength2(.4+7*t)*sin(-v)) + a*strength2(.5+11*t)*sin(
			5*u-v) + a*strength2(.6+13*t)*sin(7*u-v) + a*strength2(.7+17*t)*sin(v-3*u))
}

func fabricPath(x float64) geom.Vec {
	loc := geom.Vec{sin(41 * x), sin(43 * x), sin((41*43-2)*x) * pow(3, sin(prime(8)*t)-1)}
	displacement := geom.Vec{0, 0, pow(10, sin(prime(9)*t)-1) * pow(spow(texture2((loc.X-loc.Y)*pi, (loc.X+loc.Y)*pi, t), pow(2, sin(prime(9)*t)))/2+.5, pow(2, cos(prime(10)*t)))}
	return loc.Plus(displacement.Scaled(0))
}

func uv2xyz(u, v, t float64) geom.Vec {
	minWidth := cos(prime(7)*t)*.025 + .025
	maxWidth := (.05-minWidth)*(sin(prime(6)*t)/2+.5) + minWidth
	width := (sin(math.Trunc(pow(10, sin(prime(5)*t)+3))*v)/2+.5)*(maxWidth-minWidth) + minWidth
	return pathWrapper(u, v, width, fabricPath).By(geom.Vec{1, 1, pow(3, cos(prime(4)*t)-1)})
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
	c.FOV = 35
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
	minX := 0.0
	minY := 0.0
	minU := 1000
	minV := 1000
	maxU := 0
	maxV := 0
	var uHistogram [101]int
	var vHistogram [101]int
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 0; uIndex <= 1000; uIndex++ {
		for vIndex := 0; vIndex <= 1000; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 1000), index2radians(float64(vIndex), 1000), t)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				uHistogram[uIndex/50]++
				vHistogram[vIndex/50]++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 1000), index2radians(float64(vIndex), 1000), t)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 1000), index2radians(float64(vIndex-1), 1000), t)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, vertex.X)
				maxY = math.Max(maxY, vertex.Y)
				minX = math.Min(minX, vertex.X)
				minY = math.Min(minY, vertex.Y)
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
	fmt.Println("minX, maxX, minY, maxY", minX, maxX, minY, maxY)
	fmt.Println(uHistogram)
	fmt.Println(vHistogram)
	//cameraLoc = cameraLoc.Scaled(math.Max(maxX, math.Max(maxY, maxZ))*8)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	//distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 1000)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 1000)
	startUIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * float64(minU))
	endUIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * float64(maxU))
	startVIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * float64(minV))
	endVIndex := int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * float64(maxV))
	fmt.Printf(
		"distance from center: %v, distance from focal point: %v, nU: %v, nV: %v, startUIndex: %v, endUIndex: %v, startVIndex: %v, endVIndex: %v\nU*V: %v, percent: %v\n",
		cameraLoc.Len(), distance, nU, nV, startUIndex, endUIndex, startVIndex, endVIndex, (endUIndex-startUIndex)*(endVIndex-startVIndex), (endVIndex-startVIndex)*100/nV)
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
	textureArray := []float32{}
	numFaces := 0
	// Open the JPEG file
	file, err := os.Open("self-portrait.jpg")
	if err != nil {
		fmt.Println(err)
		return
	}
	defer file.Close()

	// Decode the JPEG image
	img1, err := jpeg.Decode(file)
	if err != nil {
		fmt.Println(err)
		return
	}
	rMin := uint32(65535)
	rMax := uint32(0)
	gMin := uint32(65535)
	gMax := uint32(0)
	bMin := uint32(65535)
	bMax := uint32(0)
	aMin := uint32(65535)
	aMax := uint32(0)
	for x := 0; x < img1.Bounds().Dx(); x++ {
		for y := 0; y < img1.Bounds().Dy(); y++ {
			r, g, b, a := img1.At(x, y).RGBA()
			if rMin > r {
				rMin = r
			}
			if rMax < r {
				rMax = r
			}
			if gMin > g {
				gMin = g
			}
			if gMax < g {
				gMax = g
			}
			if aMin > a {
				aMin = a
			}
			if aMax < a {
				aMax = a
			}
			if bMin > b {
				bMin = b
			}
			if bMax < b {
				bMax = b
			}
		}
	}
	fmt.Println(rMin, rMax, gMin, gMax, bMin, bMax, aMin, aMax)
	pixelCenter := img1.At(img1.Bounds().Dx()/2, img1.Bounds().Dy()/2)
	pixelUR := img1.At(img1.Bounds().Dx()-1, img1.Bounds().Dy()-1)
	pixelLR := img1.At(0, img1.Bounds().Dy()-1)
	pixelUL := img1.At(img1.Bounds().Dx()-1, 0)
	pixelLL := img1.At(0, 0)
	r, g, b, a := pixelCenter.RGBA()
	fmt.Printf("x: %v, y: %v, r: %v, g: %v, b: %v, a: %v, pixelCenter: %v, pixelUR: %v, pixelLR: %v, pixelUL: %v, pixelLL: %v\n",
		img1.Bounds().Dx(),
		img1.Bounds().Dy(),
		r, g, b, a,
		pixelCenter,
		pixelUR,
		pixelLR,
		pixelUL,
		pixelLL,
	)
	imageLocXMin := float64(img1.Bounds().Dx())
	imageLocXMax := 0.0
	imageLocYMin := float64(img1.Bounds().Dy())
	imageLocYMax := 0.0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			loc := uv2xyz(u, v, t)
			// blendValue := float32((.5-cos(v/2-.7*sin(v))/2)*(.01*pow(spow(shapeTexture(3, 2, t, loc), pow(strength(5, t), 4))/2+.5, pow(strength(7, t), 4))))
			//textureValue := pow(spow(shapeTexture(1, .75-cos(41*t)*.5, t, loc), .1)/2+.5, pow(10, sin(29*t)))
			factor := tan(c.FOV * .9 / 360 * pi)
			cameraSpaceTransform := c.trans.Inverse()
			imageLoc := cameraSpaceTransform.MultPoint(loc)
			imageLoc = imageLoc.
				Scaled(2500 / imageLoc.Z / factor).
				Plus(geom.Vec{float64(img1.Bounds().Dx()/2) + 150, float64(img1.Bounds().Dy()/2) - 250, 0})
			/*
				imageLoc := geom.Vec{.707*loc.X - .707*loc.Y + .75, -.707*loc.X - .707*loc.Y - 1.25, 0}
				imageLoc = imageLoc.
					Scaled(.75 / 2.0 / pi).
					Minus(geom.Vec{minX, minY, 0}).
					By(geom.Vec{1 / (maxX - minX), 1 / (maxY - minY), 0}).
					By(geom.Vec{float64(img1.Bounds().Dx()), float64(img1.Bounds().Dy()), 0})
			*/
			if !c.invisible(loc) {
				imageLocXMin = min(imageLocXMin, imageLoc.X)
				imageLocXMax = max(imageLocXMax, imageLoc.X)
				imageLocYMin = min(imageLocYMin, imageLoc.Y)
				imageLocYMax = max(imageLocYMax, imageLoc.Y)
			}
			lumosity := 1 -
				(.2126*float32(r)/65536 +
					.7152*float32(g)/65536 +
					.0722*float32(b)/65536)
			rRaw, gRaw, bRaw, _ := img1.At(int(imageLoc.X), int(imageLoc.Y)).RGBA()
			r := float32(float64(rRaw) / 65536)
			g := float32(float64(gRaw) / 65536)
			b := float32(float64(bRaw) / 65536)
			if true {
				textureArray = append(textureArray, r, g, b)
			} else if frameNumber%4 == 1 {
				textureArray = append(textureArray, 1-r, 1-g, 1-b)
			} else if frameNumber%4 == 2 {
				textureArray = append(textureArray, lumosity, lumosity, lumosity)
			} else {
				textureArray = append(textureArray, 1-lumosity, 1-lumosity, 1-lumosity)
			}
			blendArray = append(blendArray, lumosity, lumosity, lumosity)
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
	fmt.Println("imageLocXMin, imageLocXMax, imageLocYMin, imageLocYMax", imageLocXMin, imageLocXMax, imageLocYMin, imageLocYMax)
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			power := 15 * pow(3, cos(5*t))
			envmapValue := .8*float32(pow(sin(u/2), power)*pow(sin(v), power)) + .2
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
	texturePath := fmt.Sprintf("data/%v.texture.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	texture, _ := os.Create(texturePath)
	rgbe.Encode(texture, endUIndex-startUIndex, endVIndex-startVIndex, textureArray)
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
		Scale     float64
		G         float64
		Red       float64
		Green     float64
		Blue      float64
		Red2      float64
		Green2    float64
		Blue2     float64
		EnvX      float64
		EnvY      float64
		EnvZ      float64
		Switch1   int
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value=".25"/>
        <float name="aperture_radius" value=".00000000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="1, 1, 0"/>
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
        <integrator type="path">
        </integrator>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <bsdf type="blendbsdf">
            <float name="weight" value="{{ .Switch1 }}"/>
        	<bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
				<bsdf type="dielectric">
				</bsdf>
				 <bsdf type="twosided">
					<bsdf type="conductor">
						<texture type="bitmap" name="eta">
							<string name="filename" value="mitsuba.blend.rgbe"/>
						</texture>
						<texture type="bitmap" name="k">
							<string name="filename" value="mitsuba.texture.rgbe"/>
						</texture>
					</bsdf>
				 </bsdf>
			 </bsdf>
        	<bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
				 <bsdf type="twosided">
					<bsdf type="conductor">
						<texture type="bitmap" name="eta">
							<string name="filename" value="mitsuba.blend.rgbe"/>
						</texture>
						<texture type="bitmap" name="k">
							<string name="filename" value="mitsuba.texture.rgbe"/>
						</texture>
					</bsdf>
				 </bsdf>
				<bsdf type="dielectric">
				</bsdf>
			 </bsdf>
		 </bsdf>
    </shape>
</scene>
`)

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
		pow(10, sin(17*t)+2),
		cos(19*t) * .5,
		cos(2*t)/3 + .666,
		sin(3*t)/3 + .666,
		sin(5*t)/3 + .666,
		.666 - cos(2*t)/3,
		.666 - sin(3*t)/3,
		.666 - sin(5*t)/3,
		sin(prime(3)*t) * 175,
		sin(prime(2)*t) * 175,
		sin(prime(1)*t) * 175,
		0,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
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
