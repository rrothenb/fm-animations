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
	"github.com/hunterloftis/pbr/pkg/material"
	"github.com/hunterloftis/pbr/pkg/surface"
)

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[i-1])
}

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

var tGlobal = 0.0

var tBase = 0.0

// series187 is series186 with one thing changed at a time, to find out which
// ingredient makes 186 read as a photograph of something rather than a render:
//
//	base       186 unchanged
//	dielectric thindielectric -> dielectric, so transmitted light bends again
//	nomedium   interior medium removed
//	noself     tube radius shrunk per frame until the tube no longer self-intersects
//	highior    index contrast floor lifted from ~1.0001 to 1.3
//	fixwinding the ~22% of faces whose winding points into the tube flipped
//	           outward, so the medium is consistently on the tube's inside
//	shell      a real shell: outer surface plus an inner surface offset inward by
//	           -thickness (a fraction of the tube radius), both dielectric, with the
//	           medium living in the shell between them
var variant = "base"
var shellThickness = .05

// Radius of the tube swept around innerKnot, in knot units. 186 fixes it at 1.5,
// which makes the tube pass through itself many times over.
var tubeRadius = 1.5

var imageWidth = 1024
var samples = 256

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
	return sin(x)*.75 + 1.25
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
	Width  float64
	Height float64
	Lens   float64
	FStop  float64
	Focus  float64
	FOV    float64

	trans    *geom.Mtx
	position geom.Vec
	target   geom.Vec
}

var zAxis = geom.Dir{0, 0, 1}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
		Width:    0.02,
		Height:   0.03,
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
	aspectRatio := s.Width / s.Height
	if projectedPoint.X < projectedPoint.Z*factor*aspectRatio || projectedPoint.X > -projectedPoint.Z*factor*aspectRatio {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
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
		cos(v / 2.0),
	}
}

func foldedSphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u) * (1.25 - pow(cos(7*v/2.0), 2)),
		sin(v/2.0) * sin(u) * (1.25 - pow(cos(7*v/2.0), 2)),
		cos(7 * v / 2.0),
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
	return torusKnot(t, 1, .5+sin(2*tGlobal)*.49, 5, 7, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5+cos(3*tGlobal)*.49, 7, 5, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	loc, _ := circle(t).Plus(geom.Vec{0, 0, .75 + 1*sin(2*t)}).Unit()
	return loc.Scaled(5 + sin(t)*2)
}

func focusPath(t float64) geom.Vec {
	return unitLissajousKnot(t+2, 3, 4, 5).Scaled(.4 - sin(5*t)*.2)
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

func blendValue(u, v, t float64) float64 {
	loc := pathWrapper(u, v, 1.5, innerKnot)
	return shaper(texture(1+.01*texture(pow(10, cos(prime(2)*t)-1), loc.X, loc.Y, loc.Z, t), loc.X, loc.Y, loc.Z, t), sin(prime(5)*t), 0)/2 + .5
}

func texture(a, x, y, z, t float64) float64 {
	x = x * 2 * pi
	y = y * 2 * pi
	z = z * 2 * pi
	return sin(
		x + y +
			a*strength(.1+2*t)*sin(x+a*strength(.2+3*t)*sin(x)) +
			a*strength(.3+5*t)*sin(y+a*strength(.4+7*t)*sin(y)) +
			a*strength(.5+5*t)*sin(z+a*strength(.6+7*t)*sin(z)) +
			a*strength(.7+11*t)*sin(2*x+3*y) +
			a*strength(.8+13*t)*sin(2*y-3*z) +
			a*strength(.9+17*t)*sin(3*z-5*x))
}

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

func uv2xyz(u, v, t float64) geom.Vec {
	loc := pathWrapper(u, v, tubeRadius, innerKnot)
	return loc.Scaled(1 + .01*texture(pow(10, cos(prime(2)*t)-1), loc.X, loc.Y, loc.Z, t))
}

// selfIntersectionFreeRadius shrinks the tube until no two stretches of the
// centreline that are more than half a turn of tube apart (arc length > pi*r)
// come within 2r of each other, then backs off a further 10% to leave room for
// the 1% texture displacement.
func selfIntersectionFreeRadius() float64 {
	n := 3000
	points := make([]geom.Vec, n)
	arc := make([]float64, n+1)
	for i := range points {
		points[i] = innerKnot(float64(i) / float64(n) * 2 * pi)
		if i > 0 {
			arc[i] = arc[i-1] + points[i].Minus(points[i-1]).Len()
		}
	}
	length := arc[n-1] + points[0].Minus(points[n-1]).Len()
	for r := 1.5; r > .005; r *= .9 {
		clear := true
		for i := 0; i < n && clear; i++ {
			for j := i + 1; j < n; j++ {
				s := arc[j] - arc[i]
				if min(s, length-s) > pi*r && points[i].Minus(points[j]).Len() < 2*r {
					clear = false
					break
				}
			}
		}
		if clear {
			return r * .9
		}
	}
	return .005
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	tGlobal = t
	tBase = 5.9375 * dt
	if variant == "noself" {
		tubeRadius = selfIntersectionFreeRadius()
	}
	fmt.Printf("variant: %v, tube radius: %v\n", variant, tubeRadius)
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t).Scaled(.075)
	focusPoint := geom.Vec{0, 0, 0}
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FOV = 35
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
	maxX := 0.0
	maxY := 0.0
	maxZ := 0.0
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 1; uIndex <= 500; uIndex++ {
		for vIndex := 1; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t).Scaled(.075)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t).Scaled(.075)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t).Scaled(.075)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, math.Abs(vertex.X))
				maxY = math.Max(maxY, math.Abs(vertex.Y))
				maxZ = math.Max(maxZ, math.Abs(vertex.Z))
				if cameraLoc.Minus(closestPoint).Len() > cameraLoc.Minus(vertex).Len() {
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
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	fmt.Printf("distance from center: %v, distance from focal point: %v, nU: %v, nV: %v\n", cameraLoc.Len(), distance, nU, nV)
	vertexIndicies := make([][]int32, nU+1)
	vertexPositions := make([][]geom.Vec, nU+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	// The shell's inner surface: the outer one pulled in along its outward normal.
	// Same vertex count and indices, so the face list can be written to both.
	shell := variant == "shell"
	var innerBuffered *bufio.Writer
	if shell {
		innerData, _ := os.Create(fmt.Sprintf("data/%v.inner.data.ply", frameNumber))
		innerBuffered = bufio.NewWriter(innerData)
	}
	offset := shellThickness * tubeRadius * .075
	writeVertex := func(w *bufio.Writer, vertex geom.Vec, normal geom.Dir, uIndex, vIndex int) {
		binary.Write(w, binary.LittleEndian, float32(vertex.X))
		binary.Write(w, binary.LittleEndian, float32(vertex.Y))
		binary.Write(w, binary.LittleEndian, float32(vertex.Z))
		binary.Write(w, binary.LittleEndian, float32(normal.X))
		binary.Write(w, binary.LittleEndian, float32(normal.Y))
		binary.Write(w, binary.LittleEndian, float32(normal.Z))
		binary.Write(w, binary.LittleEndian, float32(index2radians(float64(uIndex), nU)/pi/2))
		binary.Write(w, binary.LittleEndian, float32(index2radians(float64(vIndex), nV)/pi/2))
	}
	// radial points from the tube's centreline out through a point on its surface.
	radial := func(p geom.Vec, vIndex int) geom.Vec {
		return p.Minus(innerKnot(index2radians(float64(vIndex), nV)).Scaled(.075))
	}
	for uIndex := 0; uIndex <= nU; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		vertexPositions[uIndex] = make([]geom.Vec, nV+1)
		for vIndex := 0; vIndex <= nV; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t).Scaled(.075)
			if c.invisible(vertex) {
				vertexIndicies[uIndex][vIndex] = -1
				continue
			}
			normal := uvIndexToNormal(uIndex, vIndex, nU, nV, t)
			vertexIndicies[uIndex][vIndex] = int32(numVerticies)
			vertexPositions[uIndex][vIndex] = vertex
			numVerticies++
			writeVertex(PlyDataBuffered, vertex, *normal, uIndex, vIndex)
			if shell {
				outward := *normal
				if geom.Vec(outward).Dot(radial(vertex, vIndex)) < 0 {
					outward = outward.Inv()
				}
				writeVertex(innerBuffered, vertex.Minus(geom.Vec(outward).Scaled(offset)), outward, uIndex, vIndex)
			}
		}
	}
	outwardFaces := 0
	envmapArray := []float32{}
	blendArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			blendValue := float32(blendValue(u, v, t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)

			topRight := vertexIndicies[uIndex][vIndex]
			topLeft := vertexIndicies[uIndex+1][vIndex]
			botRight := vertexIndicies[uIndex][vIndex+1]
			botLeft := vertexIndicies[uIndex+1][vIndex+1]
			if topRight == -1 || topLeft == -1 || botRight == -1 || botLeft == -1 {
				continue
			}
			// 186's winding, kept as-is for every variant but shell. Mitsuba picks
			// the interior medium from the geometric normal, so count how often it
			// faces out of the tube; shell and fixwinding flip any face that doesn't.
			pTR, pBL, pTL := vertexPositions[uIndex][vIndex], vertexPositions[uIndex+1][vIndex+1], vertexPositions[uIndex+1][vIndex]
			outward := pBL.Minus(pTR).Cross(pTL.Minus(pTR)).Dot(radial(pTR, vIndex)) >= 0
			if outward {
				outwardFaces++
			}
			faces := [2][3]int32{{topRight, botLeft, topLeft}, {topRight, botRight, botLeft}}
			if (shell || variant == "fixwinding") && !outward {
				faces = [2][3]int32{{topRight, topLeft, botLeft}, {topRight, botLeft, botRight}}
			}
			for _, face := range faces {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, face)
				if shell {
					binary.Write(innerBuffered, binary.LittleEndian, byte(3))
					binary.Write(innerBuffered, binary.LittleEndian, face)
				}
				numFaces++
			}
		}
	}
	fmt.Printf("faces whose winding points out of the tube: %.1f%%\n", 100*float64(outwardFaces)/float64(numFaces/2))
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			power := 2 * pow(10, sin(5*t)/2+.5)
			envmapValue := float32(pow(sin(u/2), power) * pow(sin(v), power))
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
	if shell {
		innerBuffered.Flush()
		innerHeader, _ := os.Create(fmt.Sprintf("data/%v.inner.header.ply", frameNumber))
		tmpl.Execute(innerHeader, mesh)
	}
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, nU, nV, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera        geom.Vec
		LookAt        geom.Vec
		Distance      float64
		IntIOR        float64
		ExtIOR        float64
		G             float64
		Abbe          float64
		FilmThickness float64
		FilmIOR       float64
		Roughness     float64
		Albedo        float64
		SigmaT        float64
		EnvX          float64
		EnvY          float64
		EnvZ          float64
		Width         int
		Samples       int
		MaxDepth      int
		BSDF          string
		Medium        bool
		Shell         bool
	}
	// The film/no-film blend 186 puts on its one surface, parameterised by which
	// side of the surface is which so the shell's inner face can reuse it.
	sensorTemplate, _ := template.New("some template").Parse(`
{{ define "blend" }}
		<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.blend.rgbe"/>
				<boolean name="raw" value="true"/>
			</texture>
			<bsdf type="{{ .BSDF }}">
				<float name="film_thickness" value="{{ .FilmThickness }}"/>
				<float name="film_ior" value="{{ .FilmIOR }}"/>
				<float name="int_ior" value="{{ .Int }}"/>
				<float name="ext_ior" value="{{ .Ext }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
			<bsdf type="{{ .BSDF }}">
				<float name="int_ior" value="{{ .Int }}"/>
				<float name="ext_ior" value="{{ .Ext }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
		</bsdf>
{{ end }}
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value="0.0000000000001"/>
        <float name="fov" value="35"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="{{ .Samples }}"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="{{ .Width }}"/>
            <integer name="height" value="{{ .Width }}"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="{{ .EnvX }}"/>
            <rotate value="0, 1, 0" angle="{{ .EnvY }}"/>
            <rotate value="0, 0, 1" angle="{{ .EnvZ }}"/>
        </transform>
    </emitter>
        <integrator type="volpathmis">
            <integer name="max_depth" value="{{ .MaxDepth }}"/>
        </integrator>
    <medium id="medium1" type="homogeneous">
        <float name="albedo" value="{{ .Albedo }}"/>
        <float name="sigma_t" value="{{ .SigmaT }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
    <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        {{ template "blend" (call .Side .IntIOR .ExtIOR) }}
        {{ if .Medium }}<ref id="medium1" name="interior"/>{{ end }}
    </shape>
    {{ if .Shell }}
    <!-- Inner face of the shell: its normal points out into the shell, so the
         shell's medium and index are on its exterior side, the hollow's on its interior. -->
    <shape type="ply">
        <string name="filename" value="mitsuba.inner.ply"/>
        {{ template "blend" (call .Side .ExtIOR .IntIOR) }}
        <ref id="medium1" name="exterior"/>
    </shape>
    {{ end }}
</scene>
`)

	// 1 + 10^(cos-3) puts the index contrast in [1.0001, 1.01]. highior keeps the
	// same sweep but stands it on a 1.3 floor, landing in [1.302, 1.5].
	intIor := 1 + pow(10, cos(prime(32)*t)-3)
	if variant == "highior" {
		intIor = 1.3 + 20*(intIor-1)
	}
	extIor := 1.0
	if frameNumber%2 == 1 {
		extIor = intIor
		intIor = 1.0
	}

	// Pick the interference order / optical thickness you want to sweep through.
	// Order ~0.5–2 keeps you in the vivid, 4-wavelength-resolvable zone.
	opticalThickness := 100 + (sin(prime(33)*t)/2+0.5)*650 // n·d ∈ [100, 750] nm
	filmIor := 1.8 + sin(prime(34)*t)*.7                   // choose for material look
	filmThickness := opticalThickness / filmIor            // d follows from the two

	// Target dispersion strength: subtle to noticeable "flint-like" fire,
	// staying out of the rutile-grade noisy tail.
	dispStrength := 0.005 + (sin(prime(35)*t)/2+0.5)*0.020 // Δn ∈ [0.005, 0.025]
	abbe := (max(intIor, extIor) - 1) / dispStrength       // derived, not swept

	bsdf := "thindielectric"
	if variant == "dielectric" || variant == "shell" {
		bsdf = "dielectric"
	}
	maxDepth := 16
	if variant == "shell" {
		// Every layer now costs two bounces instead of one.
		maxDepth = 32
	}
	params := sensor{
		cameraLoc,
		focusPoint,
		distance,
		intIor,
		extIor,
		cos(prime(36)*t) * .95,
		abbe,
		filmThickness,
		filmIor,
		pow(10, sin(prime(37)*t)*4-4),
		sin(prime(38)*t)*.5 + .5,
		pow(10, sin(prime(39)*t)*4-4),
		sin(prime(40)*t) * 179,
		sin(prime(41)*t) * 179,
		sin(prime(42)*t) * 179,
		imageWidth,
		samples,
		maxDepth,
		bsdf,
		variant != "nomedium",
		variant == "shell",
	}
	type side struct {
		BSDF, FilmThickness, FilmIOR, Int, Ext, Abbe any
	}
	sensorTemplate.Execute(sensorFile, struct {
		sensor
		Side func(i, e float64) side
	}{params, func(i, e float64) side {
		return side{bsdf, filmThickness, filmIor, i, e, abbe}
	}})
	fmt.Printf("ior: %v\n", max(intIor, extIor))
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.StringVar(&variant, "variant", variant, "base, dielectric, nomedium, noself, highior or shell")
	flag.Float64Var(&shellThickness, "thickness", shellThickness, "shell thickness as a fraction of the tube radius")
	flag.IntVar(&imageWidth, "width", imageWidth, "Image width and height")
	flag.IntVar(&samples, "samples", samples, "Samples per pixel")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
