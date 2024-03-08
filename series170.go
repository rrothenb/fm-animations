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
var frameNumber = 0
var globalT = 0.0
var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return pow(float64(primes[i-1]), 1)
}

func hsb2rgb(hue, sat, bri float64) (rgb geom.Vec) {
	u := bri
	if sat == 0 {
		rgb = geom.Vec{u, u, u}
	} else {
		h := (hue - math.Floor(hue)) * 6
		f := h - math.Floor(h)
		p := bri * (1 - sat)
		q := bri * (1 - sat*f)
		t := bri * (1 - sat*(1-f))
		switch int(h) {
		case 0:
			rgb = geom.Vec{u, t, p}
		case 1:
			rgb = geom.Vec{q, u, p}
		case 2:
			rgb = geom.Vec{p, u, t}
		case 3:
			rgb = geom.Vec{p, q, u}
		case 4:
			rgb = geom.Vec{t, p, u}
		case 5:
			rgb = geom.Vec{u, p, q}
		}
	}
	return
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
	return pow(4, sin(float64(n)*(x+float64(n)/10)))
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
	factor := tan(s.FOV * 1.5 / 360 * pi)
	if projectedPoint.X < projectedPoint.Z*factor*s.AspectRatio || projectedPoint.X > -projectedPoint.Z*factor*s.AspectRatio {
		return true
	}
	return false
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
	if projectedPoint.Z > 0.0 {
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
	center := path(q * t).Scaled(R)
	delta := .01
	normal, _ := path(q*t + delta).Scaled(R).Minus(path(q*t - delta).Scaled(R)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	a := -.2
	return cosVec.Scaled(r * cos(p*t-a*sin(2*p*t))).Plus(sinVec.Scaled(r * sin(p*t+a*sin(2*p*t))).Plus(center))
}

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{sin(float64(xN) * t), sin(float64(yN) * t), cos(float64(zN) * t)}
}

func unitLissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	point, _ := lissajousKnot(t, xN, yN, zN).Unit()
	return geom.Vec(point)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.48125, 3, 2, circle)
}

func middleKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.48125/3, 5, 3, outerKnot)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.48125/4, 7, 5, middleKnot)
}

func lastKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.48125/8, 11, 7, innerKnot)
}

func cameraPath(t float64) geom.Vec {
	return outerKnot(2 * t)
}

func focusPath(t float64) geom.Vec {
	return cameraPath(t - sin(3*t)*pi/2 + pi/2 + .01)
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v + delta).Minus(path(v - delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	a := -.2
	return cosVec.Scaled(r * cos(u-a*sin(2*u))).Plus(sinVec.Scaled(r * sin(u+a*sin(2*u)))).Plus(center)
}

func knot(t float64) geom.Vec {
	n := int(pow(2, 1-cos(globalT))+.5) * 2
	return lissajousKnot(t, 1+n, 2+n, 3+n)
}

func shapeTexture(f, a, t float64, loc geom.Vec) float64 {
	loc = loc.Scaled(f * 2 * pi)
	return sin(loc.Z +
		a*strength(7, t)*sin(a*strength(23, t)*loc.Z) +
		a*strength(11, t)*sin(a*strength(19, t)*loc.Z+a*strength(29, t)*sin(a*strength(31, t)*loc.Y)) +
		a*strength(13, t)*sin(a*strength(17, t)*loc.Z) + a*strength(37, t)*sin(a*strength(41, t)*loc.X-a*strength(43, t)*loc.Y) +
		a*strength(47, t)*sin(loc.Z+a*strength(53, t)*sin(loc.X*loc.Z+.1*a*strength(59, t)*sin(loc.Z*loc.Y))))
}

func cube(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0+.6*sin(v)) * cos(u-.6*sin(2*u)),
		sin(v/2.0+.6*sin(v)) * sin(u+.6*sin(2*u)),
		cos(v/2.0 - .6*sin(v)),
	}
}

func metalBlendValue(t float64, loc geom.Vec) float64 {
	return spow(pow(shapeTexture(2, 1, t, loc)/2+.5, 10)*2-1, .1)/2 + .5
}

func texture(u, v, t float64) float64 {
	v = v * pow(10, sin(47*t)+3)
	return sin(
		strength(37, t)*sin(4*u+strength(53, t)*sin(u))*sin(400*v+strength(61, t)*100*v) +
			strength(31, t)*sin(3*u+strength(43, t)*sin(v)) +
			strength(29, t)*sin(300*v+strength(47, t)*sin(u)) +
			strength(41, t)*sin(2*u+200*v+strength(59, t)*sin(u+v+strength(67, t)*sin(u-v))))

}

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, a)*2-1, b)
}

func theTexture(u, v, t float64) float64 {
	loc := innerKnot(v)
	a := sin(43*t)*.1 + .1
	return (1 - a - a*texture(u, v, t)) * shaper(shapeTexture(1*pow(4, sin(41*t)), .1*pow(2, sin(37*t)), t, loc), 1*pow(4, sin(31*t)), 100*pow(4, sin(29*t)))
}

func blendValue(u, v, t float64) float64 {
	return pow(theTexture(u, v, t)/2+.5, pow(2, sin(23*t)+2))
}

func shape(u, v, t float64) geom.Vec {
	return pathWrapper(u, v, .05*pow(2, sin(19*t))*theTexture(u, v, t), innerKnot)
}

func uv2xyz(u, v, t float64) geom.Vec {
	return shape(u, v, t)
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

func renderSurfaces(pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	width := int(aspectRatio * float64(height))
	t := float64(frameNumber) * dt
	globalT = t
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	mirrorLoc := focusPoint.Minus(cameraLoc).Scaled(.5)
	mirrorLoc2 := cameraLoc.Scaled(1.5)
	fov := 45.0 + sin(17*t)*15
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
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
	//cameraLoc = cameraLoc.Scaled(math.Max(maxX, math.Max(maxY, maxZ))*8)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v, minZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ, minZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	for nU < 8 {
		ratio = ratio * 1.5
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
	startUIndex := 0
	endUIndex := nU
	startVIndex := 0
	endVIndex := nV
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
	metalBlendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			loc := shape(u, v, t)
			// blendValue := float32((.5-cos(v/2-.7*sin(v))/2)*(.01*pow(spow(shapeTexture(3, 2, t, loc), pow(strength(5, t), 4))/2+.5, pow(strength(7, t), 4))))
			blendValue := float32(blendValue(u, v, t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			metalBlendValue := float32(metalBlendValue(t, loc))
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
	globalIllumination := sin(prime(40)*t)*.05 + .05
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			envmapValue := float32(pow(sin(u/2), pow(2, sin(31*t)*2+3)) * pow(sin(v), pow(2, sin(37*t)*2+3)))
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
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	metalBlend, _ := os.Create(metalBlendPath)
	rgbe.Encode(metalBlend, endUIndex-startUIndex, endVIndex-startVIndex, metalBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera       geom.Vec
		LookAt       geom.Vec
		Distance     float64
		FogRadius    float64
		Angle        float64
		MinZ         float64
		FOV          float64
		Aperture     float64
		Height       int
		Width        int
		Samples      int
		RowHeight    int
		IntIOR       float64
		ETA          geom.Vec
		K            geom.Vec
		G            float64
		Scale        float64
		Weight1      int
		Weight2      int
		MirrorLoc    geom.Vec
		MirrorLoc2   geom.Vec
		EmitterScale float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
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
        <float name="scale" value="{{ .EmitterScale }}"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="0"/>
        </transform>
    </emitter>
    <integrator type="volpathmis">
        <integer name="max_depth" value="16"/>
    </integrator>
    <medium id="medium1" type="homogeneous">
        <float name="scale" value="{{ .Scale }}"/>
        <rgb name="sigma_t" value="{{ .ETA.X }}, {{ .ETA.Y }}, {{ .ETA.Z }}"/>
        <rgb name="albedo" value="{{ .K.X }}, {{ .K.Y }}, {{ .K.Z }}"/>
        <phase type="hg">
            <float name="g" value="{{ .G }}"/>
        </phase>
    </medium>
    <bsdf type="blendbsdf" id="object_bsdf">
        <float name="weight" value="{{ .Weight1 }}"/>
    <bsdf type="blendbsdf">
        <float name="weight" value="{{ .Weight2 }}"/>
            <bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
                <bsdf type="dielectric">
                    <string name="ext_ior" value="air"/>
            		<float name="int_ior" value="{{ .IntIOR }}"/>
                </bsdf>
                <bsdf type="twosided">
                    <bsdf type="conductor">
                        <rgb name="k" value="{{ .ETA.X }}, {{ .ETA.Y }}, {{ .ETA.Z }}"/>
                        <rgb name="eta" value="{{ .K.X }}, {{ .K.Y }}, {{ .K.Z }}"/>
                    </bsdf>
                </bsdf>
            </bsdf>
            <bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
                <bsdf type="dielectric">
                    <string name="ext_ior" value="air"/>
            		<float name="int_ior" value="{{ .IntIOR }}"/>
                </bsdf>
                <bsdf type="twosided">
                    <bsdf type="conductor">
                        <rgb name="eta" value="{{ .ETA.X }}, {{ .ETA.Y }}, {{ .ETA.Z }}"/>
                        <rgb name="k" value="{{ .K.X }}, {{ .K.Y }}, {{ .K.Z }}"/>
                    </bsdf>
                </bsdf>
            </bsdf>
    </bsdf>
    <bsdf type="blendbsdf">
        <float name="weight" value="{{ .Weight2 }}"/>
            <bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
                <bsdf type="twosided">
                    <bsdf type="conductor">
                        <rgb name="k" value="{{ .ETA.X }}, {{ .ETA.Y }}, {{ .ETA.Z }}"/>
                        <rgb name="eta" value="{{ .K.X }}, {{ .K.Y }}, {{ .K.Z }}"/>
                    </bsdf>
                </bsdf>
                <bsdf type="dielectric">
                    <string name="ext_ior" value="air"/>
            		<float name="int_ior" value="{{ .IntIOR }}"/>
                </bsdf>
            </bsdf>
            <bsdf type="blendbsdf">
                <texture type="bitmap" name="weight">
                    <string name="filename" value="mitsuba.blend.rgbe"/>
                </texture>
                <bsdf type="twosided">
                    <bsdf type="conductor">
                        <rgb name="eta" value="{{ .ETA.X }}, {{ .ETA.Y }}, {{ .ETA.Z }}"/>
                        <rgb name="k" value="{{ .K.X }}, {{ .K.Y }}, {{ .K.Z }}"/>
                    </bsdf>
                </bsdf>
                <bsdf type="dielectric">
                    <string name="ext_ior" value="air"/>
            		<float name="int_ior" value="{{ .IntIOR }}"/>
                </bsdf>
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
        <ref id="medium1" name="interior"/>
    </shape>
    <shape type="ply">
        <string name="filename" value="trefoil.large.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
		<bsdf type="roughdielectric">
        	<float name="alpha" value=".001"/>
			<string name="int_ior" value="air"/>
            <float name="ext_ior" value="{{ .IntIOR }}"/>
		</bsdf>
   </shape>
</scene>
`)

	eta := hsb2rgb(sin(2*t)/2+.5, cos(7*t)*.1+.9, cos(11*t)*.1+.9)
	k := hsb2rgb(cos(3*t)/2+.5, cos(5*t)*.1+.9, cos(13*t)*.1+.9)

	if (frameNumber/4)%2 == 1 {
		temp := eta
		eta = k
		k = temp
	}

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		.1,
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		0,
		minZ,
		fov,
		.0000000000000001,
		height,
		width,
		samples,
		height / numRows,
		1 + pow(10, -cos(59*t)-1),
		eta,
		k,
		sin(53*t) * .99,
		pow(10, sin(61*t)/2+3),
		frameNumber % 2,
		(frameNumber / 2) % 2,
		mirrorLoc,
		mirrorLoc2,
		1 - globalIllumination,
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
	frameNumber = *frame
	renderSurfaces(*pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
