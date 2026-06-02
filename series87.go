//go:build ignore

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
	// "github.com/hunterloftis/pbr/pkg/surface"
	// "github.com/hunterloftis/pbr/pkg/material"
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
var atan2 = math.Atan2
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

func strength(x float64) float64 {
	return sin(x)*.75 + .75
}

func subtexture2(u, v, t float64) float64 {
	return sin(7*u+v*strength(61*t)*subtexture3(u, v, t))+sin(5*v+v*strength(59*t)*subtexture3(u, v, t))
}

func subtexture3(u, v, t float64) float64 {
	return sin(2*u+3*v)
}

func texture(u, v, t float64) float64 {
	v = 2*pi - v
	return sin(3*u-3*v+v*strength(67*t)*subtexture2(3*u, 3*v, t))*sin(3*u+3*v+v*strength(71*t)*subtexture2(3*v, 3*u, t))
}

func textureOriginal(u, v, t float64) float64 {
	return sin(
		3*u + 5*v + strength(.1+2*t)*sin(
			2*u+strength(.2+3*t)*sin(3*u)) + strength(.3+5*t)*sin(
			7*v+strength(.4+7*t)*sin(5*v)) + strength(.5+11*t)*sin(
			11*u+13*v) + strength(.6+13*t)*sin(17*u-5*v) + strength(.7+17*t)*sin(23*v-11*u))
}

func radiusOriginal(u, v, t float64) float64 {
	return 1.0 + .075*pow(pow(texture(u, v, t), 2), 3)
}

func blend(u, v, t float64) float64 {
	return spow(texture(u, v, t), sin(5*t)*.1+.3)/2+.5
}

func uvTexture(u, v, t float64, texture func (x, y, z, t float64) float64, shape func (u, v, t float64) geom.Vec) float64 {
	loc := shape(u, v, t)
	return texture(loc.X, loc.Y, loc.Z, t)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2-1
}

func pushout(x, n float64) float64 {
	return spow(x*2-1, n)/2+.5
}

func combinedTexture(u, v, t float64) float64 {
	textureBlend := spow(cos(73*t), .5)/2+.5
	return textureBlend*texture(u, v, t)+(1-textureBlend)*textureOriginal(u, v, t)
}

func radius(u, v, t float64) float64 {
	return  1+(sin(t)*.4+.5)*.1*pow(spow(combinedTexture(u, v, t), pow(10, sin(17*t)-1))/2+.5, pow(10, sin(23*t)))*spow(sin(v+.5*sin(19*v)), .5)
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
		Width:    0.09,
		Height:   0.16,
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
	factor := .5
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

func isCurved(v float64) float64 {
	return spow(cos(v/2.0-.7*sin(v)), .1)/2+.5
}

func halfSphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v/2.0) * isCurved(v),
	}
}

func bowl(u, v, thickness, percent float64) geom.Vec {
	v2 := 2*pi*(sin(v/2))*percent
	thickness = thickness*percent
	r := 1 + spow(sin(v+.7*sin(2*v)), .25)*thickness/2
	return geom.Vec{
		sin(v2/2.0) * cos(u),
		sin(v2/2.0) * sin(u),
		cos(v2/2.0),
	}.Scaled(r)
}

func boysSurface(u, v, t float64) geom.Vec {
	return geom.Vec{
		(cos(u)*cos(2*v) + sqrt(2)*sin(u)*cos(v)) * cos(u) / (sqrt(2) - sin(2*u)*sin(3*v)),
		(cos(u)*sin(2*v) - sqrt(2)*sin(u)*sin(v)) * cos(u) / (sqrt(2) - sin(2*u)*sin(3*v)),
		sqrt(2) * pow(cos(u), 2) / (sqrt(2) - sin(2*u)*sin(2*v)),
	}
}

func sinG(x, a, b float64) float64 {
	return sin(x+a*sin(2*x)+b/2*sin(4*x))
}

func cosG(x, a, b float64) float64 {
	return cos(x-a*sin(2*x)+b/2*sin(4*x))
}

func generalizedSphere(u,v,t float64) geom.Vec {
	v = v/2
	a := .4-.4*cos(6*t)
	b := .4-.4*cos(3*t)
	c := .4-.4*cos(4*t)
	d := .4-.4*cos(5*t)
	e := .4-.4*cos(2*t)
	f := .4-.4*cos(7*t)
	return geom.Vec{
		sinG(v, a, b) * cosG(u, c, d),
		sinG(v, a, b) * sinG(u, c, d),
		cosG(v, e, f),
	}
}

// maybe torusKnot should have a path input and for a regular torus knot it's a circle but for a cable know it's a torusKnot
func torusKnot(t, R, r float64, pInt, qInt int, path func(x float64) geom.Vec) geom.Vec {
	p := float64(pInt)
	q := float64(qInt)
	pathPoint := path(q*t)
	return geom.Vec{(R+r*cos(p*t))*pathPoint.X, (R+r*cos(p*t))*pathPoint.Y, r*sin(p*t)+pathPoint.Z}
}

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{sin(float64(xN)*t), sin(float64(yN)*t), cos(float64(zN)*t)}
}

func unitLissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	point, _ := lissajousKnot(t, xN, yN, zN).Unit()
	return geom.Vec(point)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .49, 11, 13, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .4, 13, 11, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	altitude := sin(11*t)*.3+.6
	scaledAltitude := altitude*(sin(t)*.4+.5)
	radiusMultiple := 1-scaledAltitude
	fmt.Printf("\naltitude: %v\nscaledAltitude: %v\nradiusMultiple: %v\nt: %#v\n", altitude, scaledAltitude, radiusMultiple, t)
	return bowl(29*t, pow(sin(31*t)/2+.5, .25)*pi/2+pi/4, 0, sin(t)*.4+.5).Scaled(radiusMultiple)
}

func focusPath(t float64) geom.Vec {
	return bowl(47*t+1+sin(t)*.9, pow(sin(41*t)/2+.5, 1.5)*pi/2+pi/4*5, .1, sin(t)*.4+.5)
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
	return unitLissajousKnot(t, 19, 20, 21)
}

func shape(u, v, t float64) geom.Vec {
	return bowl(u, v, .1, sin(t)*.4+.5)
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	return shape(u, v, t).Scaled(radius(u, v, t))
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
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
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
	closestPoint := geom.Vec{0,0,0}
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
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	//distance = cameraLoc.Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth/totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	for nV > 30000 && nV > nU {
		ratio = ratio*2
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
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
	blendArray := []float32{}
	landBlendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			blendValue := float32(blend(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			landBlendValue := float32(spow(combinedTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t), .1)/2+.5)
			landBlendArray = append(landBlendArray, landBlendValue, landBlendValue, landBlendValue)

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
			envmapValue := float32(pow(sin(u/2), 2)*pow(sin(v), 2))
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
	landBlendPath := fmt.Sprintf("data/%v.land.blend.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	landBlend, _ := os.Create(landBlendPath)
	rgbe.Encode(landBlend, endUIndex-startUIndex, endVIndex-startVIndex, landBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera geom.Vec
		LookAt geom.Vec
		Distance float64
		FogRadius float64
		Angle float64
		MinZ float64
		Scale float64
		Metal1 float64
		Metal2 float64
		MarkerSize float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".00001"/>
        <float name="fov" value="45"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, -1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="420"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="1800"/>
            <integer name="height" value="3200"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="180"/>
        </transform>
    </emitter>
        <integrator type="path">
        </integrator>
    <bsdf type="blendbsdf" id="object_bsdf">
		<texture type="bitmap" name="weight">
			<string name="filename" value="mitsuba.land.blend.rgbe"/>
		</texture>
    	<bsdf type="blendbsdf">
			<float name="weight" value="{{ .Metal1 }}"/>
		   <bsdf type="twosided">
				<bsdf type="roughconductor">
				<float name="alpha" value=".01"/>
				<spectrum name="eta" filename="spd/15i.spd"/>
				<spectrum name="k" filename="spd/11i.spd"/>
				</bsdf>
			</bsdf>
		   <bsdf type="twosided">
				<bsdf type="roughconductor">
				<float name="alpha" value=".01"/>
				<string name="material" value="Ag"/>
				</bsdf>
			</bsdf>
		</bsdf>
    	<bsdf type="blendbsdf">
			<float name="weight" value="{{ .Metal2 }}"/>
		   <bsdf type="twosided">
				<bsdf type="roughconductor">
				<float name="alpha" value=".01"/>
				<spectrum name="eta" filename="spd/27.spd"/>
				<spectrum name="k" filename="spd/10.spd"/>
				</bsdf>
			</bsdf>
		   <bsdf type="twosided">
				<bsdf type="roughconductor">
				<float name="alpha" value=".01"/>
				<spectrum name="eta" filename="spd/2.spd"/>
				<spectrum name="k" filename="spd/3.spd"/>
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
    </shape>
</scene>
`)
	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		180+65-t/2/pi*360,
		minZ,
		pow(10, cos(53*t)+1),
		spow(cos(13*t), .5)/2+.5,
		spow(cos(17*t), .5)/2+.5,
	distance*.05})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
