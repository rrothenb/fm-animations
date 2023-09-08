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
	factor := tan(s.FOV * 1.25 / 360 * pi)
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
	return false
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

func spherePart(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/8.0) * cos(u),
		sin(v/8.0) * sin(u),
		1,
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
	return geom.Vec{0, 0, -10}
}

func focusPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 0}
}

func strength(x float64) float64 {
	return sin(x) + 1
}

func texture(u, v, t float64) float64 {
	nU := floor(sin(23*t) + 1.5)
	nV := floor(sin(29*t) + 1.5)
	return sin(
		nU*u + nV*v +
			strength(1.7+19*t)*sin(nU*u+strength(.7+7*t)*sin(nV*u+strength(.3+3*t)*sin(u-2*nU*v)*sin(4*nU*u+nU*v))) +
			strength(1.5+17*t)*sin(nV*v+strength(.5+5*t)*sin(nU*v+strength(.1+2*t)*sin(nV*u-2*nV*v)*sin(4*nU*u+v))) +
			strength(1.3+13*t)*sin(nU*u+nV*v))
}

func radius(u, v, t float64) float64 {
	return 1.0 - .1*pow(spow(texture(u, v, t), pow(6, sin(5*t)))/2+.5, pow(6, sin(7*t)))
}

func blendTexture(u, v, t float64) float64 {
	return pow(texture(u, v, t)/2+.5, pow(3, sin(7*t)))
}

func primaryMask(u, v, t float64) float64 {
	baseTexture := sin(texture(u, v, t))
	texture := 0.0
	if abs(baseTexture) < sin(2*t)*.25+.35 {
		texture = 1.0
	}
	return texture
}

func secondaryMask(u, v, t float64) float64 {
	baseTexture := sin(texture(u, v, t))
	texture := 0.0
	minThreshold := sin(2*t)*.25 + .55
	maxThreshold := .8
	if abs(baseTexture) > sin(3*t)*(maxThreshold-minThreshold)/2+(maxThreshold+minThreshold)/2 {
		texture = 1.0
	}
	return texture
}

func uv2xyz(u, v, t float64) geom.Vec {
	blendValue := pow(spow(texture(u, v, t), pow(2, cos(3*t)))/2+.5, pow(2, cos(7*t))) * sin(5*t)
	return geom.Vec{u - pi, v - pi, .1 * blendValue}
}

/*
func uv2xyz(u, v, t float64) geom.Vec {
	minV := sin(2*t)*pi/2+pi/2
	maxV := minV + sin(3*t)*pi/4+pi/2
	limitedV := minV + v/2/pi*(maxV-minV)
	a := 1-spow(cos(25*v), .5)*.5
	return pathWrapper(u, limitedV+pow(a-.5, 2)/15, .25*pow(sin(v/2+.7*sin(v)), .5)*a, outerKnot)
}
*/

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

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	globalT = t
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	distance := cameraLoc.Minus(focusPoint).Len()
	fov := 40.0
	c.FOV = fov
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
	minU := 1000
	minV := 1000
	maxU := 0
	maxV := 0
	closestPoint := geom.Vec{0, 0, 0}
	for uIndex := 0; uIndex <= 1000; uIndex++ {
		for vIndex := 0; vIndex <= 1000; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 1000), index2radians(float64(vIndex), 1000), t)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 1000), index2radians(float64(vIndex), 1000), t)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 1000), index2radians(float64(vIndex-1), 1000), t)
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
	midX := (minX + maxX) / 2
	midY := (minY + maxY) / 2
	midZ := (minZ + maxZ) / 2
	center := geom.Vec{midX, midY, midZ}
	focusPoint = focusPath(t)
	//focusPoint = geom.Vec{(minX+maxX)/2, (minY+maxY)/2, (minZ+maxZ)/2}
	//cameraLoc = cameraLoc.Scaled(math.Max(maxX, math.Max(maxY, maxZ))*8)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	// distance = cameraLoc.Minus(closestPoint).Len()
	distance = cameraLoc.Minus(focusPoint).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v, minZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ, minZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 1000)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 1000)
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
	primaryMaskArray := []float32{}
	secondaryMaskArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			blendValue := 1 - float32(pow(spow(texture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t), pow(2, cos(3*t)))/2+.5, pow(4, cos(5*t))))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			primaryMaskValue := float32(primaryMask(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
			primaryMaskArray = append(primaryMaskArray, primaryMaskValue, primaryMaskValue, primaryMaskValue)
			secondaryMaskValue := float32(secondaryMask(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
			secondaryMaskArray = append(secondaryMaskArray, secondaryMaskValue, secondaryMaskValue, secondaryMaskValue)

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
			envmapValue := float32(pow(sin(u/2), 60) * pow(sin(v), 60))
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
	primaryMaskPath := fmt.Sprintf("data/%v.primary.mask.rgbe", frameNumber)
	secondaryMaskPath := fmt.Sprintf("data/%v.secondary.mask.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	primaryMaskFile, _ := os.Create(primaryMaskPath)
	secondaryMaskFile, _ := os.Create(secondaryMaskPath)
	rgbe.Encode(primaryMaskFile, endUIndex-startUIndex, endVIndex-startVIndex, primaryMaskArray)
	rgbe.Encode(secondaryMaskFile, endUIndex-startUIndex, endVIndex-startVIndex, secondaryMaskArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera               geom.Vec
		LookAt               geom.Vec
		Distance             float64
		FogRadius            float64
		Angle                float64
		Weight1              int
		Weight2              int
		Weight3              int
		Weight4              int
		EnvX                 float64
		EnvY                 float64
		EnvZ                 float64
		FOV                  float64
		Rough1               float64
		Rough2               float64
		MinX                 float64
		MaxX                 float64
		MinY                 float64
		MaxY                 float64
		MinZ                 float64
		MaxZ                 float64
		MidX                 float64
		MidY                 float64
		MidZ                 float64
		BoundingSphereRadius float64
		Red                  float64
		Green                float64
		Blue                 float64
		Red2                 float64
		Green2               float64
		Blue2                float64
		IntIOR               float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="orthographic" id="Camera-camera">
        <transform name="to_world">
			<scale x="3.14159" y="3.14159"/>
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 1, 0"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="144"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="800"/>
            <integer name="height" value="800"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="10"/>
        <transform name="to_world">
            <rotate value="0, 1, 0" angle="135"/>
        </transform>
    </emitter>
    <integrator type="path" />
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
    	<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.primary.mask.rgbe"/>
			</texture>
			<bsdf type="null">
			</bsdf>
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
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough1 }}"/>
						<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						<rgb name="eta" value="1, 1, 1"/>
						</bsdf>
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
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough2 }}"/>
						<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						<rgb name="eta" value="1, 1, 1"/>
						</bsdf>
					</bsdf>
				</bsdf>
			</bsdf>
		</bsdf>
			</bsdf>
    </shape>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
    	<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.secondary.mask.rgbe"/>
			</texture>
			<bsdf type="null">
			</bsdf>
    	<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.blend.rgbe"/>
			</texture>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight3 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight4 }}"/>
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
					<float name="weight" value="{{ .Weight4 }}"/>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough1 }}"/>
						<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						<rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						</bsdf>
					</bsdf>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough1 }}"/>
						<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						<rgb name="eta" value="1, 1, 1"/>
						</bsdf>
					</bsdf>
				</bsdf>
			</bsdf>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight1 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight2 }}"/>
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
					<float name="weight" value="{{ .Weight2 }}"/>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough2 }}"/>
						<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						</bsdf>
					</bsdf>
				   <bsdf type="twosided">
						<bsdf type="roughconductor">
						<float name="alpha" value="{{ .Rough2 }}"/>
						<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						<rgb name="eta" value="1, 1, 1"/>
						</bsdf>
					</bsdf>
				</bsdf>
			</bsdf>
		</bsdf>
			</bsdf>
        <transform name="to_world">
            <translate x="0" y="0" z="-.01"/>
        </transform>
    </shape>
</scene>
`)
	r := 0.0
	r = max(r, center.Minus(geom.Vec{minX, minY, minZ}).Len())
	r = max(r, center.Minus(geom.Vec{minX, minY, maxZ}).Len())
	r = max(r, center.Minus(geom.Vec{minX, maxY, minZ}).Len())
	r = max(r, center.Minus(geom.Vec{minX, maxY, maxZ}).Len())
	r = max(r, center.Minus(geom.Vec{maxX, minY, minZ}).Len())
	r = max(r, center.Minus(geom.Vec{maxX, minY, maxZ}).Len())
	r = max(r, center.Minus(geom.Vec{maxX, maxY, minZ}).Len())
	r = max(r, center.Minus(geom.Vec{maxX, maxY, maxZ}).Len())

	fmt.Println("bounding sphere radius", r)
	fmt.Printf("frame: %v, weights: %v, %v, %v, %v\n", frameNumber, frameNumber%2, (frameNumber/2)%2, (frameNumber/4)%2, (frameNumber/8)%2)

	sensorTemplate.Execute(
		sensorFile,
		sensor{
			cameraLoc,
			focusPoint,
			distance,
			focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
			0,
			0, //frameNumber % 2,
			1, //(frameNumber / 2) % 2,
			1, //(frameNumber / 4) % 2,
			0, //(frameNumber / 8) % 2,
			sin(2*t) * 45,
			sin(3*t) * 45,
			sin(5*t) * 45,
			fov,
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
			r,
			cos(2*t)/2 + .5,
			cos(3*t)/2 + .5,
			cos(5*t)/2 + .5,
			.5 - cos(5*t)/2,
			.5 - cos(11*t)/2,
			.5 - cos(7*t)/2,
			1.5 + sin(7*t)*.25,
		})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 100, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	_ = flag.Float64("aspectratio", 1.0, "Aspect ratio")
	_ = flag.Int("height", 720, "Height")
	_ = flag.Int("samples", 25, "Samples")
	_ = flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
