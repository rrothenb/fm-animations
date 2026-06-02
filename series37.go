//go:build series37
// +build series37

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
	return spow(sin(x), .5)*2
}

func radius(u, v, t float64) float64 {
	return 1.0 + .05*strength(2*t+.1)*spow(shapeTexture(u, v, t), pow(10, sin(3*t)))
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
		Width:    0.054,
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
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	//fmt.Printf("\npoint: %#v\nprojectedPoint: %#v\ncameraSpaceTransform: %#v\n", point, projectedPoint, cameraSpaceTransform)
	factor := .35
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
	return torusKnot(t, 1, .25, 2, 3, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .15, 3, 2, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	loc, _ := circle(t).Plus(geom.Vec{0,0,1.5*sin(3*t)}).Unit()
	return loc.Scaled(2+sin(5*t)*.5)
}

func focusPath(t float64) geom.Vec {
	return unitLissajousKnot(t, 2, 3, 5).Scaled((1.5-sin(t))/3)
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
	return pathWrapper(u, v, .25, circle)
}

func shapeTexture(u, v, t float64) float64 {
	loc := sphere(u, v, t).Scaled(5)
	loc2 := sphere(
		u+strength(7*t+.1)*sin(loc.X+loc.Y),
		v+strength(11*t+.2)*sin(loc.Z-loc.X+strength(13*t+.3)*sin(loc.Z+loc.Y)),
		t).Scaled(5)
	return sin(
		strength(17*t+.4)*sin(loc.X + loc2.Y + strength(19*t+.5)*sin(loc2.Z-loc.Y+strength(23*t+.6)*sin(loc.Z+loc2.X))))
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	loc := sphere(u, v, t)
	return loc.Scaled(radius(u, v, t))
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t, radius).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t, radius).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t, radius).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t, radius).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	envSize := 5000
	cameraLoc := cameraPath(t).Scaled(.075)
	focusPoint := focusPath(t).Scaled(.075)
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
	closestPoint := geom.Vec{0,0,0}
	for uIndex := 1; uIndex <= 500; uIndex++ {
		for vIndex := 1; vIndex <= 500; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
			if !c.invisible(vertex) {
				//fmt.Println(vertex)
				numTriangles++
				vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
				vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t, radius).Scaled(.075)
				totalWidth += vertex.Minus(vertexLeft).Len()
				totalHeight += vertex.Minus(vertexBelow).Len()
				minDistance = math.Min(minDistance, vertex.Len())
				maxDistance = math.Max(maxDistance, vertex.Len())
				maxX = math.Max(maxX, math.Abs(vertex.X))
				maxY = math.Max(maxY, math.Abs(vertex.Y))
				maxZ = math.Max(maxZ, math.Abs(vertex.Z))
				if (cameraLoc.Minus(closestPoint).Len() > cameraLoc.Minus(vertex).Len()) {
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
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth/totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	fmt.Printf("distance from center: %v, distance from focal point: %v, nU: %v, nV: %v\n", cameraLoc.Len(), distance, nU, nV)
	vertexIndicies := make([][]int32, nU+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= nU; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		for vIndex := 0; vIndex <= nV; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, radius).Scaled(.075)
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
	envmapArray := []float32{}
	blendArray := []float32{}
	metalBlendArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * pi
			// blendValue := float32(uvTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t, blendTexture, sphere))
			blendValue := float32(spow((sin(2*u+1000*v+strength(3*t)*sin(5*u+873*v))+sin(3*u+901*v+strength(2*t)*sin(u-1101*v)))/2, .1))/2+.5
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			metalBlendValue := float32(pow(spow(-spow(sin(2*t+.1), .25)*shapeTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t), pow(10, sin(7*t)*2))/2+.5, pow(2, sin(5*t)*2)))
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
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			envmapValue := float32(pow(sin(u/2)*sin(v), .5)*pow(spow(shapeTexture(u, v, t), .25)/2+.5, .25))
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
	rgbe.Encode(blend, nU, nV, blendArray)
	metalBlend, _ := os.Create(metalBlendPath)
	rgbe.Encode(metalBlend, nU, nV, metalBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera geom.Vec
		LookAt geom.Vec
		Distance float64
		FogRadius float64
		Angle float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value=".25"/>
        <float name="aperture_radius" value=".0001"/>
        <float name="fov" value="35"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="independent">
            <integer name="sample_count" value="256"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="5000"/>
            <integer name="height" value="5000"/>
            <rfilter type="box"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="-90"/>
            <rotate value="0, 0, 1" angle="{{ .Angle }}"/>
        </transform>
    </emitter>
    <integrator type="path" />
    <bsdf type="blendbsdf" id="object_bsdf">
        <texture type="bitmap" name="weight">
            <string name="filename" value="mitsuba.metal.blend.rgbe"/>
        </texture>
         <bsdf type="twosided">
            <bsdf type="roughconductor">
                <float name="alpha" value=".01"/>
                    <spectrum name="eta" filename="spd/2.spd"/>
                    <spectrum name="k" filename="spd/5.spd"/>
            </bsdf>
        </bsdf>
       <bsdf type="twosided">
            <bsdf type="roughconductor">
                <float name="alpha" value=".5"/>
                <string name="material" value="Ag"/>
            </bsdf>
        </bsdf>
    </bsdf>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <ref id="object_bsdf"/>
    </shape>
</scene>
`)
	angle := -t/pi*180
	if angle < -180  {
		angle = angle + 360
	}
	sensorTemplate.Execute(sensorFile,sensor{cameraLoc, focusPoint, distance, focusPoint.Minus(cameraLoc).Scaled(.5).Len(), angle})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 720, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
