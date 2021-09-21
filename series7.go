// This is actually 6b at this point (still torus)
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
	return sin(x)*.4 + .5
}

func subtexture2(u, v, t float64) float64 {
	return sin(5*u+7*v+strength(4*t)*subtexture3(u, v, t))
}

func subtexture3(u, v, t float64) float64 {
	return sin(2*u+3*v)
}

func texture(u, v, t float64) float64 {
	v = 97*v
	return spow(sin(u + strength(1+3*t)*subtexture2(u, v, t)) *
			sin(v + strength(2+5*t)*subtexture2(0, v, t)) *
			sin(u - v + strength(3+7*t)*subtexture2(u, v, t)) *
			sin(u + v + strength(4+11*t)*subtexture2(u, v, t)), .25)/2+.5
}

func radius(u, v, t float64) float64 {
	return 1.0 - .9*texture(u, v, t)
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

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
		Width:    0.024,
		Height:   0.024,
		Lens:     0.050, // 50mm focal length
		FStop:    4,
		Focus:    1,
		position: geom.Vec{0, 0, 0},
		target:   geom.Vec{0, 0, -5},
	}
	s.transform()
	return s
}

// LookAt orients a Camera to face a target.
func (s *SLR2) LookAt(target geom.Vec) *SLR2 {
	s.target = target
	s.transform()
	return s
}

// MoveTo moves a Camera to a position given by x, y, and z coordinates.
func (s *SLR2) MoveTo(pos geom.Vec) *SLR2 {
	s.position = pos
	s.transform()
	return s
}

func (s *SLR2) transform() {
	s.trans = geom.LookMatrix(s.position, s.target)
}

func (s *SLR2) invisible(point geom.Vec) bool {
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	if projectedPoint.X < projectedPoint.Z/2 || projectedPoint.X > -projectedPoint.Z/2 {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z/2 || projectedPoint.Y > -projectedPoint.Z/2 {
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
	pathPoint := path(q*t)
	return geom.Vec{(R+r*cos(p*t))*pathPoint.X, (R+r*cos(p*t))*pathPoint.Y, r*sin(p*t)+pathPoint.Z}
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5, 2, 5, circle)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .25, 2, 5, innerKnot)
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v+delta).Minus(path(v-delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r*cos(u)).Plus(sinVec.Scaled(r*sin(u))).Plus(center)
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	r := .125*a
	return pathWrapper(u, v, r, outerKnot)
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
	cameraLoc := geom.Vec{0, math.Sin(t), math.Cos(t)}.Scaled(.2)
	focusPoint := geom.Vec{0, 0, 0}
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	// distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	distance := cameraLoc.Len() + .015
	// TODO replace n with nU and nV
	nU := int(float64(pixels) / distance * 3)
	if nU > maxSubdivisions {
		nU = maxSubdivisions
	}
	nV := nU
	if desiredTriangles > 0 {
		numTriangles := 0
		totalWidth := 0.0
		totalHeight := 0.0
		for uIndex := 1; uIndex <= 500; uIndex++ {
			for vIndex := 1; vIndex <= 500; vIndex++ {
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
				if !c.invisible(vertex) {
					numTriangles++
					vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
					vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t, radius).Scaled(.075)
					totalWidth += vertex.Minus(vertexLeft).Len()
					totalHeight += vertex.Minus(vertexBelow).Len()
				}
			}
		}
		ratio := totalWidth/totalHeight
		fmt.Println(totalWidth, totalHeight, ratio)
		fmt.Println(numTriangles)
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
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
	roughnessArray := []float32{}
	blendArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
			//u := float64(uIndex) / float64(n) * 2 * pi
			v := float64(vIndex) / float64(nV) * pi
			envmapValue := float32(.025+pow(sin(v/2),20)) // float32((1-pow(1-pow(texture(u, v, t), 2), pow(1-v/pi, 3)*10)))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
			roughnessValue := float32(0.0) // float32(pow(spow(subtexture1(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t), .5)*.5+.5, 2))*.5+.1
			blendValue := float32(pushout(texture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t),  .1))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
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
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, nU, nV, envmapArray)
	roughness, _ := os.Create(roughnessPath)
	rgbe.Encode(roughness, nU, nV, roughnessArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, nU, nV, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Loc geom.Vec
		Distance float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="smaller"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".001"/>
        <float name="fov" value="35"/>
        <transform name="to_world">
            <lookat target="0, 0, 0" origin="{{ .Loc.X }}, {{ .Loc.Y }}, {{ .Loc.Z }}" up="1, 0, 0"/>
        </transform>

        <sampler type="independent">
            <integer name="sample_count" value="256"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="3800"/>
            <integer name="height" value="3800"/>
            <string name="pixel_format" value="rgb"/>
            <rfilter type="gaussian"/>
        </film>
    </sensor>
</scene>
`)
	fmt.Println(cameraLoc)
	sensorTemplate.Execute(sensorFile,sensor{cameraLoc, distance})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 200, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
