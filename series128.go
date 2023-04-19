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

func initSinfts() [100]float64 {
	array := [100]float64{}
	for i := 0; i < len(array); i++ {
		array[i] = math.NaN()
	}
	return array
}

var sinfts = initSinfts()

var globalT = 0.0

func sinft(f int) float64 {
	if math.IsNaN(sinfts[f-1]) {
		sinfts[f-1] = sin(float64(f+50)*globalT + float64(f)/10)
	}
	return sinfts[f-1]
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

func strength(f int) float64 {
	return pow(2, sinft(f)+1)
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

var zAxis = geom.Dir{0, 1, 0}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
		Width:    0.10,
		Height:   0.10,
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
	a := .25 - cos(17*t)*.25
	b := .25 - cos(19*t)*.25
	c := .25 - cos(23*t)*.25
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-b*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+b*sin(2*u)),
		cos(v/2.0 - c*sin(v)),
	}
}

func baseShape(u, v, t float64) geom.Vec {
	return sphere(u, v, t)
}

func cameraPath(t float64) geom.Vec {
	return baseShape(2*t, 3*t, t).Scaled(2)
}

func focusPath(t float64) geom.Vec {
	v := 3*t - floor(3*t/(2*pi))*2*pi
	maxDV := min(2*pi-v, pi/2)
	minDV := maxDV / 2
	dV := (sin(2*t)/2+.5)*(maxDV-minDV) + minDV
	return baseShape(2*t+pi/3+pi/6*sin(3*t), v+dV, t)
}

func shape(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

// 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97
func texture(u, v, t float64) float64 {
	vFreq := floor(14 + sinft(13)*10)
	uFreq := floor((sinft(14)/2+.5)*(vFreq-4) + 4)
	a := sinft(15) * .5
	b := sinft(16)
	c := 1 / strength(1)
	d := 1 / strength(2)
	e := 1 / strength(3)
	f := 1 / strength(4)
	g := 1 / strength(5)
	h := 1 / strength(6)
	i := 1 / strength(7)
	j := 1 / strength(8)
	k := 1 / strength(9)
	l := 1 / strength(10)
	m := 1 / strength(11)
	n := 1 / strength(12)
	return pow(shape(blend(sinft(16)*.25+.75,
		subtexture(vFreq, 0, c, f, g, l, m, n, u, v),
		subtexture(vFreq-uFreq, uFreq, d, h, i, e, j, k, u, v),
	), a, b), 2)
}

func and(a, b float64) float64 {
	return a * b
}
func or(a, b float64) float64 {
	return 1 - (1-a)*(1-b)
}

func blend(a, b, c float64) float64 {
	return a*and(b, c) + (1-a)*or(b, c)
}

func subtexture(vF, uF, a1, a2, a3, a4, a5, a6, u, v float64) float64 {
	return sin(vF*v + uF*u +
		a1*sin(v+a2*sin(u)+a3*sin(v)) +
		a4*sin(u+a5*sin(u)+a6*sin(v)))
}

func uv2xyz(u, v, t float64) geom.Vec {
	tuv := texture(pow(10, floor(sinft(17)+2))*u, pow(10, floor(sinft(18)+2))*v, t)
	tvu := texture(pow(10, floor(sinft(19)+2))*v, pow(10, floor(sinft(20)+2))*u, t)
	microTexture := blend(sinft(21)/2+.5, tuv, tvu)
	return sphere(u, v, t).Scaled(1 - .25*texture(u, v, t) - .0025*microTexture*(1-metalBlendTexture(u, v, t)))
}

func metalBlendTexture(u, v, t float64) float64 {
	return texture(u, v, t)

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

func renderSurfaces(pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	globalT = t
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	fov := 50.0 + 10*sinft(22)
	c.FOV = fov
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
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	//distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	weight := sinft(36)/2 + .5
	distance = weight*distance + (1-weight)*cameraLoc.Minus(closestPoint).Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	for nV > 15000 {
		ratio = ratio * 10
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
	metalBlendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			metalBlendValue := float32(metalBlendTexture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
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
			envmapValue := float32(pow(sin(u/2)*sin(v), pow(2, sinft(23)+2)))
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
	metalBlendPath := fmt.Sprintf("data/%v.metal.blend.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	metalBlend, _ := os.Create(metalBlendPath)
	rgbe.Encode(metalBlend, endUIndex-startUIndex, endVIndex-startVIndex, metalBlendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera      geom.Vec
		LookAt      geom.Vec
		Distance    float64
		FOV         float64
		EnvX        float64
		EnvY        float64
		EnvZ        float64
		Roughness   float64
		Red         float64
		Green       float64
		Blue        float64
		Red2        float64
		Green2      float64
		Blue2       float64
		ExtIOR      float64
		Scale       float64
		Aperture    float64
		G           float64
		MediumScale float64
		Weight      float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value="{{ .Aperture }}"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 1, 0"/>
        </transform>

        <sampler type="independent">
            <integer name="sample_count" value="256"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="2400"/>
            <integer name="height" value="2400"/>
            <rfilter type="box"/>
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
    <medium id="medium1" type="homogeneous">
        <float name="scale" value="{{ .MediumScale }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
    <bsdf type="blendbsdf">
        <texture type="bitmap" name="weight">
            <string name="filename" value="mitsuba.metal.blend.rgbe"/>
        </texture>
    <bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight }}"/>
            <bsdf type="roughdielectric">
				<float name="alpha" value="{{ .Roughness }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
            </bsdf>
         <bsdf type="twosided">
            <bsdf type="diffuse">
            </bsdf>
            </bsdf>
    </bsdf>
         <bsdf type="twosided">
            <bsdf type="conductor">
				<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
				<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
            </bsdf>
		 </bsdf>
    </bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="{{ .Scale }}"/>
            <translate x="0" y="0" z="0"/>
        </transform>
    <bsdf type="blendbsdf">
        <texture type="bitmap" name="weight">
            <string name="filename" value="mitsuba.metal.blend.rgbe"/>
        </texture>
            <bsdf type="roughdielectric">
				<float name="alpha" value="{{ .Roughness }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
            </bsdf>
         <bsdf type="twosided">
            <bsdf type="conductor">
				<rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
				<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
            </bsdf>
		 </bsdf>
    </bsdf>
    </shape>
</scene>
`)

	red, green, blue := hsb2rgb(sinft(24)/2+.5, .95, .95)
	red2, green2, blue2 := hsb2rgb(sinft(25)/2+.5, .95, .95)

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		fov,
		sinft(26) * 175,
		sinft(27) * 175,
		sinft(28) * 175,
		pow(10, sinft(29)-1),
		red,
		green,
		blue,
		red2,
		green2,
		blue2,
		1.5 + sinft(30)*.5,
		6 + sinft(31)*3,
		pow(10, sinft(32)-3),
		sinft(33) * .9,
		pow(10, sinft(34)+3),
		sinft(35)/2 + .5,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 1024, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	frameNumber = *frame
	renderSurfaces(*pixels, *maxSubdivisions, dt, *desiredTriangles)
}
