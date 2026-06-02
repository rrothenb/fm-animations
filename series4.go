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
	return sign(x) * pow(abs(x), y)
}

func strength(x float64) float64 {
	return sin(x)*.9 + 1
}

func subtexture2(u, v, t float64) float64 {
	a := sin(7*u+strength(4*t)*subtexture3(u, v, t)) + sin(5*v+strength(3*t)*subtexture3(u, v, t))
	b := sin(11*u + 13*v + strength(4*t)*subtexture3(u, v, t))
	weight := sin(3*t+.5*sin(6*t))/2 + .5
	return weight*a + (1-weight)*b
}

func subtexture3(u, v, t float64) float64 {
	return sin(2*u + 3*v)
}

func texture(u, v, t float64) float64 {
	return sin(3*u-3*v+strength(5*t)*subtexture2(3*u, 3*v, t)) * sin(3*u+3*v+strength(7*t)*subtexture2(3*v, 3*u, t))
}

func radius(u, v, t float64) float64 {
	return 1.0 + (sin(2*t)*.2+.3)*texture(u, v, t)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2 - 1
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
	if projectedPoint.Z > 0.0 {
		return true
	}
	return false
	if projectedPoint.Z < -s.position.Len() {
		return true
	}
	if projectedPoint.X < projectedPoint.Z/2 || projectedPoint.X > -projectedPoint.Z/2 {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z/2 || projectedPoint.Y > -projectedPoint.Z/2 {
		return true
	}
	return false
}

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	return geom.Vec{
		u/pi - 1,
		v*2/pi - 1,
		a,
	}
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, n int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	right := uv2xyz(index2radians(float64(uIndex)+.1, n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
	up := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)+.1, n), t, radius).Scaled(.075)
	down := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex)-.1, n), t, radius).Scaled(.075)
	normal, _ := left.Minus(right).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	cameraLoc := geom.Vec{0, 0, 1}.Scaled(.14)
	unitCameraLoc, _ := cameraLoc.Unit()
	focusPoint := unitCameraLoc.Scaled(.075)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	n := int(float64(pixels) / distance * 3)
	if n > maxSubdivisions {
		n = maxSubdivisions
	}
	if desiredTriangles > 0 {
		numTriangles := 0
		for uIndex := 0; uIndex <= 500; uIndex++ {
			for vIndex := 0; vIndex <= 500; vIndex++ {
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius).Scaled(.075)
				if !c.invisible(vertex) {
					numTriangles++
				}
			}
		}
		fmt.Println(numTriangles)
		n = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)) * 500)
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, n: %v\n", cameraLoc.Len(), distance, n)
	vertexIndicies := make([][]int32, n+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= n; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, n+1)
		for vIndex := 0; vIndex <= n; vIndex++ {
			vertex := uv2xyz(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t, radius).Scaled(.075)
			if c.invisible(vertex) {
				vertexIndicies[uIndex][vIndex] = -1
				continue
			}
			normal := uvIndexToNormal(uIndex, vIndex, n, t)
			vertexIndicies[uIndex][vIndex] = int32(numVerticies)
			numVerticies++
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.X))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Y))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Z))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex), n)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex), n)/pi/2))
		}
	}
	envmapArray := []float32{}
	roughnessArray := []float32{}
	blendArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < n; vIndex++ {
		for uIndex := 0; uIndex < n; uIndex++ {
			u := float64(uIndex) / float64(n) * 2 * pi
			v := float64(vIndex) / float64(n) * pi
			envmapValue := float32(1 - pow(1-pow(texture(u, v, t), 2), pow(1-v/pi, 3)*10))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
			roughnessValue := float32(0.0) // float32(pow(spow(subtexture1(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t), .5)*.5+.5, 2))*.5+.1
			roughnessArray = append(roughnessArray, roughnessValue, roughnessValue, roughnessValue)
			blendValue := float32(pow(.5-spow(texture(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t), .5)*.5, pow(10, sin(t))))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)

			topRight := vertexIndicies[uIndex][vIndex]
			topLeft := vertexIndicies[uIndex+1][vIndex]
			botRight := vertexIndicies[uIndex][vIndex+1]
			botLeft := vertexIndicies[uIndex+1][vIndex+1]
			if topRight == -1 || topLeft == -1 || botRight == -1 || botLeft == -1 {
				continue
			}
			if vIndex != 0 {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
				binary.Write(PlyDataBuffered, binary.LittleEndian, topLeft)
				numFaces++
			}
			if vIndex != n-1 {
				binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
				binary.Write(PlyDataBuffered, binary.LittleEndian, topRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botRight)
				binary.Write(PlyDataBuffered, binary.LittleEndian, botLeft)
				numFaces++
			}
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
	rgbe.Encode(envmap, n, n, envmapArray)
	roughness, _ := os.Create(roughnessPath)
	rgbe.Encode(roughness, n, n, roughnessArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, n, n, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Loc     geom.Vec
		Weight1 int
		Weight2 int
		Weight3 int
		Weight4 int
		Rough1  float64
		Rough2  float64
		Red     float64
		Green   float64
		Blue    float64
		Red2    float64
		Green2  float64
		Blue2   float64
		IntIOR  float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="orthographic" id="Camera-camera">
        <transform name="to_world">
			<scale x=".05" y=".05"/>
            <lookat target="0, 0, 0" origin="{{ .Loc.X }}, {{ .Loc.Y }}, {{ .Loc.Z }}" up="0, 1, 0"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="144"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="800"/>
            <integer name="height" value="800"/>
            <rfilter type="gaussian"/>
        </film>
    </sensor>
    <integrator type="path" />
    <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
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
                <float name="alpha" value=".01"/>
                <spectrum name="eta" filename="spd/15.spd"/>
                <spectrum name="k" filename="spd/11.spd"/>
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
                <float name="alpha" value=".01"/>
                <spectrum name="eta" filename="spd/15i.spd"/>
                <spectrum name="k" filename="spd/11i.spd"/>
						</bsdf>
					</bsdf>
				</bsdf>
			</bsdf>
		</bsdf>
</shape>

    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="2"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="90"/>
        </transform>
    </emitter>
</scene>
`)
	fmt.Println(cameraLoc)
	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		frameNumber % 2,
		(frameNumber / 2) % 2,
		(frameNumber / 4) % 2,
		(frameNumber / 8) % 2,
		pow(10, sin(5*t)-2),
		pow(10, cos(7*t)-2),
		cos(2*t)/2 + .5,
		cos(3*t)/2 + .5,
		cos(5*t)/2 + .5,
		.5 - cos(13*t)/2,
		.5 - cos(11*t)/2,
		.5 - cos(7*t)/2,
		1.5 + sin(17*t)*.25,
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
