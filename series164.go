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
var tan = math.Tan
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var min = math.Min
var max = math.Max
var floor = math.Floor

var tGlobal = 0.0
var frameNumber = 0

var baseframe = 596

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

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, a)*2-1, b)
}

func strength(x float64) float64 {
	return sin(x)*.9 + 1
}

func subtexture2(u, v, t float64) float64 {
	return sin(11*u + 13*v + strength(prime(16)*t)*subtexture3(u, v, t))
}

func subtexture3(u, v, t float64) float64 {
	return sin(2*u + 3*v)
}

func texture(u, v, t float64) float64 {
	return (sin(u+strength(prime(15)*t)*subtexture2(u, v, t)) +
		sin(v+strength(prime(14)*t)*subtexture2(0, v, t)) +
		sin(u-v+strength(prime(13)*t)*subtexture2(u, v, t)) +
		sin(u+v+strength(prime(12)*t)*subtexture2(u, v, t))) / 4
}

func radius(u, v, t float64) float64 {
	return 1.0 - strength(t)*pow(texture(u, v, t), 2)
}

func pushdown(x, n float64) float64 {
	return pow(x/2+.5, n)*2 - 1
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
	if s.FOV > 60 {
		return false
	}
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	factor := tan(s.FOV * 3 / 360 * pi)
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

func uv2xyz(u, v, t float64, radius func(u, v, t float64) float64) geom.Vec {
	a := radius(u, v, t)
	R := .6
	r := (.4 + cos(prime(11)*t)*.1) * a
	n := float64(baseframe%4) + 1
	t2 := float64(frameNumber) / 192 * 2 * pi
	a1 := sin(prime(10)*t+t2) * .5
	a2 := sin(prime(9)*t+t2) * .5
	a3 := sin(prime(8)*t+t2) * .5
	loc := geom.Vec{
		(R + r*cos(u-a1*sin(2*u))) * cos(v-a2*sin(2*v)),
		(R + r*cos(u-a1*sin(2*u))) * sin(v+a2*sin(2*v)),
		r * sin(u+a3*sin(2*u)),
	}
	return geom.Vec{
		loc.X*cos((.5-sin(v)/2)*pi*n) + loc.Z*sin((.5-sin(v)/2)*pi*n),
		(n/2 + 1) * loc.Y,
		loc.X*sin((.5-sin(v)/2)*pi*n) - loc.Z*cos((.5-sin(v)/2)*pi*n),
	}
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

func renderSurfaces(pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	width := int(aspectRatio * float64(height))
	t := float64(baseframe) * dt
	tGlobal = t
	envSize := int(pow(float64(desiredTriangles), .5))
	length := (float64(baseframe%4)+1)/2 + 1
	cameraDir, _ := geom.Vec{sin(t), cos(t), sin(prime(7)*t)*.25 + 2}.Unit()
	cameraLoc := cameraDir.Scaled(.5 + sin(prime(17)*t)*.5).By(geom.Vec{1, length, 1})
	focusPoint := geom.Vec{0, 0, 0}
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FStop = 64
	c.FOV = 60 + sin(prime(18)*t)*30
	c.AspectRatio = aspectRatio
	// distance := cameraLoc.Minus(focusPoint).Len() - c.Lens
	distance := focusPoint.Minus(geom.Vec{0, 0, .035}).Len()
	nU := int(float64(pixels) / distance * 3)
	if nU > maxSubdivisions {
		nU = maxSubdivisions
	}
	nV := nU
	if desiredTriangles > 0 {
		numTriangles := 0
		totalWidth := 0.0
		totalHeight := 0.0
		for uIndex := 0; uIndex <= 500; uIndex++ {
			for vIndex := 0; vIndex <= 500; vIndex++ {
				vertex := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500), t, radius)
				if !c.invisible(vertex) {
					numTriangles++
					vertexLeft := uv2xyz(index2radians(float64(uIndex-1), 500), index2radians(float64(vIndex), 500), t, radius)
					vertexBelow := uv2xyz(index2radians(float64(uIndex), 500), index2radians(float64(vIndex-1), 500), t, radius)
					totalWidth += vertex.Minus(vertexLeft).Len()
					totalHeight += vertex.Minus(vertexBelow).Len()
				}
			}
		}
		fmt.Println(numTriangles)
		ratio := totalWidth / totalHeight
		nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
		nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	}
	vertexIndicies := make([][]int32, nU+1)
	numVerticies := 0
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	PlyDataBuffered := bufio.NewWriter(plyData)
	for uIndex := 0; uIndex <= nU; uIndex++ {
		vertexIndicies[uIndex] = make([]int32, nV+1)
		for vIndex := 0; vIndex <= nV; vIndex++ {
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
			roughnessValue := float32(0.0) // float32(pow(spow(subtexture1(index2radians(float64(uIndex), n), index2radians(float64(vIndex), n), t), .5)*.5+.5, 2))*.5+.1
			blendValue := float32(pow(spow(texture(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t), .1*strength(prime(5)*t))*.5+.5, 10*strength(prime(6)*t)))
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
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			v := float64(vIndex) / float64(envSize) * pi
			u := float64(uIndex) / float64(envSize) * pi
			envmapValue := float32((1 - pow(1-pow(texture(u, v, t), 2), pow(1-v/pi, 3)*10)))
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
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	roughness, _ := os.Create(roughnessPath)
	rgbe.Encode(roughness, nU, nV, roughnessArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, nU, nV, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Loc      geom.Vec
		Distance float64
		Height   int
		Width    int
		Samples  int
		Glass    float64
		Red1     float64
		Green1   float64
		Blue1    float64
		Red2     float64
		Green2   float64
		Blue2    float64
		FOV      float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="smaller"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat target="0, 0, 0" origin="{{ .Loc.X }}, {{ .Loc.Y }}, {{ .Loc.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="{{ .Samples }}"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="{{ .Width }}"/>
            <integer name="height" value="{{ .Height }}"/>
            <rfilter type="lanczos"/>
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
				<float name="weight" value="{{ .Glass }}"/>
				<bsdf type="twosided">
					<bsdf type="conductor">
                        <rgb name="k" value="{{ .Red1 }}, {{ .Green1 }}, {{ .Blue1 }}"/>
                        <rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
					</bsdf>
				</bsdf>
				<bsdf type="dielectric">
				</bsdf>
			</bsdf>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Glass }}"/>
				<bsdf type="dielectric">
				</bsdf>
				<bsdf type="twosided">
					<bsdf type="conductor">
                        <rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
                        <rgb name="eta" value="{{ .Red1 }}, {{ .Green1 }}, {{ .Blue1 }}"/>
					</bsdf>
				</bsdf>
			</bsdf>
        </bsdf>
    </shape>

    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value=".5"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="45"/>
        </transform>
    </emitter>
    <shape type="rectangle">
        <bsdf type="twosided">
            <bsdf type="diffuse">
                <rgb name="reflectance" value="0, 0, 0"/>
            </bsdf>
        </bsdf>
        <transform name="to_world">
            <scale value="100000000000"/>
            <rotate value="0, 1, 0" angle="0"/>
            <translate x="0" y="0" z="-1.25"/>
        </transform>
    </shape>
</scene>
`)
	fmt.Println(cameraLoc)
	red1 := .5 - cos(prime(4)*t)/2
	green1 := sin(prime(3)*t)/2 + .5
	blue1 := cos(prime(2)*t)/2 + .5
	red2 := red1
	green2 := green1
	blue2 := blue1

	if baseframe%4 == 0 {
		red2 = 1 - red2
		green2 = 1 - green2
		blue2 = 1 - blue2
	} else if baseframe%4 == 1 {
		red2 = 1 - red2
		green2 = 1 - green2
	} else if baseframe%4 == 2 {
		green2 = 1 - green2
		blue2 = 1 - blue2
	} else if baseframe%4 == 3 {
		blue2 = 1 - blue2
		red2 = 1 - red2
	}

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		distance,
		height,
		width,
		samples,
		cos(prime(1)*t)/2 + .5,
		red1,
		green1,
		blue1,
		red2,
		green2,
		blue2,
		c.FOV,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 1000, "Max frames")
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
