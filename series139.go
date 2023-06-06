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
var frameNumberGlobal = 0

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
	return sin(x)*1.25 + 1.25
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

func sin2(x float64) float64 {
	return spow(sin(x), pow(2, sin(7*tGlobal)-1))
}

func cos2(x float64) float64 {
	return spow(cos(x), pow(2, sin(7*tGlobal)-1))
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

func innerKnot(t float64) geom.Vec {
	p := [4]int{3, 2, 3, 2}
	q := [4]int{2, 3, 2, 3}
	i := (frameNumberGlobal / 64) % 4
	return torusKnot(t, 1, .5+cos(3*tGlobal)*.49, p[i], q[i], outerKnot)
}

func cameraPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 10}
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
	a := cos(3*tGlobal) * .5
	return cosVec.Scaled(r * cos(u-a*sin(2*u))).Plus(sinVec.Scaled(r * sin(u+a*sin(2*u)))).Plus(center)
}

func knot(t float64) geom.Vec {
	return unitLissajousKnot(t, 19, 20, 21)
}

func texture(u, v, t float64) float64 {
	v = v * 3
	a := pow(3, cos(31*t)-1)
	fU := floor(sin(29*t)*10 + 5)
	fV := floor(sin(23*t)*10 + 5)
	return sin(
		fU*u + fV*v +
			a*strength(1.7+19*t)*sin(2*u+a*strength(.7+7*t)*sin(3*u+a*strength(.3+3*t)*sin(5*u-7*v)*sin(11*u+3*v))) +
			a*strength(1.5+17*t)*sin(7*v+a*strength(.5+5*t)*sin(5*v+a*strength(.1+2*t)*sin(7*u-11*v)*sin(13*u+5*v))) +
			a*strength(1.3+13*t)*sin(5*u+7*v) +
			a*strength(1.1+11*t)*sin(17*u)*sin(19*v))
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
func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .51, 11, 3, circle)
}

func uv2xyz(u, v, t float64) geom.Vec {
	tuv := texture(pow(10, floor(sin(31*t)+2))*u, pow(10, floor(sin(29*t)+2))*v, t)
	tvu := texture(pow(10, floor(sin(19*t)+2))*v, pow(10, floor(sin(23*t)+2))*u, t)
	microTexture := blend(sin(17*t)/2+.5, tuv, tvu)
	blendValue := pow(spow(texture(u, v, t), pow(2, cos(13*t)))/2+.5, pow(2, cos(11*t)))
	a := pow(sin(u/2-.25*pi)/2+.5, pow(10, sin(7*t)+2))
	return pathWrapper(u, v, outerKnot(v).Len()/5.5-pow(10, sin(2*t)-2)*blendValue*a-.001*spow(sin(3*t), 4)*(1-(sin(5*t)/2+.5)*a)*microTexture, outerKnot)
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

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	tGlobal = t
	frameNumberGlobal = frameNumber
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := geom.Vec{0, 0, 0}
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FOV = 15
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
	maxX := -100.0
	maxY := -100.0
	maxZ := -100.0
	minX := 100.0
	minY := 100.0
	minZ := 100.0
	closestPoint := geom.Vec{0, 0, 0}
	farthestPoint := cameraLoc
	for uIndex := 1; uIndex <= 500; uIndex++ {
		for vIndex := 1; vIndex <= 500; vIndex++ {
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
				maxX = math.Max(maxX, vertex.X)
				maxY = math.Max(maxY, vertex.Y)
				maxZ = math.Max(maxZ, vertex.Z)
				minX = math.Min(minX, vertex.X)
				minY = math.Min(minY, vertex.Y)
				minZ = math.Min(minZ, vertex.Z)
				if cameraLoc.Minus(closestPoint).Len() > cameraLoc.Minus(vertex).Len() {
					closestPoint = vertex
				}
				if cameraLoc.Minus(farthestPoint).Len() < cameraLoc.Minus(vertex).Len() {
					farthestPoint = vertex
				}
			}
		}
	}
	midX := (minX + maxX) / 2
	midY := (minY + maxY) / 2
	midZ := (minZ + maxZ) / 2
	extent := math.Min(maxX-minX, math.Min(maxY-minY, maxZ-minZ))
	fmt.Printf("\nextent: %v\n", extent)
	center := geom.Vec{midX, midY, midZ}
	focusPoint = center
	distance = cameraLoc.Minus(closestPoint).Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ)
	ratio := totalWidth / totalHeight
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
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(uIndex), nU)/pi/2))
			binary.Write(PlyDataBuffered, binary.LittleEndian, float32(index2radians(float64(vIndex), nV)/pi/2))
		}
	}
	envmapArray := []float32{}
	numFaces := 0
	for vIndex := 0; vIndex < nV; vIndex++ {
		for uIndex := 0; uIndex < nU; uIndex++ {
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
			power := 2 * pow(4, -cos(3*t)/2+.5)
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
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	sensorFile, _ := os.Create("sensor.xml")

	type instance struct {
		Scale float64
		Loc   geom.Vec
	}
	type Sensor struct {
		Camera     geom.Vec
		LookAt     geom.Vec
		Distance   float64
		EnvX       float64
		EnvY       float64
		EnvZ       float64
		G          float64
		Scale      float64
		Red        float64
		Green      float64
		Blue       float64
		Red2       float64
		Green2     float64
		Blue2      float64
		InnerRings []instance
		OuterRings []instance
		FOV        float64
		Blend      float64
		Weight1    int
		Weight2    int
		Weight3    int
		Weight4    int
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".00000000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 1, 0"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="25"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="800"/>
            <integer name="height" value="800"/>
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
        <integrator type="volpathmis">
            <integer name="max_depth" value="16"/>
        </integrator>
    <medium id="medium1" type="homogeneous">
        <float name="scale" value="{{ .Scale }}"/>
        <rgb name="sigma_t" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
        <rgb name="albedo" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
    <medium id="medium2" type="homogeneous">
        <float name="scale" value="{{ .Scale }}"/>
        <rgb name="sigma_t" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
        <rgb name="albedo" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
 <shape type="shapegroup" id="innerRing">
    <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight1 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight2 }}"/>
				   <bsdf type="twosided">
						<bsdf type="diffuse">
						</bsdf>
					</bsdf>
				      <bsdf type="twosided">
						<bsdf type="conductor">
					<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
					<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
						</bsdf>
				     </bsdf>
				</bsdf>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight2 }}"/>
        <bsdf type="dielectric">
        </bsdf>
    	<bsdf type="blendbsdf">
		<float name="weight" value="{{ .Blend }}"/>
        <bsdf type="dielectric">
        </bsdf>
		   <bsdf type="twosided">
				<bsdf type="conductor">
					<rgb name="eta" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
					<rgb name="k" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
				</bsdf>
			</bsdf>
			</bsdf>
			</bsdf>
        </bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
 </shape>
 <shape type="shapegroup" id="outerRing">
    <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <transform name="to_world">
            <scale value="1"/>
            <translate x="0" y="0" z="0"/>
        </transform>
			<bsdf type="blendbsdf">
				<float name="weight" value="{{ .Weight3 }}"/>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight4 }}"/>
				   <bsdf type="twosided">
						<bsdf type="diffuse">
						</bsdf>
					</bsdf>
				      <bsdf type="twosided">
						<bsdf type="conductor">
					<rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
					<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
						</bsdf>
				     </bsdf>
				</bsdf>
				<bsdf type="blendbsdf">
					<float name="weight" value="{{ .Weight4 }}"/>
        <bsdf type="dielectric">
        </bsdf>
    	<bsdf type="blendbsdf">
		<float name="weight" value="{{ .Blend }}"/>
		   <bsdf type="twosided">
				<bsdf type="conductor">
					<rgb name="eta" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
					<rgb name="k" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
				</bsdf>
			</bsdf>
        <bsdf type="dielectric">
        </bsdf>
			</bsdf>
			</bsdf>
        </bsdf>
	<ref id="medium2" name="interior"/>
    </shape>
 </shape>
{{range .InnerRings }}<shape type="instance"><ref id="innerRing"/><transform name="to_world"><scale value="{{ .Scale }}"/><translate x="{{ .Loc.X }}" y="{{ .Loc.Y }}" z="{{ .Loc.Z }}"/></transform></shape>{{end}}
{{range .OuterRings }}<shape type="instance"><ref id="outerRing"/><transform name="to_world"><scale value="{{ .Scale }}"/><translate x="{{ .Loc.X }}" y="{{ .Loc.Y }}" z="{{ .Loc.Z }}"/></transform></shape>{{end}}
</scene>
`)
	colorDiff := 1 / float64(frameNumber%7+2)
	red, green, blue := hsb2rgb(sin(29*t)/2+.5, .95, .95)
	red2, green2, blue2 := hsb2rgb(sin(23*t)/2+.5+colorDiff, .95, .95)

	innerRings := []instance{}
	outerRings := []instance{}
	num := 0.0
	for x := -num; x <= num; x++ {
		for y := -num; y <= num; y++ {
			loc := geom.Vec{x, y, 0}
			innerRings = append(innerRings, instance{1, loc})
			innerRings = append(innerRings, instance{pow(5+cos(3*t)*.5, 6), loc})
			outerRings = append(outerRings, instance{5 + cos(3*t)*.5, loc})
		}
	}

	lowFOV := 45 + cos(3*t)*10
	highFOV := 135 + cos(3*t)*10
	fovWeight := .5 + cos(t)*.5
	fov := fovWeight*lowFOV + (1-fovWeight)*highFOV

	sensor := Sensor{
		cameraLoc,
		focusPoint,
		distance,
		sin(2*t) * 175,
		sin(3*t) * 175,
		sin(5*t) * 175,
		-cos(7*t) * .9,
		pow(4, cos(11*t)+2),
		red,
		green,
		blue,
		red2,
		green2,
		blue2,
		innerRings,
		outerRings,
		fov,
		spow(sin(t), .25)/2 + .5,
		frameNumber % 2,
		(frameNumber / 2) % 2,
		(frameNumber / 4) % 2,
		(frameNumber / 8) % 2,
	}

	sensorTemplate.Execute(sensorFile, sensor)
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

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 1024, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
