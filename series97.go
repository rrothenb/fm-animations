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

func pushout(x, duty, degree float64) float64 {
	return spow(pow(x, duty)*2-1, degree)/2 + .5
}

func strength(n int, x float64) float64 {
	return pow(2, sin(pow(float64(n), .5)*(x+float64(n)/3)))
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
	return false
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	//fmt.Printf("\npoint: %#v\nprojectedPoint: %#v\ncameraSpaceTransform: %#v\n", point, projectedPoint, cameraSpaceTransform)
	factor := .35
	aspectRatio := s.Width / s.Height
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
	return torusKnot(t, 1, .25, 2, 3, circle)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .15, 3, 2, outerKnot)
}

func cameraPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 2}
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
	return cosVec.Scaled(r * cos(u-.5*sin(u))).Plus(sinVec.Scaled(r * sin(u+.5*sin(u)))).Plus(center)
}

func knot(t float64) geom.Vec {
	return unitLissajousKnot(t, 19, 20, 21)
}

func sphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v / 2.0),
	}
}

func shapeTexture(f, a, t float64, loc geom.Vec) float64 {
	loc = loc.Scaled(f * 2 * pi)
	loc.X = abs(loc.X)
	loc.Y = abs(loc.Y)
	loc.Z = abs(loc.Z)
	return sin(
		a*strength(7, t)*sin(a*strength(23, t)*loc.Z) +
			a*strength(7, t)*sin(a*strength(23, t)*loc.Y) +
			a*strength(7, t)*sin(a*strength(23, t)*loc.X) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y+a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z+a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z+a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.X-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y-a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Y-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Z)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z-a*strength(29, t)*sin(a*strength(31, t)*3*loc.X)) +
			a*strength(11, t)*sin(a*strength(19, t)*2*loc.Z-a*strength(29, t)*sin(a*strength(31, t)*3*loc.Y)))
}

func cube(u, v, t float64) geom.Vec {
	a := sin(t)*.2 + .25
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-a*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+a*sin(2*u)),
		cos(v/2.0 - a*sin(v)),
	}
}

func shape(u, v, t float64) geom.Vec {
	return cube(u, v, t)
}

func strength2(x float64) float64 {
	return pow(2, sin(x)*2)
}

func texture2(u, v, t float64) float64 {
	a := .5
	minT := pi * .975
	maxT := pi * 1.025
	t = minT + t/2/pi*(maxT-minT)
	return sin(
		10*u + a*strength2(.1+2*t)*sin(
			2*u+a*strength2(.2+3*t)*sin(3*u)) + a*strength2(.3+5*t)*sin(
			2*v+a*strength2(.4+7*t)*sin(-v)) + a*strength2(.5+11*t)*sin(
			5*u-v) + a*strength2(.6+13*t)*sin(7*u-v) + a*strength2(.7+17*t)*sin(v-3*u))
}

func fabricPath(t float64) geom.Vec {
	return geom.Vec{sin(599 * (t - pi)), sin(601 * (t - pi)), sin(2 * (t - pi))}
}

func uv2xyz(u, v, t float64) geom.Vec {
	loc := pathWrapper(u, v, .003, fabricPath).By(geom.Vec{1, 1, .1})
	displacement := geom.Vec{0, 0, .1 * pow(spow(texture2((loc.X-loc.Y)*pi, (loc.X+loc.Y)*pi, t), pow(1.5, sin(2*t)))/2+.5, pow(1.5, cos(3*t)))}
	return loc.Plus(displacement)
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
	//distance = cameraLoc.Minus(closestPoint).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v, minZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ, minZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
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
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			loc := shape(u, v, t)
			// blendValue := float32((.5-cos(v/2-.7*sin(v))/2)*(.01*pow(spow(shapeTexture(3, 2, t, loc), pow(strength(5, t), 4))/2+.5, pow(strength(7, t), 4))))
			blendValue := float32(pow(spow(shapeTexture(2, 1, t, loc), 10)/2+.5, 10))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
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
			envmapValue := float32(pow(sin(u/2), 2) * pow(sin(v), 2))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
			/*
				u := float64(uIndex) / float64(envSize) * 2 * pi
				v := float64(vIndex) / float64(envSize) * 2 * pi
				envmapValue := pow(spow(shapeTexture(1, .75-cos(41*t)*.5, t, sphere(u, v, t)), pow(10, sin(23*t)))/2+.5, pow(10, sin(29*t)))
				envmapArray = append(
					envmapArray,
					float32(pow(envmapValue, pow(2, cos(11*t)))),
					float32(pow(envmapValue, pow(2, sin(7*t)))),
					float32(pow(envmapValue, pow(2, -cos(5*t)))))

			*/
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
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	sensorFile, _ := os.Create("sensor.xml")

	type instance struct {
		Angle float64
		Loc   geom.Vec
		Scale float64
	}
	type sensor struct {
		Camera    geom.Vec
		LookAt    geom.Vec
		Distance  float64
		FogRadius float64
		Angle     float64
		MinZ      float64
		IntIOR    float64
		ExtIOR    float64
		Roughness float64
		FOV       float64
		Scale     float64
		SigmaT    float64
		Albedo    float64
		G         float64
		Instances []instance
		Red       float64
		Green     float64
		Blue      float64
		Red2      float64
		Green2    float64
		Blue2     float64
		EnvX      float64
		EnvY      float64
		EnvZ      float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value=".25"/>
        <float name="aperture_radius" value=".00000000001"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="1, 1, 0"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="25"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="500"/>
            <integer name="height" value="500"/>
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
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
        <ref id="medium1" name="interior"/>
			<bsdf type="null">
			</bsdf>
    </shape>

</scene>
`)
	angle := 90.0
	instances := []instance{}
	num := 5
	for x := -num; x <= num; x++ {
		for y := -num; y <= num; y++ {
			for z := -num; z <= num; z++ {
				loc := geom.Vec{float64(x), float64(y), float64(z)}
				angle := float64((x+y+z)%4) * 90
				instances = append(instances, instance{angle, loc, .4 + cos(19*t)*.09})
			}
		}
	}
	fmt.Println(len(instances))

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		angle,
		minZ,
		sin(7*t) + 2,
		cos(11*t) + 2,
		pow(10, sin(13*t)*2-3),
		40,
		pow(10, sin(19*t)*2+1),
		sin(23*t)/2 + .5,
		sin(29*t)/2 + .5,
		sin(31*t) * .99,
		instances,
		cos(2*t)/3 + .666,
		sin(3*t)/3 + .666,
		sin(5*t)/3 + .666,
		.666 - cos(2*t)/3,
		.666 - sin(3*t)/3,
		.666 - sin(5*t)/3,
		sin(19*t) * 175,
		sin(23*t) * 175,
		sin(29*t) * 175,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 32, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
