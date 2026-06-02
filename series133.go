//go:build ignore
// +build ignore

package main

import (
	"bufio"
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"os"
	"text/template"

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

func pushout(x, duty, degree float64) float64 {
	return spow(pow(x, duty)*2-1, degree)/2 + .5
}

func strength(n int, x float64) float64 {
	return pow(1.5, sin(float64(n)*x+float64(n)/10))
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
		Width:    0.16,
		Height:   0.09,
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
	factor := tan(s.FOV * .75 / 360 * pi)
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
	return cosVec.Scaled(r * cos(p*t)).Plus(sinVec.Scaled(r * sin(p*t)).Plus(center))
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5-.45*cos(2*globalT), 2, 3, circle)
}

func middleKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5-.45*cos(3*globalT), 2, 3, outerKnot)
}

func innerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5-.45*cos(5*globalT), 2, 3, middleKnot)
}

func lastKnot(t float64) geom.Vec {
	return torusKnot(t, 1, .5-.45*cos(7*globalT), 2, 3, innerKnot)
}

func cameraPath(t float64) geom.Vec {
	return circle(t + pi/4).Scaled(3.75 + cos(t)*.75).Plus(geom.Vec{0, 0, 4})
}

func focusPath(t float64) geom.Vec {
	return geom.Vec{0, 0, .5}
}

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v + delta).Minus(path(v - delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * cos(u)).Plus(sinVec.Scaled(r * sin(u)).Plus(center))
}

func sphere(u, v, t float64) geom.Vec {
	return geom.Vec{
		sin(v/2.0) * cos(u),
		sin(v/2.0) * sin(u),
		cos(v / 2.0),
	}
}

func shapeTexture(f, a, t float64, loc geom.Vec) float64 {
	loc = loc.Scaled(f * 2 * pi / (.7 + .2*sin(2*t)))
	s2 := strength(1, t)
	s3 := strength(2, t)
	s5 := strength(3, t)
	s7 := strength(4, t)
	s11 := strength(5, t)
	s1 := strength(6, t)
	return sin(
		a*s2*sin(a*s5*loc.Y) +
			a*s2*sin(a*s5*loc.X) +
			a*s3*sin(a*s7*2*loc.X+a*s11*sin(a*s1*3*loc.Y)) +
			a*s3*sin(a*s7*2*loc.X+a*s11*sin(a*s1*3*loc.Z)) +
			a*s3*sin(a*s7*2*loc.Y+a*s11*sin(a*s1*3*loc.Z)) +
			a*s3*sin(a*s7*2*loc.Z+a*s11*sin(a*s1*3*loc.X)) +
			a*s3*sin(a*s7*2*loc.X-a*s11*sin(a*s1*3*loc.Z)) +
			a*s3*sin(a*s7*2*loc.Y-a*s11*sin(a*s1*3*loc.X)) +
			a*s3*sin(a*s7*2*loc.Z-a*s11*sin(a*s1*3*loc.X)) +
			a*s3*sin(a*s7*2*loc.Z-a*s11*sin(a*s1*3*loc.Y)))
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
	a := .75
	minT := pi * .95
	maxT := pi * 1.05
	t = minT + t/2/pi*(maxT-minT)
	return sin(
		10*u + a*strength2(.1+2*t)*sin(
			2*u+a*strength2(.2+3*t)*sin(3*u)) + a*strength2(.3+5*t)*sin(
			2*v+a*strength2(.4+7*t)*sin(-v)) + a*strength2(.5+11*t)*sin(
			5*u-v) + a*strength2(.6+13*t)*sin(7*u-v) + a*strength2(.7+17*t)*sin(v-3*u))
}

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	a := sin(2*globalT) * .333
	b := sin(3*globalT) * .333
	return geom.Vec{
		sin(float64(xN)*t + a*sin(2*float64(xN)*t) + b*sin(float64(xN)*t)),
		sin(float64(yN)*t + a*sin(2*float64(yN)*t) + b*sin(float64(yN)*t)),
		cos(float64(zN)*t - a*sin(2*float64(zN)*t) - b*sin(float64(zN)*t))}
}

func knot(t float64) geom.Vec {
	return lissajousKnot(t, 5, 6, 7)
}

func uv2xyz(u, v, t float64) geom.Vec {
	funcs := [4]func(t float64) geom.Vec{outerKnot, middleKnot, innerKnot, lastKnot}
	a := pow(5, sin(2*t))
	f := floor(pow(10, sin(3*t)*.5+1.5))
	texture := sin(t + f*v + a*sin(5*t)*5*sin(v+a*sin(7*t)*5*sin(f*v)) + a*sin(11*t)*sin(10*v) + a*sin(13*t)*sin(u+v))
	texture = spow(pow(texture/2+.5, pow(4, sin(17*t)))*2-1, pow(4, sin(19*t)))
	maxR := pow(2, cos(23*t)-2)
	r := texture*maxR/2 + maxR/2
	return pathWrapper(u, v, r, funcs[frameNumber%4])
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

func renderSurfaces(pixels int, maxSubdivisions int, dt float64, desiredTriangles int) {
	t := float64(frameNumber) * dt
	globalT = t
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.FOV = 60
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
	focusPoint = geom.Vec{0, 0, -minZ * (sin(29*t) + 1)}
	distance = cameraLoc.Minus(closestPoint.Plus(focusPoint)).Len()
	//distance = cameraLoc.Len()
	fmt.Printf("minDistance: %v, maxDistance: %v, distance: %v, len: %v, maxX: %v, maxY: %v, maxZ: %v, minZ: %v\n", minDistance, maxDistance, distance, cameraLoc.Len(), maxX, maxY, maxZ, minZ)
	ratio := totalWidth / totalHeight
	fmt.Println(totalWidth, totalHeight, ratio)
	fmt.Println(numTriangles)
	nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 500)
	nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 500)
	/*
		for nV > 65000 {
			ratio = ratio * 1.1
			nU = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)*ratio) * 100)
			nV = int(sqrt(float64(desiredTriangles)/float64(numTriangles*2)/ratio) * 100)
		}
	*/
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
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
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
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera   geom.Vec
		LookAt   geom.Vec
		Distance float64
		FOV      float64
		Red      float64
		Green    float64
		Blue     float64
		Red2     float64
		Green2   float64
		Blue2    float64
		G        float64
		Scale    float64
		MinZ     float64
		IntIOR   float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="thinlens" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="focus_distance" value="{{ .Distance }}"/>
        <float name="aperture_radius" value=".01"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="0, 0, 1"/>
        </transform>

        <sampler type="multijitter">
            <integer name="sample_count" value="1024"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="2400"/>
            <integer name="height" value="2400"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="kloppenheim_06_4k.hdr"/>
         <transform name="to_world">
            <rotate value="1, 0, 0" angle="90"/>
        </transform>
   </emitter>
        <integrator type="volpathmis">
            <integer name="max_depth" value="25"/>
        </integrator>
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
		<bsdf type="dielectric">
			<float name="int_ior" value="{{ .IntIOR }}"/>
			<float name="ext_ior" value="1.1"/>
    	</bsdf>
		<medium type="homogeneous" name="interior">
			<float name="scale" value="{{ .Scale }}"/>
			<rgb name="sigma_t" value="{{ .Red }}, {{ .Green }}, {{ .Blue }}"/>
			<rgb name="albedo" value="{{ .Red2 }}, {{ .Green2 }}, {{ .Blue2 }}"/>
			<phase type="hg">
				<float name="g" value="{{ .G }}"/>
			</phase>
		</medium>
        <transform name="to_world">
            <translate x="0" y="0" z="{{ .MinZ }}"/>
        </transform>
    </shape>
	<shape type="rectangle">
        <transform name="to_world">
            <scale value="40"/>
            <translate x="0" y="0" z="0"/>
        </transform>
         <bsdf type="roughplastic">
						<float name="alpha" value=".1"/>
				<rgb name="diffuse_reflectance" value="0, 0, 0"/>
		 </bsdf>
	</shape>
</scene>
`)

	red, green, blue := hsb2rgb(2*t/(2*pi), sin(31*t)*.2+.21, sin(37*t)/3+.5)
	red2, green2, blue2 := hsb2rgb(3*t/(2*pi), sin(41*t)*.2+.21, sin(43*t)/3+.5)

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		c.FOV,
		red,
		green,
		blue,
		red2,
		green2,
		blue2,
		-sin(47*t) * .95,
		pow(10, 2+cos(53*t)),
		-minZ,
		sin(59*t)*.01 + 1.1,
	})
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
	maxFrames := flag.Int("maxframes", 1000, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	frameNumber = *frame
	renderSurfaces(*pixels, *maxSubdivisions, dt, *desiredTriangles)
}
