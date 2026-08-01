// go:build ignore

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

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[i-1])
}

type MeshType struct {
	NumVertices int
	NumFaces    int
}

var sin = math.Sin
var cos = math.Cos
var tan = math.Tan
var atan = math.Atan
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var min = math.Min
var max = math.Max
var yScale = 1.0
var scale = 0.0
var UL = geom.Vec{0, 0, 0}
var UR = geom.Vec{0, 0, 0}
var LL = geom.Vec{0, 0, 0}
var LR = geom.Vec{0, 0, 0}

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

func strength(n int, x float64) float64 {
	return pow(12, sin(float64(n)*(x+float64(n)/10)))
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

var zAxis = geom.Dir{0, 1, 0}

// NewSLR constructs a new camera with 35mm sensor full-frame / 50mm lens defaults.
func NewSLR2() *SLR2 {
	s := &SLR2{
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
	if projectedPoint.X < projectedPoint.Z*factor*s.AspectRatio || projectedPoint.X > -projectedPoint.Z*factor*s.AspectRatio {
		return true
	}
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
	if projectedPoint.Z < -1.0 {
		return true
	}
	return false
	if projectedPoint.Y < projectedPoint.Z*factor || projectedPoint.Y > -projectedPoint.Z*factor {
		return true
	}
	return false
}

func cameraPath(t, frustumHalfAngle float64) geom.Vec {
	margin := 6.0 * pi / 180
	capRadius := pi/2 - frustumHalfAngle - margin
	if capRadius < 0 {
		capRadius = 0
	}
	theta := capRadius * (sin(prime(29)*t)/2 + .5) // in [0, capRadius): never reaches the edge
	azimuth := prime(28) * t
	radius := 4.5 + 2.5*sin(prime(12)*t) // distance from origin, breathes in [2, 7]
	return geom.Vec{
		radius * sin(theta) * cos(azimuth),
		radius * sin(theta) * sin(azimuth),
		radius * cos(theta),
	}
}

func rectangle(u, v, t float64) geom.Vec {
	u = u / 2 / pi
	v = v / 2 / pi
	a0 := LL.X
	a1 := LR.X - a0
	a2 := UL.X - a0
	a3 := UR.X - a0 - a1 - a2
	b0 := LL.Y
	b1 := LR.Y - b0
	b2 := UL.Y - b0
	b3 := UR.Y - b0 - b1 - b2
	return geom.Vec{
		a0 + a1*u + a2*v + a3*u*v,
		b0 + b1*u + b2*v + b3*u*v,
		0,
	}
}

func blendValue(u, v, t float64) float64 {
	return shaper(texture1(u, v, t), pow(4, sin(prime(27)*t)), pow(2, cos(prime(26)*t))-1)/2 + .5
}

func texture(a, u, v, t float64) float64 {
	uPortion := sin(prime(25)*t)*.2 + .3
	vPortion := sin(prime(24)*t)*.2 + .3
	uOffset := (sin(prime(23)*t)/2 + .5) * (1 - uPortion)
	vOffset := (sin(prime(22)*t)/2 + .5) * (1 - vPortion)
	u = u*uPortion + 2*pi*uOffset
	v = v*vPortion + 2*pi*vOffset
	return sin(
		a*strength(9, t)*sin(11*u+a*strength(16, t)*sin(u))*sin(17*v+a*strength(17, t)*16*v) +
			a*strength(10, t)*sin(12*u+a*strength(15, t)*sin(v+a*strength(19, t)*sin(2*u-3*v))) +
			a*strength(11, t)*sin(13*v+a*strength(14, t)*sin(u+a*strength(20, t)*sin(3*u+2*v))) +
			a*strength(12, t)*sin(14*u+15*v+a*strength(13, t)*sin(u+v+a*strength(18, t)*sin(u-v))))

}

func texture1(u, v, t float64) float64 {
	a := pow(12, sin(prime(21)*t))
	return texture(a, u, v, t)
}

func texture2(u, v, t float64) float64 {
	a := pow(12, cos(prime(20)*t))
	return texture(a, u, v, t)
}

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, a)*2-1, b)
}

func shape(u, v, t float64) geom.Vec {
	a := spow(u/2/pi*(1-u/2/pi)*v/2/pi*(1-v/2/pi), .1)
	return rectangle(u, v, t).Plus(geom.Vec{
		0,
		0,
		a*scale*.1*cos(prime(17)*t)*(shaper(texture1(u, v, t), pow(2, sin(prime(18)*t)), pow(3, sin(prime(19)*t)-1))-1) +
			a*scale*.1*sin(prime(16)*t)*(shaper(texture2(u, v, t), pow(2, cos(prime(15)*t)), pow(3, cos(prime(14)*t)-1))-1),
	})
}

func uv2xyz(u, v, t float64) geom.Vec {
	return shape(u, v, t)
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t)
	right := uv2xyz(index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t)
	up := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t)
	down := uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t)
	// right.Minus(left) (not left.Minus(right)) so the shading normal keeps the
	// correct outward sense now that rectangle() no longer mirrors world-X.
	normal, _ := right.Minus(left).Cross(up.Minus(down)).Unit()
	return &normal
}

func writeVertex(PlyDataBuffered *bufio.Writer, vertex geom.Vec, normal geom.Dir, u, v float64) {
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.X))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Y))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(vertex.Z))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.X))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Y))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(normal.Z))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(u/2/pi))
	binary.Write(PlyDataBuffered, binary.LittleEndian, float32(v/2/pi))
}

func writeFace(PlyDataBuffered *bufio.Writer, a, b, c int32) {
	// Winding is a, c, b (reversed) so the geometric normal matches the flipped
	// shading normal after rectangle() stopped mirroring world-X.
	binary.Write(PlyDataBuffered, binary.LittleEndian, byte(3))
	binary.Write(PlyDataBuffered, binary.LittleEndian, a)
	binary.Write(PlyDataBuffered, binary.LittleEndian, c)
	binary.Write(PlyDataBuffered, binary.LittleEndian, b)
}

// emitFace writes a triangle with whichever winding makes its geometric normal
// agree with outward, so the body's faces can be listed in any order without
// having to track the sign conventions of the camera transform by hand.
func emitFace(PlyDataBuffered *bufio.Writer, ia, ib, ic int32, pa, pb, pc, outward geom.Vec) {
	if pb.Minus(pa).Cross(pc.Minus(pa)).Dot(outward) >= 0 {
		writeFace(PlyDataBuffered, ia, ic, ib)
	} else {
		writeFace(PlyDataBuffered, ia, ib, ic)
	}
}

func renderSurfaces(frameNumber int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	width := int(aspectRatio * float64(height))
	t := float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))
	focusPoint := geom.Vec{0, 0, 0}
	// Wide-angle field of view, swept by a prime like the other animated
	// parameters. fovHoriz is the horizontal FOV in degrees; the textured
	// rectangle below is *derived* from it (the rectangle is the frustum's z=0
	// slice), so widening the FOV genuinely widens the perspective instead of
	// just scaling the texture down to fit a fixed cone. Range here is ~30°..90°;
	// adjust the 60/30 to taste.
	fovHoriz := 60.0 + 30.0*sin(prime(30)*t)
	halfTanH := tan(fovHoriz / 2 * pi / 180)
	// Half-angle from the view axis to a frustum corner (the extreme ray). The
	// camera path uses this to stay far enough above the plane that every corner
	// still lands on it.
	tanV := halfTanH / aspectRatio
	frustumHalfAngle := atan(sqrt(halfTanH*halfTanH + tanV*tanV))
	cameraLoc := cameraPath(t, frustumHalfAngle)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.AspectRatio = aspectRatio
	// Frustum corner rays in camera space at unit depth. The vertical extent is
	// driven by the film aspect ratio (not a hard-coded 16:9), so the textured
	// rectangle matches the rendered frame exactly regardless of -aspectratio.
	UL = c.trans.MultPoint(geom.Vec{-halfTanH, halfTanH / aspectRatio, -1})
	UR = c.trans.MultPoint(geom.Vec{halfTanH, halfTanH / aspectRatio, -1})
	LL = c.trans.MultPoint(geom.Vec{-halfTanH, -halfTanH / aspectRatio, -1})
	LR = c.trans.MultPoint(geom.Vec{halfTanH, -halfTanH / aspectRatio, -1})
	// Mitsuba measures fov along the larger film axis (fov_axis="larger"): that is
	// the horizontal axis when the film is landscape, the vertical when portrait.
	fov := fovHoriz
	if aspectRatio < 1 {
		fov = 2 * atan(halfTanH/aspectRatio) * 180 / pi
	}
	c.FOV = fov
	ray := UL.Minus(cameraLoc)
	UL = ray.Scaled(cameraLoc.Z / (cameraLoc.Z - UL.Z)).Plus(cameraLoc)
	ray = UR.Minus(cameraLoc)
	UR = ray.Scaled(cameraLoc.Z / (cameraLoc.Z - UR.Z)).Plus(cameraLoc)
	ray = LL.Minus(cameraLoc)
	LL = ray.Scaled(cameraLoc.Z / (cameraLoc.Z - LL.Z)).Plus(cameraLoc)
	ray = LR.Minus(cameraLoc)
	LR = ray.Scaled(cameraLoc.Z / (cameraLoc.Z - LR.Z)).Plus(cameraLoc)
	scale = pow(UL.Minus(UR).Len()*LR.Minus(LL).Len(), .5)
	distance := cameraLoc.Minus(focusPoint).Len()
	fmt.Printf("\nyScale: %v\nUL: %v\nUR: %v\nLL: %v\nLR: %v\ncameraLoc: %v\nfocusPoint: %v\ndistance: %v\nt: %#v\nscale: %v\n",
		yScale, UL, UR, LL, LR, cameraLoc, focusPoint, distance, t, scale)
	fmt.Printf("fovHoriz: %v, mitsuba fov: %v, aspectRatio: %v\n", fovHoriz, fov, aspectRatio)

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
	farthestPoint := cameraLoc
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
				if cameraLoc.Minus(farthestPoint).Len() < cameraLoc.Minus(vertex).Len() {
					farthestPoint = vertex
				}
			}
		}
	}
	//cameraLoc = cameraLoc.Scaled(math.Max(maxX, math.Max(maxY, maxZ))*8)
	//boundingSpheroid := surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{maxX*.95, maxY*.95, maxZ*.95})
	// dir, _ := focusPoint.Minus(cameraLoc).Unit()
	// _, distance = surface.UnitSphere(material.Mirror(1)).Scale(geom.Vec{.075, .075, .075}).Intersect(geom.NewRay(cameraLoc, dir), 10.0)
	distanceWeight := cos(prime(13)*t)/5 + .5
	distance = distanceWeight*cameraLoc.Minus(closestPoint).Len() + (1-distanceWeight)*cameraLoc.Minus(farthestPoint).Len()
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
			writeVertex(PlyDataBuffered, vertex, *normal, index2radians(float64(uIndex-startUIndex), endUIndex-startUIndex), index2radians(float64(vIndex-startVIndex), endVIndex-startVIndex))
		}
	}
	// --- Cube body ---------------------------------------------------------
	// The patch is inset into the front face of a cube. Coplanar with the patch,
	// a flat frame runs from the patch boundary out to the front face's edge;
	// from there four perpendicular walls drop straight back to the base.
	// frameScale is how many patch widths across the front face is, and the body
	// is as deep as the front face is wide, so the solid is a cube.
	frameScale := 1.5
	depth := frameScale * scale
	// Take the outward side from the patch itself -- (LR-LL)x(UL-LL) is the
	// du x dv that writeFace's winding treats as front -- so the body agrees
	// with the patch however the camera transform happens to be oriented.
	frontDir, _ := LR.Minus(LL).Cross(UL.Minus(LL)).Unit()
	front := geom.Vec(frontDir)
	back := front.Scaled(-depth)
	// The patch is the view frustum's z = 0 slice, so scaling a boundary point
	// about the view axis (the origin) lands it on the front face's edge and
	// keeps that edge straight.
	outset := func(p geom.Vec) geom.Vec { return p.Scaled(frameScale) }

	addVertex := func(p, normal geom.Vec, u, v float64) int32 {
		dir, _ := normal.Unit()
		index := int32(numVerticies)
		numVerticies++
		writeVertex(PlyDataBuffered, p, dir, u, v)
		return index
	}

	// The four sides, walked counter-clockwise as seen from the front: side i
	// runs from corner i to corner i+1, matching the patch boundary from
	// LL to LR to UR to UL.
	frameCorner := []geom.Vec{outset(LL), outset(LR), outset(UR), outset(UL)}
	baseCorner := make([]geom.Vec, 4)
	cornerUV := [][2]float64{{0, 0}, {2 * pi, 0}, {2 * pi, 2 * pi}, {0, 2 * pi}}
	frameIndex := make([]int32, 4)
	baseIndex := make([]int32, 4)
	wallTop := make([][2]int32, 4)
	wallBottom := make([][2]int32, 4)
	for i, p := range frameCorner {
		baseCorner[i] = p.Plus(back)
		frameIndex[i] = addVertex(p, front, cornerUV[i][0], cornerUV[i][1])
	}
	// Each wall gets its own vertices so the cube's edges stay crisp instead of
	// being rounded off by normals shared with the frame and the neighbouring
	// walls -- with a dielectric that difference is visible in the refraction.
	for i := range frameCorner {
		next := (i + 1) % 4
		normal := frameCorner[next].Minus(frameCorner[i]).Cross(front)
		wallTop[i] = [2]int32{
			addVertex(frameCorner[i], normal, cornerUV[i][0], cornerUV[i][1]),
			addVertex(frameCorner[next], normal, cornerUV[next][0], cornerUV[next][1]),
		}
		wallBottom[i] = [2]int32{
			addVertex(baseCorner[i], normal, cornerUV[i][0], cornerUV[i][1]),
			addVertex(baseCorner[next], normal, cornerUV[next][0], cornerUV[next][1]),
		}
	}
	for i, p := range baseCorner {
		baseIndex[i] = addVertex(p, back, cornerUV[i][0], cornerUV[i][1])
	}

	// The patch boundary, one list per side, in the same counter-clockwise
	// order. The frame fans off these, so it shares the patch's edge vertices
	// exactly and leaves no seam.
	sideIndices := make([][]int32, 4)
	sidePoints := make([][]geom.Vec, 4)
	appendBoundary := func(side, uIndex, vIndex int) {
		sideIndices[side] = append(sideIndices[side], vertexIndicies[uIndex][vIndex])
		sidePoints[side] = append(sidePoints[side],
			uv2xyz(index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t))
	}
	for uIndex := startUIndex; uIndex <= endUIndex; uIndex++ {
		appendBoundary(0, uIndex, startVIndex)
	}
	for vIndex := startVIndex; vIndex <= endVIndex; vIndex++ {
		appendBoundary(1, endUIndex, vIndex)
	}
	for uIndex := endUIndex; uIndex >= startUIndex; uIndex-- {
		appendBoundary(2, uIndex, endVIndex)
	}
	for vIndex := endVIndex; vIndex >= startVIndex; vIndex-- {
		appendBoundary(3, startUIndex, vIndex)
	}

	envmapArray := []float32{}
	blendArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			blendValue := float32(blendValue(u, v, t))
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			topRight := vertexIndicies[uIndex][vIndex]
			topLeft := vertexIndicies[uIndex+1][vIndex]
			botRight := vertexIndicies[uIndex][vIndex+1]
			botLeft := vertexIndicies[uIndex+1][vIndex+1]
			if topRight == -1 || topLeft == -1 || botRight == -1 || botLeft == -1 {
				continue
			}
			numFaces++
			writeFace(PlyDataBuffered, topRight, botLeft, topLeft)
			numFaces++
			writeFace(PlyDataBuffered, topRight, botRight, botLeft)
		}
	}

	// Frame: four planar fans, each from a front face corner across the
	// subdivided patch edge on that side. The frame is flat, so a fan is exact,
	// and its outer boundary is a single unsplit edge per side -- the same edge
	// the wall above it uses, so there are no T-junctions to crack open.
	for side := range frameCorner {
		next := (side + 1) % 4
		apex, apexPoint := frameIndex[side], frameCorner[side]
		indices, points := sideIndices[side], sidePoints[side]
		for j := 0; j+1 < len(indices); j++ {
			if indices[j] == -1 || indices[j+1] == -1 {
				continue
			}
			numFaces++
			emitFace(PlyDataBuffered, apex, indices[j], indices[j+1],
				apexPoint, points[j], points[j+1], front)
		}
		if last := len(indices) - 1; indices[last] != -1 {
			numFaces++
			emitFace(PlyDataBuffered, apex, indices[last], frameIndex[next],
				apexPoint, points[last], frameCorner[next], front)
		}
	}

	// Walls: each front face edge dropped straight back to the base.
	for side := range frameCorner {
		next := (side + 1) % 4
		normal := frameCorner[next].Minus(frameCorner[side]).Cross(front)
		numFaces += 2
		emitFace(PlyDataBuffered, wallTop[side][0], wallTop[side][1], wallBottom[side][1],
			frameCorner[side], frameCorner[next], baseCorner[next], normal)
		emitFace(PlyDataBuffered, wallTop[side][0], wallBottom[side][1], wallBottom[side][0],
			frameCorner[side], baseCorner[next], baseCorner[side], normal)
	}

	// Base.
	numFaces += 2
	emitFace(PlyDataBuffered, baseIndex[0], baseIndex[1], baseIndex[2],
		baseCorner[0], baseCorner[1], baseCorner[2], back)
	emitFace(PlyDataBuffered, baseIndex[0], baseIndex[2], baseIndex[3],
		baseCorner[0], baseCorner[2], baseCorner[3], back)
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			envmapValue := float32(pow(sin(u/2), 10) * pow(sin(v), 10))
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

	type sensor struct {
		Camera        geom.Vec
		LookAt        geom.Vec
		Distance      float64
		Angle         float64
		MinZ          float64
		FOV           float64
		Aperture      float64
		Height        int
		Width         int
		Samples       int
		IntIOR        float64
		ExtIOR        float64
		G             float64
		Abbe          float64
		FilmThickness float64
		FilmIOR       float64
		Roughness     float64
		Albedo        float64
		SigmaT        float64
		EnvX          float64
		EnvY          float64
		EnvZ          float64
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

        <sampler type="multijitter">
            <integer name="sample_count" value="{{ .Samples }}"/>
        </sampler>

        <film type="hdrfilm" id="film">
            <integer name="width" value="{{ .Width }}"/>
            <integer name="height" value="{{ .Height }}"/>
            <rfilter type="lanczos"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
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
        <float name="albedo" value="{{ .Albedo }}"/>
        <float name="sigma_t" value="{{ .SigmaT }}"/>
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
		<bsdf type="roughdielectric">
			<float name="alpha" value="{{ .Roughness }}"/>
			<float name="film_thickness" value="{{ .FilmThickness }}"/>
			<float name="film_ior" value="{{ .FilmIOR }}"/>
			<float name="int_ior" value="{{ .IntIOR }}"/>
			<float name="ext_ior" value="{{ .ExtIOR }}"/>
			<float name="abbe" value="{{ .Abbe }}"/>
		</bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
</scene>
`)

	intIor := 1 + 2*pow(10, cos(prime(5)*t)*3-3)
	extIor := 1.0
	if frameNumber%2 == 1 {
		extIor = intIor
		intIor = 1.0
	}

	// Pick the interference order / optical thickness you want to sweep through.
	// Order ~0.5–2 keeps you in the vivid, 4-wavelength-resolvable zone.
	opticalThickness := 100 + (sin(prime(6)*t)/2+0.5)*650 // n·d ∈ [100, 750] nm
	filmIor := 1.8 + sin(prime(7)*t)*.7                   // choose for material look
	filmThickness := opticalThickness / filmIor           // d follows from the two

	// Target dispersion strength: subtle to noticeable "flint-like" fire,
	// staying out of the rutile-grade noisy tail.
	dispStrength := 0.005 + (sin(prime(8)*t)/2+0.5)*0.020 // Δn ∈ [0.005, 0.025]
	abbe := (intIor - 1) / dispStrength                   // derived, not swept

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		0,
		minZ,
		fov * .9,
		pow(10, sin(prime(31)*t)*5-7),
		height,
		width,
		samples,
		intIor,
		extIor,
		cos(prime(4)*t) * .95,
		abbe,
		filmThickness,
		filmIor,
		pow(10, sin(prime(9)*t)*4-4),
		sin(prime(10)*t)*.5 + .5,
		pow(10, sin(prime(11)*t)*4-4),
		sin(prime(3)*t) * 90,
		sin(prime(2)*t) * 90,
		sin(prime(1)*t) * 90,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	aspectRatio := flag.Float64("aspectratio", 1.0, "Aspect ratio")
	height := flag.Int("height", 2560, "Height")
	samples := flag.Int("samples", 1024, "Samples")
	numRows := flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
