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
	// "github.com/hunterloftis/pbr/pkg/surface"
	// "github.com/hunterloftis/pbr/pkg/material"
)

var primes = []int{2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271}

func prime(i int) float64 {
	return float64(primes[44-i-1])
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
var round = math.Round
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

func strength(n, x float64) float64 {
	return pow(5, sin(n*(x+n/10))) * 2
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
	// Buffer between the frustum's extreme corner ray and the plane. Swept like
	// everything else, so the reachable grazing floor isn't rigidly tied to the
	// FOV: near 3 deg the camera can lean noticeably further over than a fixed
	// 6 deg allowed, near 12 deg it hangs back. Keep the low end well off zero
	// -- the far corner's distance goes as 1/sin(margin), so it runs away fast
	// below a couple of degrees.
	margin := (7.5 + 4.5*sin(prime(1)*t)) * pi / 180
	capRadius := pi/2 - frustumHalfAngle - margin
	if capRadius < 0 {
		capRadius = 0
	}
	theta := capRadius * (sin(prime(2)*t)/2 + .5) // in [0, capRadius): never reaches the edge
	azimuth := prime(3) * t
	radius := 4.5 + 2.5*sin(prime(4)*t) // distance from origin, breathes in [2, 7]
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
	return shaper(texture(u, v, t), sin(prime(5)*t), cos(prime(6)*t))/2 + .5
}

func texture(u, v, t float64) float64 {
	fu1 := sin(prime(7) * t)
	fu2 := cos(prime(8) * t)
	fu3 := sin(prime(9) * t)
	fu4 := cos(prime(10) * t)
	fu5 := sin(prime(11) * t)
	fu6 := cos(prime(12) * t)
	fu7 := sin(prime(13) * t)
	fu8 := cos(prime(14) * t)
	fv1 := sin(prime(15) * t)
	fv2 := cos(prime(16) * t)
	fv3 := sin(prime(17) * t)
	fv4 := cos(prime(18) * t)
	fv5 := sin(prime(19) * t)
	fv6 := cos(prime(20) * t)
	fv7 := sin(prime(21) * t)
	fv8 := cos(prime(22) * t)
	a1d1 := strength(prime(23), t)
	a1d2 := strength(prime(24), t)
	a1d3 := strength(prime(25), t)
	a2d3 := strength(prime(26), t)
	a2d2 := strength(prime(27), t)
	a3d3 := strength(prime(28), t)
	a4d3 := strength(prime(29), t)
	return cos(fu1*u + fv1*v +
		a1d1*sin(fu2*u+fv2*v+
			a1d2*cos(fu3*u+fv3*v+
				a1d3*sin(fu5*u+fv5*v)+
				a2d3*cos(fu6*u+fv6*v)+
				a2d2*sin(fu4*u+fv4*v+
					a3d3*cos(fu7*u+fv7*v)+
					a4d3*sin(fu8*u+fv8*v)))))

}

func shaper(x, a, b float64) float64 {
	return spow(pow(x/2+.5, pow(10, a))*2-1, pow(10, b))
}

func shape(u, v, t float64) geom.Vec {
	a := spow(u/2/pi*(1-u/2/pi)*v/2/pi*(1-v/2/pi), .1)
	return rectangle(u, v, t).Plus(geom.Vec{
		0,
		0,
		a * scale * .2 * cos(prime(43)*t) * texture(u, v, t),
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
	// The patch is a plain axis-aligned rectangle in the z = 0 plane, centred on
	// the origin. Series 184 made it the view frustum's z = 0 slice instead -- a
	// keystone that changed shape every frame -- purely so that no triangle was
	// spent outside the frame. The view camera does that job properly: with the
	// film standard parallel to z = 0 the patch images as a rectangle and fills
	// the frame exactly, and the camera's off-axis position is carried by back
	// rise/shift rather than by pointing the camera at the patch. The cost is
	// that magnification across the patch is now uniform, so the in-plane
	// foreshortening 184 got from the oblique lookat is gone; the relief and the
	// body still show perspective, from the off-axis lens.
	//
	// This angle no longer reaches the sensor. It sets how wide the patch is
	// relative to the camera distance, which is what the old FOV sweep was
	// really buying: the strength of the perspective on the relief, from
	// near-orthographic at 5 deg to steep at 120.
	patchSubtense := 62.5 + 57.5*sin(prime(30)*t)
	halfTanH := tan(patchSubtense / 2 * pi / 180)
	// Angular radius of a patch corner seen from a camera at `radius` on the
	// plane's normal. The camera path uses it to stay far enough above the plane
	// that every corner is still seen at a decent grazing angle -- the same use
	// 184 made of it, and still a conservative bound now that the framing comes
	// from the film rather than from a cone about the view axis.
	tanV := halfTanH / aspectRatio
	frustumHalfAngle := atan(sqrt(halfTanH*halfTanH + tanV*tanV))
	cameraLoc := cameraPath(t, frustumHalfAngle)
	// The camera looks straight down the plane's normal, so the film is parallel
	// to the patch. Everything oblique about the view is in the back movements.
	lookAt := cameraLoc.Plus(geom.Vec{0, 0, -1})
	c := NewSLR2().MoveTo(cameraLoc).LookAt(lookAt)
	c.AspectRatio = aspectRatio
	// The film's *pixel* aspect, not the nominal one: `width` is truncated to an
	// integer, and the patch has to match the film exactly or a sliver of the
	// frame falls off its edge.
	filmAspect := float64(width) / float64(height)
	halfWidth := cameraLoc.Len() * halfTanH
	halfHeight := halfWidth / filmAspect
	// Named for where they land in the frame, as before: with this camera image
	// left is world -x and image up is world +y, so u still runs left to right
	// across the frame and v still runs bottom to top.
	LL = geom.Vec{-halfWidth, -halfHeight, 0}
	LR = geom.Vec{halfWidth, -halfHeight, 0}
	UL = geom.Vec{-halfWidth, halfHeight, 0}
	UR = geom.Vec{halfWidth, halfHeight, 0}
	scale = LR.Minus(LL).Len()
	distance := cameraLoc.Minus(focusPoint).Len()
	fmt.Printf("\nyScale: %v\nUL: %v\nUR: %v\nLL: %v\nLR: %v\ncameraLoc: %v\nfocusPoint: %v\ndistance: %v\nt: %#v\nscale: %v\n",
		yScale, UL, UR, LL, LR, cameraLoc, focusPoint, distance, t, scale)
	fmt.Printf("patchSubtense: %v, filmAspect: %v, aspectRatio: %v\n", patchSubtense, filmAspect, aspectRatio)

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
	distanceWeight := cos(prime(31)*t)/5 + .5
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
	frameScale := 10.0
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
		FocalLength   float64
		Bellows       float64
		FilmWidth     float64
		BackShift     float64
		BackRise      float64
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
    <sensor type="viewcamera" id="Camera-camera">
        <float name="focal_length" value="{{ .FocalLength }}"/>
        <float name="bellows_extension" value="{{ .Bellows }}"/>
        <float name="film_width" value="{{ .FilmWidth }}"/>
        <float name="back_shift" value="{{ .BackShift }}"/>
        <float name="back_rise" value="{{ .BackRise }}"/>
        <float name="aperture_radius" value="{{ .Aperture }}"/>
        <boolean name="falloff" value="false"/>
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
		<bsdf type="blendbsdf">
			<texture type="bitmap" name="weight">
				<string name="filename" value="mitsuba.blend.rgbe"/>
				<boolean name="raw" value="true"/>
			</texture>
			<bsdf type="roughdielectric">
				<float name="alpha" value="{{ .Roughness }}"/>
				<float name="film_thickness" value="{{ .FilmThickness }}"/>
				<float name="film_ior" value="{{ .FilmIOR }}"/>
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
			<bsdf type="roughdielectric">
				<float name="alpha" value="{{ .Roughness }}"/>
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="ext_ior" value="{{ .ExtIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
		</bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
</scene>
`)

	// Index contrast, swept over [1.02, 3]. The exponent multiplier is the knob:
	// at k it bottoms out at 1 + 2*10^-2k, and at the 3 this used to carry the
	// floor was 1.000002 -- so far below the ~3e-3 where any structure at all
	// becomes measurable that better than half the frames rendered as a flat grey
	// veil, the object being optically index-matched with its surroundings.
	// k = 1 keeps the top of the range and lifts the floor to just above where
	// the effect is detectable, costing nothing that was ever visible.
	intIor := 1 + 2*pow(10, cos(prime(32)*t)-1)
	extIor := 1.0
	if frameNumber%2 == 1 {
		extIor = intIor
		intIor = 1.0
	}

	// Pick the interference order / optical thickness you want to sweep through.
	// Order ~0.5–2 keeps you in the vivid, 4-wavelength-resolvable zone.
	opticalThickness := 100 + (sin(prime(33)*t)/2+0.5)*650 // n·d ∈ [100, 750] nm
	filmIor := 1.8 + sin(prime(34)*t)*.7                   // choose for material look
	filmThickness := opticalThickness / filmIor            // d follows from the two

	// Target dispersion strength: subtle to noticeable "flint-like" fire,
	// staying out of the rutile-grade noisy tail.
	dispStrength := 0.005 + (sin(prime(35)*t)/2+0.5)*0.020 // Δn ∈ [0.005, 0.025]
	abbe := (max(intIor, extIor) - 1) / dispStrength       // derived, not swept

	// --- View camera ---------------------------------------------------------
	// Both standards are neutral and the film is parallel to the patch plane, so
	// the plane of focus is parallel to the top of the object -- no tilt, no
	// Scheimpflug -- and it is set on the patch plane itself.
	//
	// The aperture is a pinhole, so everything is in focus and the focus distance
	// is only a gauge: at a zero aperture every ray is a chief ray through the
	// lens centre, and only film_width/bellows and the shifts over bellows reach
	// the image. Both are derived from the bellows below, so the framing does not
	// care what f is. A quarter of the focus distance puts the bellows at a 4/3
	// draw, which makes the magnification exactly 1/3 on every frame.
	focusDistance := cameraLoc.Z
	focalLength := focusDistance / 4
	bellows := 1 / (1/focalLength - 1/focusDistance) // = focusDistance/3
	// The patch plane sits at axial distance cameraLoc.Z from the lens, and the
	// film is parallel to it, so magnification is one number for the whole patch.
	magnification := bellows / cameraLoc.Z
	// 184 rendered at fov*.9 so the patch overran the frame by ~10% and its edge
	// could never show. The fit here is exact, so the same margin is applied to
	// the film instead -- kept in its original angular form so the two series
	// frame alike; a flat constant would zoom in ~13% at the wide end.
	frameFill := tan(patchSubtense*.9/2*pi/180) / halfTanH
	frameWidth := scale * frameFill // what the frame spans on the patch plane
	filmWidth := magnification * frameWidth
	// The patch is centred on the origin while the axis pierces the plane at
	// (cameraLoc.X, cameraLoc.Y), so the film has to move to meet it. Camera +x is
	// world -x here and the film inverts, which is where the signs come from.
	// film_height is left to the plugin: it derives it from film_width and the
	// pixel aspect, which is the aspect the patch was built to.
	backShift := -magnification * cameraLoc.X
	backRise := magnification * cameraLoc.Y
	// A pinhole, fixed on every frame: nothing is ever out of focus. Zero is legal
	// here, unlike thinlens, which clamps it to epsilon.
	//
	// "Small but nonzero" would not do the same job. In object space the blur cone
	// opens linearly with distance from the plane of focus, and the body runs ten
	// patch widths back behind it and shows through the dielectric, so an aperture
	// small enough to keep the refracted interior sharp is a pinhole in all but
	// name. Depth of field here is all or nothing.
	apertureRadius := 0.0
	fmt.Printf("focalLength: %v, bellows: %v, magnification: %v, filmWidth: %v, backShift: %v, backRise: %v\n",
		focalLength, bellows, magnification, filmWidth, backShift, backRise)
	fmt.Printf("focusDistance: %v, apertureRadius: %v, minZ: %v\n",
		focusDistance, apertureRadius, minZ)

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		lookAt,
		focalLength,
		bellows,
		filmWidth,
		backShift,
		backRise,
		apertureRadius,
		height,
		width,
		samples,
		intIor,
		extIor,
		cos(prime(36)*t) * .95,
		abbe,
		filmThickness,
		filmIor,
		pow(10, sin(prime(37)*t)*4-4),
		sin(prime(38)*t)*.5 + .5,
		pow(10, sin(prime(39)*t)*4-4),
		sin(prime(40)*t) * 179,
		sin(prime(41)*t) * 179,
		sin(prime(42)*t) * 179,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 512, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	aspectRatio := flag.Float64("aspectratio", 16.0/9.0, "Aspect ratio")
	height := flag.Int("height", 2304, "Height")
	samples := flag.Int("samples", 1024, "Samples")
	numRows := flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
