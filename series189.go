//go:build ignore

package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"math"
	"os"
	"runtime"
	"sync"
	"sync/atomic"
	"text/template"

	"github.com/Opioid/rgbe"
	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/rrothenb/fm-animations/utils"
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
var t = 0.0
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
	return pow(2, sin(pow(float64(n), .5)*(x+float64(n)/3)))
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

var zAxis = geom.Dir{.707, .707, 0}

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
	cameraSpaceTransform := s.trans.Inverse()
	projectedPoint := cameraSpaceTransform.MultPoint(point)
	//fmt.Printf("\npoint: %#v\nprojectedPoint: %#v\ncameraSpaceTransform: %#v\n", point, projectedPoint, cameraSpaceTransform)
	factor := tan(s.FOV * 1.1 / 360 * pi)
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

func lissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	return geom.Vec{sin(float64(xN) * t), sin(float64(yN) * t), cos(float64(zN) * t)}
}

func unitLissajousKnot(t float64, xN, yN, zN int) geom.Vec {
	point, _ := lissajousKnot(t, xN, yN, zN).Unit()
	return geom.Vec(point)
}

func outerKnot(t float64) geom.Vec {
	return torusKnot(t, 1, 0.4, 3, 2, circle)
}

func outerKnotMaxRadius() float64 {
	rMax, _, _ := utils.MaxTubeRadius(outerKnot, 2*pi, 4000)
	return rMax
}

// weave is the seamless woven sleeve on outerKnot: the (100, K) and (100, -K)
// cables woven plain, a 2-component link. With flat .3, lift 1.1, crimp 3 at
// 95% fill, yarn .4512 cell is where the two threads first touch (on the outside
// of the trefoil's tightest bends, per YarnClearance); .45 clears it by ~1%.
var weave = func() *utils.CableWeave {
	w := utils.FitCableWeave(outerKnot, 100, .95*outerKnotMaxRadius(), .45, .3, 1.1, 20000)
	w.Crimp = 3
	return w
}()

// numSurfaces is how many separate tubes uv2xyz describes (s = 0..numSurfaces-1).
const numSurfaces = 2

// cameraPath wanders through and around the knot along a (3,4,5) Lissajous knot
// sized to the object's bounding box (about +-1.6 across, +-.8 tall), with no
// regard for whether it's inside or outside the yarns.
// insideFOV is the FOV when the camera is inside the sleeve.
const insideFOV = 100.0

// maxBackground is the fraction of the frame an outside camera's FOV may leave
// showing background, between minFOV and maxFOV: none, even if that crops the
// knot.
const maxBackground, minFOV, maxFOV = 0, 10.0, 150.0

// centerline is outerKnot as a closed polyline, for distances to the sleeve.
var centerline = func() []geom.Vec {
	pts := make([]geom.Vec, 2000)
	for i := range pts {
		pts[i] = outerKnot(index2radians(float64(i), len(pts)))
	}
	return pts
}()

// centerlineDistance is p's distance from outerKnot.
func centerlineDistance(p geom.Vec) float64 {
	best := math.Inf(1)
	for i, a := range centerline {
		b := centerline[(i+1)%len(centerline)]
		ab := b.Minus(a)
		x := max(0, min(1, p.Minus(a).Dot(ab)/ab.Dot(ab)))
		best = min(best, p.Minus(a.Plus(ab.Scaled(x))).Len())
	}
	return best
}

// background is the fraction of camera rays, over a grid across a fov-degree
// frame (fov along the larger axis, as Mitsuba's fov_axis="larger"), that miss
// the sleeve, treated as a solid tube of radius weave.Outer() around outerKnot.
func background(c *SLR2, fov float64) float64 {
	const n = 64
	tanX := tan(fov / 360 * pi)
	tanY := tanX / c.AspectRatio
	if c.AspectRatio < 1 {
		tanY = tanX
		tanX = tanY * c.AspectRatio
	}
	origin := c.position
	outer := weave.Outer()
	misses := 0
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			x := (2*(float64(i)+.5)/n - 1) * tanX
			y := (2*(float64(j)+.5)/n - 1) * tanY
			// Camera space looks down -Z.
			dir, _ := c.trans.MultPoint(geom.Vec{x, y, -1}).Minus(origin).Unit()
			// Sphere-trace the tube.
			hit := false
			for d, steps := 0.0, 0; d < 10 && steps < 200; steps++ {
				step := centerlineDistance(origin.Plus(geom.Vec(dir).Scaled(d))) - outer
				if step < 1e-3 {
					hit = true
					break
				}
				d += step
			}
			if !hit {
				misses++
			}
		}
	}
	return float64(misses) / n / n
}

// fillFOV is the widest FOV (between minFOV and maxFOV) that keeps the
// background under maxBackground, so the knot fills the frame.
func fillFOV(c *SLR2) float64 {
	if background(c, minFOV) > maxBackground {
		return minFOV
	}
	lo, hi := minFOV, maxFOV
	if background(c, hi) <= maxBackground {
		return hi
	}
	for hi-lo > .5 {
		mid := (lo + hi) / 2
		if background(c, mid) <= maxBackground {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

func cameraPath(t float64) geom.Vec {
	// Phases keep it off the axes when the frame count is a multiple of 4.
	return geom.Vec{sin(3*t + .3), sin(4*t + .7), cos(5*t + 1.1)}.By(geom.Vec{1.6, 1.6, .9})
}

// focusPath runs along the centerline of thread c over the series.
func focusPath(c int, t float64) geom.Vec {
	p, _, _ := weave.Thread(c, t)
	return p
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
	// strength depends only on t; compute each once rather than per term.
	s7, s11, s19, s23, s29, s31 := strength(7, t), strength(11, t), strength(19, t), strength(23, t), strength(29, t), strength(31, t)
	return sin(
		a*s7*sin(a*s23*loc.Z) +
			a*s7*sin(a*s23*loc.Y) +
			a*s7*sin(a*s23*loc.X) +
			a*s11*sin(a*s19*2*loc.X+a*s29*sin(a*s31*3*loc.Y)) +
			a*s11*sin(a*s19*2*loc.X+a*s29*sin(a*s31*3*loc.Z)) +
			a*s11*sin(a*s19*2*loc.Y+a*s29*sin(a*s31*3*loc.X)) +
			a*s11*sin(a*s19*2*loc.Y+a*s29*sin(a*s31*3*loc.Z)) +
			a*s11*sin(a*s19*2*loc.Z+a*s29*sin(a*s31*3*loc.X)) +
			a*s11*sin(a*s19*2*loc.Z+a*s29*sin(a*s31*3*loc.Y)) +
			a*s11*sin(a*s19*2*loc.X-a*s29*sin(a*s31*3*loc.Y)) +
			a*s11*sin(a*s19*2*loc.X-a*s29*sin(a*s31*3*loc.Z)) +
			a*s11*sin(a*s19*2*loc.Y-a*s29*sin(a*s31*3*loc.X)) +
			a*s11*sin(a*s19*2*loc.Y-a*s29*sin(a*s31*3*loc.Z)) +
			a*s11*sin(a*s19*2*loc.Z-a*s29*sin(a*s31*3*loc.X)) +
			a*s11*sin(a*s19*2*loc.Z-a*s29*sin(a*s31*3*loc.Y)))
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

// filmPattern is the film-thickness texture (0..1) at loc, a point on the
// yarn surface already scaled by 2pi. Its sharpness sweeps from nearly a step
// (exponent .1) to soft, with 0 the most common value (exponent 10).
func filmPattern(loc geom.Vec) float64 {
	sharpness := pow(10, sin(43*t+2.3))
	value := pow(spow(shapeTexture(1, .75-cos(41*t)*.5, t, loc), sharpness)/2+.5, pow(10, sin(29*t)))
	if frameNumber%2 == 1 {
		value = 1 - value
	}
	return value
}

// uv2xyz: the weave's two yarns, u around each yarn, v along it.
func uv2xyz(s int, u, v, t float64) geom.Vec {
	p, up, side, a, b := weave.Section(s, v)
	return p.Plus(up.Scaled(b * cos(u))).Plus(side.Scaled(a * sin(u)))
}

// parallelFor runs body(i) for i in [0, n) across all cores, in chunks.
func parallelFor(n int, body func(i int)) {
	const chunk = 1024
	var next atomic.Int64
	var wg sync.WaitGroup
	for w := 0; w < runtime.NumCPU(); w++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for {
				start := int(next.Add(chunk)) - chunk
				if start >= n {
					return
				}
				end := start + chunk
				if end > n {
					end = n
				}
				for i := start; i < end; i++ {
					body(i)
				}
			}
		}()
	}
	wg.Wait()
}

func index2radians(index float64, n int) float64 {
	return index / float64(n) * pi * 2
}

func uvIndexToNormal(s, uIndex, vIndex, nU int, nV int, t float64) *geom.Dir {
	left := uv2xyz(s, index2radians(float64(uIndex)-.1, nU), index2radians(float64(vIndex), nV), t)
	right := uv2xyz(s, index2radians(float64(uIndex)+.1, nU), index2radians(float64(vIndex), nV), t)
	up := uv2xyz(s, index2radians(float64(uIndex), nU), index2radians(float64(vIndex)+.1, nV), t)
	down := uv2xyz(s, index2radians(float64(uIndex), nU), index2radians(float64(vIndex)-.1, nV), t)
	// Outward: Mitsuba puts a shape's interior medium on the side away from its
	// normals, so inward normals would fill the whole world with the medium.
	normal, _ := right.Minus(left).Cross(up.Minus(down)).Unit()
	return &normal
}

func renderSurfaces(frame int, pixels int, maxSubdivisions int, dt float64, desiredTriangles int, aspectRatio float64, height int, samples int, numRows int) {
	frameNumber = frame
	width := int(aspectRatio * float64(height))
	t = float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	// Alternate threads every frame.
	focusPoint := focusPath(frameNumber%2, t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	c.AspectRatio = aspectRatio
	if centerlineDistance(cameraLoc) < weave.Outer() {
		// Inside the sleeve the knot surrounds the camera: a fixed wide FOV.
		c.FOV = insideFOV
		fmt.Printf("camera inside the sleeve, fov: %v\n", c.FOV)
	} else {
		c.FOV = fillFOV(c)
		fmt.Printf("camera outside the sleeve, fov: %v (background %.1f%%)\n", c.FOV, 100*background(c, c.FOV))
	}
	fmt.Printf("fov: %v\n", c.FOV)
	distance := cameraLoc.Minus(focusPoint).Len()
	fmt.Printf("\ncameraLoc: %v\nfocusPoint: %v\ndistance: %v\nt: %#v\n", cameraLoc, focusPoint, distance, t, c)
	// v must resolve the weave: 2*N*K crossings along each yarn, ~12 rings each.
	nU := 16
	nV := 2 * weave.N * weave.K * 12
	minZ := 1.0
	if desiredTriangles > 0 {
		// Average how fast the surface stretches in u and in v (derivatives, not
		// chords between the 500 samples, which alias on fast-oscillating
		// surfaces) so the quads come out square, and spend desiredTriangles
		// across all surfaces.
		totalWidth := 0.0
		totalHeight := 0.0
		const eps = 1e-7
		for s := 0; s < numSurfaces; s++ {
			for uIndex := 0; uIndex <= 500; uIndex++ {
				for vIndex := 0; vIndex <= 500; vIndex++ {
					u, v := index2radians(float64(uIndex), 500), index2radians(float64(vIndex), 500)
					vertex := uv2xyz(s, u, v, t)
					totalWidth += uv2xyz(s, u+eps, v, t).Minus(vertex).Len() / eps
					totalHeight += uv2xyz(s, u, v+eps, t).Minus(vertex).Len() / eps
					minZ = math.Min(minZ, vertex.Z)
				}
			}
		}
		fmt.Println(minZ)
		ratio := totalWidth / totalHeight
		// nU*nV quads per surface, at 2 triangles each, over numSurfaces surfaces.
		quads := float64(desiredTriangles) / 2 / numSurfaces
		nU = int(sqrt(quads * ratio))
		nV = int(sqrt(quads / ratio))
	}
	fmt.Printf("distance from center: %v, distance from focal point: %v, nU: %v, nV: %v\n", cameraLoc.Len(), distance, nU, nV)
	fmt.Printf("%.1f rings per weave crossing\n", float64(nV)/float64(2*weave.N*weave.K))
	if nU < 3 || nV < 3 {
		fmt.Printf("nU %v, nV %v: too few triangles to mesh the yarns\n", nU, nV)
		os.Exit(1)
	}
	// The textures are independent of the mesh: surface s owns the band of rows
	// [s, s+1]/numSurfaces, so its v texture coordinate is squeezed into it. The
	// previews (data/N.texture.rgbe, data/N.blend.rgbe) have their long side
	// capped at JPEG's 65500 so run.sh can convert them to view. The film
	// thickness Mitsuba reads (mitsuba.film.pfm) is single-channel float with
	// square-ish texels, texelsPerTriangle per triangle of the mesh (600M, 2.4
	// GB, at 150M triangles): the pattern is nearly a step, so its contours are
	// only as smooth as the texels are small.
	const texelsPerTriangle = 4
	const maxTexSize = 65500
	texCols := int(min(float64(nU), maxTexSize))
	texRows := int(min(float64(nV), maxTexSize/numSurfaces))
	fmt.Printf("textures: %v x %v\n", texCols, numSurfaces*texRows)
	texU := func(uIndex int) float32 { return float32(uIndex) / float32(nU) }
	texV := func(s, vIndex int) float32 { return (float32(s) + float32(vIndex)/float32(nV)) / numSurfaces }
	// Every vertex, face and texel is independent: fill them in parallel into
	// buffers laid out in file order, then write each buffer in one go.
	// Vertex (s, uIndex, vIndex) is number (s*(nU+1)+uIndex)*(nV+1)+vIndex.
	vertexIndex := func(s, uIndex, vIndex int) int32 { return int32((s*(nU+1)+uIndex)*(nV+1) + vIndex) }
	const vertexSize = 8 * 4
	numVerticies := numSurfaces * (nU + 1) * (nV + 1)
	vertexData := make([]byte, numVerticies*vertexSize)
	parallelFor(numVerticies, func(i int) {
		s, uIndex, vIndex := i/((nU+1)*(nV+1)), i/(nV+1)%(nU+1), i%(nV+1)
		vertex := uv2xyz(s, index2radians(float64(uIndex), nU), index2radians(float64(vIndex), nV), t)
		normal := uvIndexToNormal(s, uIndex, vIndex, nU, nV, t)
		b := vertexData[i*vertexSize:]
		for k, x := range []float32{float32(vertex.X), float32(vertex.Y), float32(vertex.Z),
			float32(normal.X), float32(normal.Y), float32(normal.Z), texU(uIndex), texV(s, vIndex)} {
			binary.LittleEndian.PutUint32(b[4*k:], math.Float32bits(x))
		}
	})
	// Two triangles per quad, quads in (s, vIndex, uIndex) order.
	const faceSize = 1 + 3*4
	numFaces := numSurfaces * nV * nU * 2
	faceData := make([]byte, numFaces*faceSize)
	parallelFor(numFaces/2, func(i int) {
		s, vIndex, uIndex := i/(nV*nU), i/nU%nV, i%nU
		topRight := vertexIndex(s, uIndex, vIndex)
		topLeft := vertexIndex(s, uIndex+1, vIndex)
		botRight := vertexIndex(s, uIndex, vIndex+1)
		botLeft := vertexIndex(s, uIndex+1, vIndex+1)
		// Wound so the face normals point out of the yarn.
		for k, tri := range [2][3]int32{{topRight, topLeft, botLeft}, {topRight, botLeft, botRight}} {
			b := faceData[(2*i+k)*faceSize:]
			b[0] = 3
			for j, index := range tri {
				binary.LittleEndian.PutUint32(b[1+4*j:], uint32(index))
			}
		}
	})
	plyDataPath := fmt.Sprintf("data/%v.data.ply", frameNumber)
	plyData, _ := os.Create(plyDataPath)
	plyData.Write(vertexData)
	plyData.Write(faceData)
	plyData.Close()
	envmapArray := []float32{}
	// Gray: rgbe always has 3 channels; Mitsuba reads equal ones as that value.
	blendArray := make([]float32, numSurfaces*texRows*texCols*3)
	textureArray := make([]float32, numSurfaces*texRows*texCols*3)
	parallelFor(numSurfaces*texRows*texCols, func(i int) {
		s, row, col := i/(texRows*texCols), i/texCols%texRows, i%texCols
		// Sample the 3D textures at the texel's center on the yarn.
		u := index2radians(float64(col)+.5, texCols)
		v := index2radians(float64(row)+.5, texRows)
		loc := weave.Tube(s, u, v).Scaled(2 * pi)
		blendValue := float32(pow(spow(shapeTexture(2, 1, t, loc), 10)/2+.5, 10))
		textureValue := float32(filmPattern(loc))
		for k := 0; k < 3; k++ {
			blendArray[3*i+k] = blendValue
			textureArray[3*i+k] = textureValue
		}
	})
	// Two triangles per quad, so each quad side gets sqrt(2*texelsPerTriangle) texels.
	filmScale := sqrt(2 * texelsPerTriangle)
	filmCols, filmRows := int(float64(nU)*filmScale), int(float64(nV)*filmScale)
	fmt.Printf("film: %v x %v\n", filmCols, numSurfaces*filmRows)
	// PFM rows run bottom to top (Mitsuba flips them back on reading), so file
	// row j is texture row numSurfaces*filmRows-1-j.
	// Each texel is the pattern's average over the texel, not its value at the
	// center: a near-step pattern then stores how much of the texel is on each
	// side, and bilinear filtering puts the contour between texels instead of
	// along their edges. 2x2 samples first; if they differ by a jump rather than
	// a gentle slope (whose 2x2 average is already accurate) the texel holds an
	// edge and gets filmSupersample^2 samples.
	const filmSupersample = 4
	filmData := make([]byte, numSurfaces*filmRows*filmCols*4)
	parallelFor(numSurfaces*filmRows*filmCols, func(i int) {
		s, row, col := i/(filmRows*filmCols), i/filmCols%filmRows, i%filmCols
		average := func(n int) (mean, spread float64) {
			lo, hi := math.Inf(1), math.Inf(-1)
			for a := 0; a < n; a++ {
				for b := 0; b < n; b++ {
					u := index2radians(float64(col)+(float64(a)+.5)/float64(n), filmCols)
					v := index2radians(float64(row)+(float64(b)+.5)/float64(n), filmRows)
					value := filmPattern(weave.Tube(s, u, v).Scaled(2 * pi))
					mean += value
					lo, hi = math.Min(lo, value), math.Max(hi, value)
				}
			}
			return mean / float64(n*n), hi - lo
		}
		value, spread := average(2)
		if spread > .25 {
			value, _ = average(filmSupersample)
		}
		j := (numSurfaces*filmRows-1-(s*filmRows+row))*filmCols + col
		binary.LittleEndian.PutUint32(filmData[4*j:], math.Float32bits(float32(value)))
	})
	for vIndex := 0; vIndex < envSize; vIndex++ {
		for uIndex := 0; uIndex < envSize; uIndex++ {
			u := float64(uIndex) / float64(envSize) * 2 * pi
			v := float64(vIndex) / float64(envSize) * pi
			power := 2 * pow(10, sin(5*t)/2+.5)
			envmapValue := float32(pow(sin(u/2), power) * pow(sin(v), power))
			envmapArray = append(envmapArray, envmapValue, envmapValue, envmapValue)
		}
	}

	fmt.Println("Mitsuba!")
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
	texturePath := fmt.Sprintf("data/%v.texture.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, texCols, numSurfaces*texRows, blendArray)
	texture, _ := os.Create(texturePath)
	rgbe.Encode(texture, texCols, numSurfaces*texRows, textureArray)
	film, _ := os.Create("mitsuba.film.pfm")
	fmt.Fprintf(film, "Pf\n%d %d\n-1.0\n", filmCols, numSurfaces*filmRows)
	film.Write(filmData)
	film.Close()
	sensorFile, _ := os.Create("sensor.xml")

	type sensor struct {
		Camera        geom.Vec
		LookAt        geom.Vec
		Distance      float64
		MinZ          float64
		FOV           float64
		Aperture      float64
		Height        int
		Width         int
		Samples       int
		RowHeight     int
		Scale         float64
		G             float64
		Red           float64
		Green         float64
		Blue          float64
		Red2          float64
		Green2        float64
		Blue2         float64
		EnvX          float64
		EnvY          float64
		EnvZ          float64
		BSDF          string
		IntIOR        float64
		ExtIOR        float64
		FilmIOR       float64
		FilmThickness float64
		Abbe          float64
		Albedo        float64
		SigmaT        float64
	}
	sensorTemplate, _ := template.New("some template").Parse(`
<scene version="2.0.0">
    <sensor type="perspective" id="Camera-camera">
        <string name="fov_axis" value="larger"/>
        <float name="fov" value="{{ .FOV }}"/>
        <transform name="to_world">
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="1, 1, 0"/>
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
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="{{ .EnvX }}"/>
            <rotate value="0, 1, 0" angle="{{ .EnvY }}"/>
            <rotate value="0, 0, 1" angle="{{ .EnvZ }}"/>
        </transform>
    </emitter>
	<integrator type="nanscrub">
        <integrator type="volpathmis">
            <integer name="max_depth" value="16"/>
        </integrator>
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
		<bsdf type="{{ .BSDF }}">
			<float name="int_ior" value="{{ .IntIOR }}"/>
			<float name="ext_ior" value="{{ .ExtIOR }}"/>
			<float name="abbe" value="{{ .Abbe }}"/>
			<float name="film_ior" value="{{ .FilmIOR }}"/>
			<float name="film_thickness_scale" value="{{ .FilmThickness }}"/>
			<texture type="bitmap" name="film_thickness">
				<string name="filename" value="mitsuba.film.pfm"/>
				<boolean name="raw" value="true"/>
			</texture>
		</bsdf>
        <ref id="medium1" name="interior"/>
    </shape>
</scene>
`)

	// Even frames are a solid dielectric, odd frames a thindielectric shell.
	bsdf := "dielectric"
	if frameNumber%2 == 1 {
		bsdf = "thindielectric"
	}
	// IORs span common transparent solids, from ice / fluoropolymer (1.3)
	// through glasses and plastics to diamond (2.42). The object's is swapped
	// inside/outside every other pair of frames so both thindielectric and
	// dielectric see eta above and below 1.
	// Each sweep gets its own phase: with few frames (t = k*2pi/maxFrames) the
	// frequencies alias onto a handful of patterns and would move in lockstep.
	intIor := 1.86 + sin(23*t)*.56
	extIor := 1.0
	if (frameNumber/2)%2 == 1 {
		extIor = intIor
		intIor = 1.0
	}
	// The film needs index contrast on both its faces to interfere, so it sweeps
	// the same 1.3-2.42 range minus a band of filmGap around the object's IOR
	// (the 1.0 side is always far enough).
	const filmGap = .3
	n := max(intIor, extIor)
	below := max(0, n-filmGap-1.3)
	above := max(0, 2.42-(n+filmGap))
	x := (sin(31*t+1.2)/2 + .5) * (below + above)
	filmIor := 1.3 + x
	if x > below {
		filmIor = n + filmGap + (x - below)
	}
	// Abbe from 10 (beyond the densest flints) to 250 (beyond fluorite),
	// geometrically around 50.
	abbe := 50 * pow(5, sin(37*t+3.6))
	// Medium: albedo (scattering / extinction) from 0, all absorption, to 1, all
	// scattering. sigma_t (extinction per unit length) from .1 to 1000: a yarn is
	// ~.01 across, so its optical depth runs from .001 (clear) to 10 (opaque).
	// g from back- to forward-scattering.
	albedo := sin(53*t+6)/2 + .5
	sigmaT := pow(10, sin(59*t+.9)*2+1)
	g := sin(61*t+3.1) * .99
	fmt.Printf("albedo: %v, sigma_t: %v, g: %v\n", albedo, sigmaT, g)
	// mitsuba.film.pfm is in [0, 1]; its max maps to an optical thickness
	// (film_ior * nm) swept through the vivid interference orders, 150 to 800 nm.
	filmThickness := (475 + sin(67*t+5.4)*325) / filmIor
	fmt.Printf("bsdf: %v, int_ior: %v, ext_ior: %v, film_ior: %v, abbe: %v, film max: %v nm (optical %v)\n",
		bsdf, intIor, extIor, filmIor, abbe, filmThickness, filmThickness*filmIor)

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		minZ,
		c.FOV,
		0, // pinhole: no aperture
		height,
		width,
		samples,
		height / numRows,
		pow(10, sin(17*t)+2),
		g,
		cos(2*t)/3 + .666,
		sin(3*t)/3 + .666,
		sin(5*t)/3 + .666,
		.666 - cos(2*t)/3,
		.666 - sin(3*t)/3,
		.666 - sin(5*t)/3,
		0,
		sin(3*t) * 120,
		sin(2*t) * 179,
		bsdf,
		intIor,
		extIor,
		filmIor,
		filmThickness,
		abbe,
		albedo,
		sigmaT,
	})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	aspectRatio := flag.Float64("aspectratio", 1.0, "Aspect ratio")
	height := flag.Int("height", 6144, "Height")
	samples := flag.Int("samples", 1024, "Samples")
	numRows := flag.Int("numrows", 1, "Number rows")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles, *aspectRatio, *height, *samples, *numRows)
}
