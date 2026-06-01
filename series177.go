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

var globalFrameNumber = 0

var sin = math.Sin
var cos = math.Cos
var pow = math.Pow
var sqrt = math.Sqrt
var pi = math.Pi
var abs = math.Abs
var min = math.Min
var max = math.Max
var atan = math.Atan
var tan = math.Tan

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

func pathWrapper(u, v, r float64, path func(x float64) geom.Vec) geom.Vec {
	delta := .01
	center := path(v)
	normal, _ := path(v + delta).Minus(path(v - delta)).Unit()
	sinVec, _ := normal.Cross(geom.Dir{0, 0, 1})
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * cos(u)).Plus(sinVec.Scaled(r * sin(u))).Plus(center)
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
	loc.X = abs(loc.X) * 10
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
	a := cos(t) * .5
	return geom.Vec{
		sin(v/2.0+a*sin(v)) * cos(u-a*sin(2*u)),
		sin(v/2.0+a*sin(v)) * sin(u+a*sin(2*u)),
		cos(v/2.0 - a*sin(v)),
	}
}

// Modulators carries the Fourier coefficient arrays for the generalized
// sphereish surface. All three modulators use frequencies 1, 2, 3, ...
//
//	A[k-1] is the coefficient of sin(k·v) in the radial modulator.
//	B[k-1] is the coefficient of sin(k·u) in the angular modulator.
//	C[k-1] is the coefficient of sin(k·v) in the vertical modulator.
//
// Migration note: the original sphereish(u, v, a, b, c) used b·sin(2u),
// which corresponds to B = []float64{0, b} here (coefficient at index 1,
// not index 0).
type Modulators struct {
	A []float64
	B []float64
	C []float64
}

// LipschitzNorm returns Σ k·|coeffs[k-1]|.
func LipschitzNorm(coeffs []float64) float64 {
	s := 0.0
	for i, c := range coeffs {
		k := float64(i + 1)
		s += k * abs(c)
	}
	return s
}

// SafeBounds reports whether the modulators satisfy the sufficient
// non-self-intersection conditions (i.e., are well-formed).
func (m Modulators) SafeBounds() bool {
	return LipschitzNorm(m.A) < 0.5 &&
		LipschitzNorm(m.B) < 1.0 &&
		LipschitzNorm(m.C) < 0.5
}

// ScaleToFit reduces coeffs in place so that LipschitzNorm(coeffs) <= maxNorm.
// Returns the scale factor applied (1.0 if no scaling was needed).
// Use maxNorm = 0.49 for A and C arrays, maxNorm = 0.99 for B arrays.
func ScaleToFit(coeffs []float64, maxNorm float64) float64 {
	n := LipschitzNorm(coeffs)
	if n <= maxNorm {
		return 1.0
	}
	s := maxNorm / n
	for i := range coeffs {
		coeffs[i] *= s
	}
	return s
}

// sphereishGeneral evaluates S(u,v) for the generalized sphereish surface
// with arbitrary-length Fourier coefficient arrays. The trivial case
// (all arrays empty) reduces to the unit sphere.
func sphereishGeneral(u, v float64, m Modulators) geom.Vec {
	alpha, gamma := v/2, v/2
	for i, ak := range m.A {
		k := float64(i + 1)
		alpha += ak * sin(k*v)
	}
	for i, ck := range m.C {
		k := float64(i + 1)
		gamma -= ck * sin(k*v)
	}
	beta := 0.0
	for i, bk := range m.B {
		k := float64(i + 1)
		beta += bk * sin(k*u)
	}
	rho := sin(alpha)
	zeta := cos(gamma)
	return geom.Vec{
		rho * cos(u-beta),
		rho * sin(u+beta),
		zeta,
	}
}

// sphereishGeneralNormal returns the unit outward normal at (u,v) for the
// generalized sphereish surface. At the parametric poles (v ≈ 0 and v ≈ 2π)
// the analytic limits (0,0,+1) and (0,0,-1) are substituted to avoid the
// degeneracy in the cross product. This function gives the normal of the
// bare generalized sphereish only; it does NOT account for any radial
// displacement texture applied on top, so it is unsuitable for shading the
// displaced render mesh — keep using numerical differentiation through the
// full uv2xyz for that.
func sphereishGeneralNormal(u, v float64, m Modulators) geom.Dir {
	const epsilon = 1e-6
	if v < epsilon {
		return geom.Dir{0, 0, 1}
	}
	if 2*pi-v < epsilon {
		return geom.Dir{0, 0, -1}
	}
	alpha, alphaP := v/2, 0.5
	gamma, gammaP := v/2, 0.5
	for i, ak := range m.A {
		k := float64(i + 1)
		alpha += ak * sin(k*v)
		alphaP += k * ak * cos(k*v)
	}
	for i, ck := range m.C {
		k := float64(i + 1)
		gamma -= ck * sin(k*v)
		gammaP -= k * ck * cos(k*v)
	}
	rho := sin(alpha)
	rhoP := cos(alpha) * alphaP
	zetaP := -sin(gamma) * gammaP
	beta, betaP := 0.0, 0.0
	for i, bk := range m.B {
		k := float64(i + 1)
		beta += bk * sin(k*u)
		betaP += k * bk * cos(k*u)
	}
	phi := u - beta
	psi := u + beta
	phiP := 1 - betaP
	psiP := 1 + betaP
	suX := -rho * sin(phi) * phiP
	suY := rho * cos(psi) * psiP
	// suZ = 0
	svX := rhoP * cos(phi)
	svY := rhoP * sin(psi)
	svZ := zetaP
	// n = sv × su (outward), with suZ = 0
	nX := -svZ * suY
	nY := svZ * suX
	nZ := svX*suY - svY*suX
	mag := sqrt(nX*nX + nY*nY + nZ*nZ)
	if mag < 1e-12 {
		return geom.Dir{0, 0, 1}
	}
	return geom.Dir{nX / mag, nY / mag, nZ / mag}
}

// Cube approximation reference.
//
// To push sphereishGeneral toward an axis-aligned cube centered at the origin,
// use the arc-length-uniform parameterization (top face on v ∈ [0, π/2], sides
// on [π/2, 3π/2], bottom on [3π/2, 2π]). beta(u) becomes a triangle wave of
// period π and amplitude π/4; alpha(v) − v/2 and v/2 − gamma(v) are the same
// triangle wave of period 2π and amplitude π/4, so A and C coincide.
//
// Fourier expansions (only the listed indices are nonzero):
//
//	B (sin(k·u), k = 4n+2):
//	  B[1]  =  2/π     ≈  0.6366
//	  B[5]  = −2/(9π)  ≈ −0.0707
//	  B[9]  =  2/(25π) ≈  0.0255
//	  B[13] = −2/(49π) ≈ −0.0130
//
//	A and C (sin(k·v), odd k), with A[k−1] = C[k−1]:
//	  A[0]/C[0] =  2/π     ≈  0.6366
//	  A[2]/C[2] = −2/(9π)  ≈ −0.0707
//	  A[4]/C[4] =  2/(25π) ≈  0.0255
//	  A[6]/C[6] = −2/(49π) ≈ −0.0130
//
// Leading-order "rounded cube": A[0] = C[0] = 2/π and B = {0, 2/π}; everything
// else zero. Reads as a cube with softened edges.
//
// Caveats:
//   - A true cube is Lipschitz-divergent (Σ k·|coef| ~ harmonic), so SafeBounds
//     rejects it. Even the leading terms exceed the limits: L1(A) = 2/π ≈ 0.637
//     vs the 0.5 cap, and L1(B) = 2·(2/π) ≈ 1.273 vs the 1.0 cap. Running each
//     array through ScaleToFit (0.49 / 0.99 / 0.49) yields a cube-ish shape
//     with visibly rounded edges.
//   - v = 0 and v = 2π are degenerate poles, so the top/bottom face centers
//     always collapse to a point; those faces are only flat in the limit of
//     infinitely many A/C terms.

// defaultModulators returns the per-frame Modulators for series173,
// matching the original sphereish(u, v, a, b, c) shape. The original
// b·sin(2u) term migrates to B = []float64{0, b}, with B[0] left at zero.
// The well-formedness bounds are enforced via ScaleToFit.
func defaultModulators(t float64) Modulators {
	m := Modulators{
		A: []float64{sin(3*t) * .5},
		B: []float64{0, sin(5*t) * .5},
		C: []float64{sin(7*t) * .5},
	}
	ScaleToFit(m.A, 0.49)
	ScaleToFit(m.B, 0.99)
	ScaleToFit(m.C, 0.49)
	return m
}

func newModulators(t float64) Modulators {
	return Modulators{
		A: []float64{2/pi - cos(2*t)*.25 - .25, .25 * sin(7*t)},
		B: []float64{0, 2/pi - cos(3*t)*.25 - .25, .25 * sin(11*t)},
		C: []float64{2/pi - cos(5*t)*.25 - .25, .25 * sin(13*t)},
	}
}

func structureAlgorithm(a float64, b float64, c float64, d float64, e float64, x float64, y float64) float64 {
	return sin(b*x + c*y + sin(c*x-b*y+(a+d)*sin(x+(b+e)*sin(y))))
}

func slabOffsetFloat(yFloat, zFloat, t float64) float64 {
	a := pow(2, sin(2*t)*3-3)
	b := pow(2, sin(3*t)*3-3)
	c := pow(2, sin(5*t)*3-3)
	d := pow(2, sin(7*t)*3-3)
	e := pow(2, sin(11*t)*3-3)
	y := yFloat / 5 * 2 * pi
	z := zFloat / 30 * 2 * pi
	return 2 + 2*structureAlgorithm(e, d, c, b, a, z, y) - y
}

func slabOffset(yIndex int, zIndex int, t float64) float64 {
	return slabOffsetFloat(float64(yIndex), float64(zIndex), t)
}

// Per-instance mesh scales. The template maps these to scale axes as
// x=Depth, y=Height, z=Width (see the instance <scale> below).
func instanceWidth(t float64) float64  { return .3 + .15*cos(2*t) }
func instanceHeight(t float64) float64 { return .3 + .15*cos(3*t) }
func instanceDepth(t float64) float64  { return .3 + .15*cos(5*t) }

// centerTip returns the world-space +X tip of the center instance (the one at
// grid y=0, z=0). The instance scales the mesh by Depth along X, so the tip's
// unscaled X from uv2xyz must be multiplied by that same Depth before adding
// the instance's base offset.
func centerTip(t float64) geom.Vec {
	loc := uv2xyz(0, pi, t) // local +X tip of the mesh; Y,Z are 0 here
	return geom.Vec{slabOffset(0, 0, t) + instanceDepth(t)*loc.X, 0, 0}
}

func cameraPath(t float64) geom.Vec {
	/*
		loc := geom.Vec{3.0, cos(t+pi*3/4) + .666, sin(t+pi*3/4) + .666}
		return loc.Scaled((2.5 - cos(t)*.5) / loc.Len())
	*/
	tip := centerTip(t)
	return geom.Vec{tip.X + cos(2*t)*15, 15, sin(3*t) * 15}
}

func focusPath(t float64) geom.Vec {
	return centerTip(t)
}

func shape(u, v, t float64) geom.Vec {
	return sphereishGeneral(u, v, newModulators(t))
}

func uv2xyz(u, v, t float64) geom.Vec {
	loc := shape(u, v, t)
	const xMax = 1.0     // |loc.X| ceiling: rho = sin(alpha) is bounded by 1
	const stretch = 50.0 // local dx multiplier at x=0; total X length grows by 1+2(s-1)/3 = 10×
	xN := loc.X / xMax
	if xN > 1 {
		xN = 1
	} else if xN < -1 {
		xN = -1
	}
	loc.X = loc.X + (stretch-1)*xMax*(xN-xN*xN*xN/3)
	return loc
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
	globalFrameNumber = frameNumber
	t := float64(frameNumber) * dt
	envSize := int(pow(float64(desiredTriangles), .5))
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	distance := cameraLoc.Minus(focusPoint).Len()
	// Tangent-plane normal at the focus using only the primary -y term of
	// slabOffset (structureAlgorithm is local wobble around this global tilt).
	// ∂slab/∂y_world from the -y_rad term is -(2π/5), so the unnormalized
	// outward normal (1, -∂slab/∂y, 0) is (1, 2π/5, 0).
	nX, nY, nZ := 1.0, 2*pi/5, 0.0
	nMag := sqrt(nX*nX + nY*nY + nZ*nZ)
	nX, nY, nZ = nX/nMag, nY/nMag, nZ/nMag
	dx := cameraLoc.X - focusPoint.X
	dy := cameraLoc.Y - focusPoint.Y
	dz := cameraLoc.Z - focusPoint.Z
	cosPsi := (dx*nX + dy*nY + dz*nZ) / distance
	if cosPsi < 0 {
		cosPsi = -cosPsi
	}
	psi := math.Acos(cosPsi)
	maxCornerFOV := 180 - 2*psi*180/pi
	// Mitsuba sensor is 1200x675 (16:9) with fov_axis = "larger", so the fov
	// passed below is the horizontal edge FOV. For aspect W:H, the diagonal-to-
	// horizontal-edge conversion is tan(hFOV/2) = tan(diag/2) / sqrt(1+(H/W)^2).
	const filmW = 1200.0
	const filmH = 675.0
	diagToEdge := sqrt(1 + (filmH/filmW)*(filmH/filmW))
	maxFOV := 2 * atan(tan(maxCornerFOV*pi/180/2)/diagToEdge) * 180 / pi * .9
	fovWeight := sin(2*t)/2 + .5
	maxFOV = min(fovWeight*maxFOV+(1-fovWeight)*30, maxFOV)
	// maxFOV = min(maxFOV, 120)
	// maxFOV = 45
	fmt.Printf("\nnormal: (%v, %v, %v)\npsi(deg): %v\nmaxCornerFOV: %v\nmaxFOV: %v\ncameraLoc: %v\nfocusPoint: %v\ndistance: %v\nt: %#v\n", nX, nY, nZ, psi*180/pi, maxCornerFOV, maxFOV, cameraLoc, focusPoint, distance, t)
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
	radius := 0.0
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
				radius = math.Max(radius, vertex.Len())
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
	blendArray := []float32{}
	textureArray := []float32{}
	numFaces := 0
	for vIndex := startVIndex; vIndex < endVIndex; vIndex++ {
		for uIndex := startUIndex; uIndex < endUIndex; uIndex++ {
			u := float64(uIndex) / float64(nU) * 2 * pi
			v := float64(vIndex) / float64(nV) * 2 * pi
			loc := uv2xyz(u, v, t).Scaled(2 * pi)
			blendValue := float32(0)
			if (u > pi/4 && u < 3*pi/4 || u > 5*pi/4 && u < 7*pi/4) && (v > pi/2 && v < 3*pi/2) {
				blendValue = float32(1)
			}
			blendArray = append(blendArray, blendValue, blendValue, blendValue)
			textureValue := pow(spow(shapeTexture(.1, .75-cos(41*t)*.5, t, loc), .1)/2+.5, pow(10, sin(29*t)))
			if frameNumber%2 == 1 {
				textureValue = 1 - textureValue
			}
			textureArray = append(
				textureArray,
				float32(textureValue),
				float32(textureValue),
				float32(textureValue))
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
			v := float64(vIndex) / float64(envSize) * 2 * pi
			power := 2 * pow(10, sin(prime(12)*t)/2+.5)
			envmapValue := pow(sin(u/2), power*4) * pow(sin(v/2), power)
			envmapArray = append(
				envmapArray,
				float32(envmapValue),
				float32(envmapValue),
				float32(envmapValue))
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
	texturePath := fmt.Sprintf("data/%v.texture.rgbe", frameNumber)
	plyHeader, _ := os.Create(plyHeaderPath)
	mesh := MeshType{}
	mesh.NumVertices = numVerticies
	mesh.NumFaces = numFaces
	tmpl.Execute(plyHeader, mesh)
	envmap, _ := os.Create(envPath)
	rgbe.Encode(envmap, envSize, envSize, envmapArray)
	blend, _ := os.Create(blendPath)
	rgbe.Encode(blend, endUIndex-startUIndex, endVIndex-startVIndex, blendArray)
	texture, _ := os.Create(texturePath)
	rgbe.Encode(texture, endUIndex-startUIndex, endVIndex-startVIndex, textureArray)
	sensorFile, _ := os.Create("sensor.xml")

	type instance struct {
		Angle  float64
		Loc    geom.Vec
		Width  float64
		Height float64
		Depth  float64
	}
	type sensor struct {
		Camera        geom.Vec
		LookAt        geom.Vec
		Distance      float64
		Aperture      float64
		FogRadius     float64
		Angle         float64
		MinZ          float64
		IntIOR        float64
		ExtIOR        float64
		FOV           float64
		EnvX          float64
		EnvY          float64
		EnvZ          float64
		Abbe          float64
		FilmThickness float64
		FilmIOR       float64
		Roughness     float64
		Scale         float64
		G             float64
		Albedo        float64
		SigmaT        float64
		Instances     []instance
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
            <integer name="sample_count" value="2070"/>
        </sampler>
        <film type="hdrfilm" id="film">
            <integer name="width" value="3072"/>
            <integer name="height" value="1728"/>
            <rfilter type="gaussian"/>
        </film>
    </sensor>
    <emitter type="envmap" id="Area_002-light">
        <string name="filename" value="mitsuba.rgbe"/>
        <float name="scale" value="1"/>
        <transform name="to_world">
            <rotate value="1, 0, 0" angle="{{ .EnvX }}"/>
        </transform>
    </emitter>
  	<integrator type="nanscrub">
        <integrator type="volpathmis">
            <integer name="max_depth" value="16"/>
        </integrator>
	</integrator>
    <medium id="medium1" type="homogeneous">
        <float name="scale" value="{{ .Scale }}"/>
        <float name="albedo" value="{{ .Albedo }}"/>
        <float name="sigma_t" value="{{ .SigmaT }}"/>
        <phase type="hg">
			<float name="g" value="{{ .G }}"/>
		</phase>
    </medium>
<shape type="shapegroup" id="my_shape_group">
   <shape type="ply">
        <string name="filename" value="mitsuba.ply"/>
			<bsdf type="dielectric">
				<float name="film_thickness" value="{{ .FilmThickness }}"/>
				<float name="film_ior" value="{{ .FilmIOR }}"/>
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
    </shape>
</shape>

{{range .Instances}}<shape type="instance"><ref id="my_shape_group"/><transform name="to_world"><scale x="{{ .Depth }}" y="{{ .Height }}" z="{{ .Width }}"/><rotate value="1, 0, 0" angle="{{ .Angle }}"/><translate x="{{ .Loc.X }}" y="{{ .Loc.Y }}" z="{{ .Loc.Z }}"/></transform></shape>{{end}}
</scene>
`)
	angle := 90.0
	instances := []instance{}
	num := 49

	for z := -2 * num; z <= 2*num; z++ {
		for y := -num; y <= num; y++ {
			loc := geom.Vec{slabOffset(y, z, t), float64(y), float64(z)}
			angle := 0.0
			instances = append(instances, instance{angle, loc, instanceWidth(t), instanceHeight(t), instanceDepth(t)})
		}
	}
	fmt.Println(len(instances))

	intIor := 1.6 + sin(prime(14)*t)*.5
	abbe := pow(10, sin(prime(15)*t)/2+1.5)
	filmThickness := pow(10, sin(prime(16)*t)/2+2.5)
	filmIor := 1.8 + sin(prime(17)*t)*.7

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		distance,
		pow(10, cos(prime(18)*t)*3-3),
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		angle,
		minZ,
		intIor,
		sin(3*t) + 2,
		maxFOV,
		t / 2 / pi * 360,
		0,
		0,
		abbe,
		filmThickness,
		filmIor,
		pow(10, cos(17*t)*2-3),
		pow(10, sin(7*t)),
		cos(11*t) * .95,
		sin(prime(7)*t)*.25 + .25,
		pow(10, 3*sin(prime(8)*t)-1),
		instances})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 128, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
