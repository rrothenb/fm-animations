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

// Knot placement after the stable-rest reorientation that is baked into the
// Mitsuba knot shape's to_world matrix (knotReorient in the template). These are
// WORLD coordinates. The orientation was found by taking the convex hull of the
// mesh's 5,010,378 vertices and choosing the facet it most stably rests on (the
// lowest-CoM / largest-base face whose center of mass projects well inside the
// ~3900-vertex contact region). That face is laid down on the canvas plane
// (X = surfaceX) so ~all those contact points share the minimum X, and the box
// is centered over the canvas at Y = Z = 0. The knot now sits flat on a real
// base instead of balancing on a single point.
var knotCenter = geom.Vec{1.0074020, 0, 0}
var knotBBoxMin = geom.Vec{-1.0, -2.9150110, -2.9087430}
var knotBBoxMax = geom.Vec{3.0148040, 2.9150110, 2.9087430}

// knotHull is a 320-point farthest-point sample of the knot's convex-hull
// vertices in WORLD coordinates (after the knotReorient transform). frameHalfAngles
// fits the FOV to these so it tracks the actual silhouette, not the looser box.
var knotHull = []geom.Vec{
	{-0.1978, -1.7637, 2.7516}, {2.3480, 1.6342, -2.8194}, {0.8534, -2.4531, -1.5883}, {-0.9511, 1.2681, -0.1487},
	{2.3084, 0.2421, 1.6670}, {0.3255, 2.8511, -1.7524}, {2.5708, 1.7375, -0.3267}, {1.1350, -2.4546, 0.8325},
	{-0.9358, -0.9439, -1.0774}, {1.0278, -0.3447, -2.6717}, {2.7248, -1.0849, -0.1528}, {0.5313, 1.4852, 1.2729},
	{-0.1612, 1.1970, -2.6449}, {0.5914, -0.0280, 2.4934}, {-0.4813, -2.5955, -0.1998}, {2.8193, -0.0577, -2.1678},
	{1.4892, -1.5198, 2.3741}, {-0.9999, -1.0563, 1.4343}, {1.0918, 2.6599, -0.8455}, {0.1990, 2.1943, -2.5772},
	{1.6266, 0.1210, 2.4512}, {0.3361, -2.8738, -0.7828}, {2.9688, 0.9365, -2.3771}, {0.6845, -1.3963, 2.8949},
	{-0.8312, -1.6706, 2.0929}, {-0.8871, 1.4768, -0.8488}, {2.2617, 1.6513, 0.2636}, {2.6844, 0.3949, -2.6408},
	{0.2432, -2.5466, -1.3487}, {0.9165, 2.8449, -1.4506}, {2.5124, 1.0012, -2.8597}, {-0.2690, -2.6567, -0.8146},
	{0.4191, -1.9141, 2.6425}, {-0.0452, 2.4757, -2.0823}, {-0.5786, -1.3299, 2.5256}, {1.9279, 0.4690, 2.0743},
	{-0.2125, 1.7817, -2.4827}, {0.9968, -1.7840, 2.6017}, {2.8896, -0.5324, -0.1361}, {0.6543, -0.6772, -2.4105},
	{-0.0173, -2.8876, -0.3313}, {0.7844, -2.7297, -1.1026}, {1.3414, -2.2971, -1.3770}, {2.7444, 1.4361, -2.4874},
	{2.4890, -1.3437, 0.2753}, {0.2195, 1.6843, 0.8627}, {1.0863, 0.2215, 2.5007}, {-0.9300, -1.1631, 1.9522},
	{3.0065, 0.4273, -2.2260}, {0.1480, 1.5940, -2.7925}, {0.1818, -1.4432, 2.8715}, {1.1976, -1.3471, 2.7534},
	{0.4047, 2.6400, -2.2131}, {-0.4874, -1.9191, 2.3835}, {2.8106, 1.3843, -0.5457}, {0.4760, 2.8388, -1.3033},
	{2.1066, 1.9542, -2.5867}, {-0.8080, 1.6161, -0.4229}, {2.6085, 1.5371, 0.0389}, {0.7922, -2.5059, 1.0562},
	{-0.1456, 2.1578, -2.3240}, {0.0484, -2.7581, -1.0582}, {-0.9629, -0.5438, -1.1539}, {2.1352, 0.1135, 2.0134},
	{2.8160, 0.7743, -2.7039}, {0.7970, 2.8090, -1.0611}, {1.2463, -0.0378, -2.7503}, {-0.9897, 1.2853, -0.5343},
	{1.0968, -2.5542, -1.2376}, {0.6964, 2.8510, -1.7557}, {2.5054, 1.7187, -2.4963}, {0.4256, -2.8042, -1.1466},
	{0.1839, 1.4375, 1.1600}, {0.0450, -1.9691, 2.5702}, {2.7070, -1.0566, 0.2092}, {-0.0224, -2.8969, -0.6977},
	{-0.8347, -1.3527, 2.2669}, {2.5869, 1.3529, -2.8199}, {1.6226, 0.4413, 2.2656}, {0.6944, -1.7454, 2.8019},
	{0.5916, -2.6430, -1.4222}, {2.8390, 1.1271, -2.6509}, {-0.3250, -2.7827, -0.4794}, {-0.2844, -1.4202, 2.7323},
	{-0.5249, -1.6631, 2.6209}, {1.1264, 2.7483, -1.1822}, {0.0410, 1.9066, -2.6851}, {0.9170, -0.0546, 2.6177},
	{2.2581, 1.3068, -2.9063}, {2.8414, 0.1791, -2.4256}, {0.1466, 2.4617, -2.3647}, {1.9067, 0.1697, 2.2667},
	{0.1321, 1.2555, -2.8017}, {2.0264, 1.7325, -2.8310}, {0.1663, -1.7731, 2.8192}, {2.8417, -0.8135, 0.0289},
	{2.7502, 1.4085, -0.2255}, {0.1589, 2.7133, -1.9970}, {-0.1456, 1.5296, -2.6724}, {0.9574, -1.5515, 2.8323},
	{1.2753, -1.6137, 2.5900}, {-0.9387, -1.4621, 1.8483}, {1.3781, 0.3254, 2.4271}, {2.9541, 0.5703, -2.5014},
	{-0.7416, -1.6282, 2.4027}, {2.6785, 1.6669, -0.6076}, {1.3479, 0.0493, 2.5632}, {0.9153, -0.6091, -2.5515},
	{-0.2534, -2.8026, -0.1628}, {0.4341, -1.6229, 2.8930}, {0.5104, 2.7933, -1.9803}, {2.1556, 0.3874, 1.9015},
	{-0.9897, -1.2363, 1.6623}, {0.4525, 1.6623, 1.0345}, {1.1310, -2.3607, -1.5629}, {1.1425, -2.4894, 0.5473},
	{-0.2369, -1.9576, 2.5253}, {1.4287, -1.3376, 2.5803}, {0.8701, -2.6405, -1.3770}, {2.4058, 1.7239, 0.0362},
	{2.6104, -1.2574, 0.0282}, {0.6500, 2.9142, -1.4912}, {3.0092, 0.6888, -2.2730}, {-0.0706, -1.5502, 2.8342},
	{0.2990, 2.8085, -1.4933}, {-0.8181, 1.4986, -0.1928}, {0.3288, 1.3520, 1.3502}, {1.2797, -0.3160, -2.6814},
	{2.5923, 1.5618, -2.6757}, {2.3216, 1.8273, -2.6313}, {2.9455, 0.1726, -2.1971}, {-0.0061, 1.0105, -2.7093},
	{-0.7999, -0.9250, -1.2849}, {-0.0399, 2.0941, -2.5392}, {0.9177, -2.5600, 0.8120}, {2.1363, 1.5160, -2.8964},
	{0.5258, -2.8218, -0.9279}, {-0.6751, -1.8208, 2.2703}, {0.1253, 2.7187, -1.7556}, {-0.9185, 1.5195, -0.6156},
	{-0.9979, -0.7116, -0.9923}, {2.7353, 1.5789, -0.3867}, {2.5915, 0.7886, -2.8005}, {0.7222, 2.8854, -1.2704},
	{0.1802, -2.8983, -0.5828}, {-0.4565, -2.6001, -0.4367}, {0.2510, -2.8502, -1.0051}, {2.8790, 1.1937, -2.4374},
	{-0.9164, -1.4573, 2.0740}, {0.0103, 2.2887, -2.4351}, {1.0210, -2.4643, 1.0275}, {0.1729, 2.6142, -2.2011},
	{-0.9510, 1.4155, -0.3550}, {2.7226, 0.9776, -2.7866}, {-0.0894, -2.7950, -0.8875}, {1.1317, 0.0027, 2.6075},
	{0.2153, -2.7261, -1.2015}, {1.5928, -1.3251, 2.3545}, {2.6624, 1.6041, -0.1624}, {-0.9211, -0.7569, -1.2186},
	{2.6832, 0.5965, -2.7227}, {0.4651, -1.4078, 2.8965}, {2.2397, 0.0959, 1.8237}, {2.0481, 0.2894, 2.1061},
	{-0.2205, -2.8138, -0.6716}, {0.4264, -2.5109, -1.4650}, {0.9397, 2.8435, -1.2339}, {2.9154, 0.9362, -2.5871},
	{2.8330, -0.8959, -0.1690}, {1.0343, -2.5212, -1.4604}, {2.7435, 1.3274, -2.6787}, {-0.3811, -1.8186, 2.6324},
	{0.6321, -2.4809, -1.5527}, {-0.0998, 1.7503, -2.6597}, {0.6249, -2.7531, -1.2442}, {-0.1167, -2.6309, -1.0188},
	{1.7344, 0.2825, 2.3494}, {0.5091, 2.9039, -1.6485}, {2.8713, 0.3779, -2.5260}, {0.7925, 0.0845, 2.5207},
	{-0.0543, 1.3423, -2.7526}, {0.4072, -2.6856, -1.3310}, {0.7969, -1.8399, 2.6341}, {1.0084, 2.7657, -1.0093},
	{2.4482, 1.1983, -2.8914}, {-0.3760, -2.6378, -0.6354}, {-0.8422, 1.2861, 0.0290}, {-0.0811, 2.3408, -2.2462},
	{1.1449, -1.5536, 2.7425}, {-0.0074, -1.8526, 2.7389}, {-0.4774, -1.4641, 2.6538}, {0.0403, -2.5926, -1.1822},
	{0.1180, -2.8857, -0.8483}, {-0.3502, -1.6270, 2.7331}, {-0.1741, 1.9815, -2.4321}, {2.4128, 1.4474, -2.8741},
	{-0.7340, -1.4268, 2.4305}, {-0.9994, 1.2061, -0.3478}, {1.1645, -0.4863, -2.6231}, {0.2623, 1.5983, 1.0573},
	{-0.6146, -1.7686, 2.4730}, {0.4826, -1.7943, 2.7983}, {0.2203, -1.9117, 2.6830}, {-0.8539, -1.1948, 2.1362},
	{-0.1260, -2.8921, -0.5117}, {0.2703, 2.3520, -2.4779}, {2.6444, 1.1560, -2.8263}, {1.2062, -2.4426, -1.3755},
	{2.7807, -1.0187, 0.0252}, {2.5007, 1.7406, -0.1432}, {0.9986, -1.3599, 2.8362}, {-0.3835, -2.7381, -0.2936},
	{0.8927, -1.7201, 2.7578}, {2.9843, 0.7564, -2.4563}, {2.2092, 1.7708, -2.7783}, {-0.8459, -1.5492, 2.2424},
	{0.7512, -1.5764, 2.8786}, {-0.2169, 1.3725, -2.5868}, {0.6085, -1.8798, 2.6478}, {0.9365, -2.6563, -1.1970},
	{0.3368, 2.7546, -2.0559}, {1.8651, 0.3550, 2.2291}, {0.3376, 2.5169, -2.3437}, {-0.9494, 1.3454, -0.7245},
	{0.3520, 1.5127, 1.2190}, {0.8029, 2.8621, -1.6023}, {0.1896, 2.0281, -2.6620}, {2.6166, -1.2218, 0.2138},
	{2.7033, 0.2322, -2.5510}, {0.1566, 1.7767, -2.7529}, {-0.2186, -2.8538, -0.3388}, {2.4612, 1.6971, -2.6755},
	{1.1429, -0.1972, -2.7218}, {-0.2308, 1.6020, -2.5222}, {2.7950, 0.0248, -2.3311}, {0.2447, -1.6161, 2.8857},
	{2.8830, -0.7231, -0.1279}, {2.0078, 1.8832, -2.7271}, {-0.6256, -1.5237, 2.5563}, {1.5593, 0.3119, 2.4042},
	{0.0137, 2.5883, -1.9509}, {2.4253, 1.6237, 0.1869}, {-0.7141, -1.2558, 2.3609}, {2.9553, 0.3011, -2.3449},
	{1.0145, -2.5541, 0.6564}, {2.8424, 0.5878, -2.6389}, {-0.9749, -1.2883, 1.8319}, {2.6366, 1.5792, -2.5055},
	{0.7356, -0.0717, 2.5853}, {1.2657, 0.2201, 2.5134}, {1.3196, -1.4536, 2.6563}, {-0.1042, 1.9125, -2.5851},
	{1.4516, 0.1787, 2.5034}, {0.5334, 2.8652, -1.8203}, {1.3565, -1.6325, 2.4359}, {0.4613, 2.8888, -1.4809},
	{0.0316, 2.4900, -2.2374}, {0.0849, 1.4317, -2.8023}, {0.9716, 0.1076, 2.5610}, {2.7765, 1.5296, -0.6428},
	{0.0101, -1.7023, 2.8310}, {-0.0208, 1.6191, -2.7471}, {0.6261, -2.7904, -1.0620}, {1.1654, 2.6999, -1.0016},
	{1.7696, 0.4788, 2.1627}, {0.7733, -2.5732, -1.4993}, {2.0250, 0.1260, 2.1538}, {1.7814, 0.1082, 2.3622},
	{2.2526, 1.9090, -2.5005}, {0.5896, -1.5409, 2.9042}, {0.8477, -1.4293, 2.8784}, {1.1630, -1.7272, 2.5414},
	{-0.1083, -1.9294, 2.6270}, {2.2420, 0.2656, 1.8299}, {0.2535, 2.7964, -1.8907}, {0.0371, -2.9117, -0.5025},
	{-0.0125, 1.1835, -2.7554}, {0.3227, -1.8286, 2.7801}, {1.4180, -1.5052, 2.5199}, {0.2168, 2.7832, -1.6308},
	{0.5649, 1.5903, 1.1260}, {2.7869, 1.4104, -0.3829}, {1.0436, -1.6784, 2.7179}, {0.9262, -0.4564, -2.6147},
	{-0.7818, -1.6965, 2.2540}, {3.0028, 0.5232, -2.3554}, {-0.9436, -1.3195, 1.9972}, {0.7790, -2.7133, -1.2615},
	{2.8177, 1.2971, -2.5409}, {-0.1070, -2.8489, -0.1945}, {0.7673, -0.5947, -2.5062}, {0.3323, -1.4894, 2.8995},
	{0.9869, -2.3660, -1.6184}, {-0.8583, 1.3869, -0.0939}, {-0.2079, -1.6151, 2.7919}, {2.7783, -0.9195, 0.1433},
	{0.0913, -1.5667, 2.8699}, {2.6924, 1.4744, -0.0873}, {2.9186, 0.7278, -2.5908}, {-0.3877, -1.9130, 2.5130},
	{2.3833, 1.8221, -2.4924}, {-0.8981, -1.5612, 1.9647}, {2.4824, 1.5552, -2.7928}, {0.0644, 2.6017, -2.0957},
	{2.6442, 1.6976, -0.4532}, {2.5384, 1.6625, -0.0086}, {2.9242, 1.0587, -2.4862}, {0.5714, -1.6842, 2.8628},
	{1.5116, -1.3939, 2.4604}, {-0.9737, -1.1458, 1.7864}, {2.7003, -1.1460, 0.0902}, {-0.9976, -1.1102, 1.5727},
	{2.0581, 0.4120, 2.0108}, {0.3771, -2.8611, -0.9286}, {2.5361, -1.3288, 0.1338}, {2.1438, 0.2649, 1.9973},
	{0.5897, 2.8418, -1.2108}, {-0.1480, -1.4348, 2.7876}, {0.3943, 2.8367, -1.8926}, {-0.8668, 1.5825, -0.7372},
}

const surfaceX = -1.0   // canvas plane: paper.ply lies at X = surfaceX
const canvasHalf = 10.0 // canvas (paper.ply scaled 10) spans Y,Z in ±canvasHalf

// cameraUp is the camera's up vector; it must match the sensor's look-at up in
// the Mitsuba template below. World +X is up (the canvas surface normal).
var cameraUp = geom.Vec{1, 0, 0}

// Film dimensions (must match the <film> block) and a hair of framing margin.
const filmW = 1200.0
const filmH = 900.0
const fovMargin = 1.02

// cameraAt places a camera orbiting the knot center around the +X (up) axis at
// distance D, azimuth az in the Y-Z plane, and elevation el above the canvas.
func cameraAt(az, el, D float64) geom.Vec {
	return knotCenter.Plus(geom.Vec{
		D * sin(el),
		D * cos(el) * cos(az),
		D * cos(el) * sin(az),
	})
}

// cameraBasis returns the orthonormal camera frame (forward, image-right,
// image-up) for a camera at P aimed at the knot center with cameraUp as up.
func cameraBasis(P geom.Vec) (fwd, right, up geom.Vec) {
	fdir, _ := knotCenter.Minus(P).Unit()
	fwd = geom.Vec(fdir)
	rdir, _ := cameraUp.Cross(fwd).Unit()
	right = geom.Vec(rdir)
	udir, _ := fwd.Cross(right).Unit()
	up = geom.Vec(udir)
	return
}

// frameHalfAngles returns the half-FOV (radians) along the image horizontal and
// vertical axes needed to just frame the knot from P. It projects the knot's
// convex-hull silhouette (knotHull) into the camera frame and fits the tightest
// field of view that contains it, coupling the two axes through the film aspect
// ratio (fov_axis="larger" => Mitsuba is given the horizontal/wider half-angle).
// Fitting the hull instead of the bounding box is what keeps the FOV tight: the
// box corners poke into empty space, so the roundish knot never fills the box.
func frameHalfAngles(P geom.Vec) (hHalf, vHalf float64) {
	fwd, right, up := cameraBasis(P)
	maxTanH, maxTanV := 0.0, 0.0
	for _, p := range knotHull {
		d := p.Minus(P)
		df := d.Dot(fwd)
		if df <= 0 {
			continue // point behind the camera
		}
		maxTanH = max(maxTanH, abs(d.Dot(right))/df)
		maxTanV = max(maxTanV, abs(d.Dot(up))/df)
	}
	tanH := max(maxTanH, maxTanV*filmW/filmH) * fovMargin
	return atan(tanH), atan(tanH * filmH / filmW)
}

// maxFrameGroundRadius marches the frustum's edge rays down to the canvas plane
// (X = surfaceX) and returns the farthest any of them lands from the canvas
// center. It returns +Inf if any edge ray points at or above the horizon, which
// would expose the canvas edge against the background. Keeping this below
// canvasHalf guarantees the whole frame lands on the canvas, so its edge is
// never visible.
func maxFrameGroundRadius(P geom.Vec) float64 {
	fwd, right, up := cameraBasis(P)
	hHalf, vHalf := frameHalfAngles(P)
	th, tv := tan(hHalf), tan(vHalf)
	worst := 0.0
	for _, sh := range []float64{-1, 0, 1} {
		for _, sv := range []float64{-1, 0, 1} {
			if sh == 0 && sv == 0 {
				continue
			}
			d := fwd.Plus(right.Scaled(th * sh)).Plus(up.Scaled(tv * sv))
			if d.X >= -1e-9 {
				return math.Inf(1) // ray not heading down toward the canvas
			}
			s := (surfaceX - P.X) / d.X
			hit := P.Plus(d.Scaled(s))
			worst = max(worst, sqrt(hit.Y*hit.Y+hit.Z*hit.Z))
		}
	}
	return worst
}

// wallMatrix builds the Mitsuba `to_world` matrix (16 row-major values) for a
// vertical backdrop that stands behind the knot from the camera's point of view,
// so the camera never sees the constant-emitter environment directly. The wall:
//   - is perpendicular to the canvas (its plane contains the up axis, +X),
//   - faces the camera (its normal is horizontal, pointing back at the camera),
//   - stands fully past the table, at ~2x the object->table-edge distance,
//   - dips its bottom edge below the table so nothing peeks under it (the table
//     occludes the wall's lower part), and so never touches the table,
//   - is sized (from camera pos, focus, FOV and aspect) to be just slightly
//     larger than the rendered view window at the wall's depth.
//
// Mitsuba's `rectangle` is the [-1,1]^2 plane in local XY with a +Z normal, so
// the matrix maps local X -> right*halfW, local Y -> up*halfH, local Z -> normal.
func wallMatrix(cameraLoc geom.Vec, maxFOV float64) string {
	const margin = 0.04               // wall is this fraction larger than the view
	const aspect = filmW / filmH      // sensor uses fov_axis="larger" (the horizontal)
	const belowTable = surfaceX - 0.1 // force the bottom edge below the canvas

	// Horizontal direction from the camera toward the knot center (drop the up
	// axis so the result lies in the canvas plane).
	h := geom.Vec{0, knotCenter.Y - cameraLoc.Y, knotCenter.Z - cameraLoc.Z}
	hdir, ok := h.Unit()
	if !ok {
		hdir = geom.Dir{X: 0, Y: 1, Z: 0} // camera straight overhead: any horizontal
	}
	hd := geom.Vec(hdir)

	// Stand the wall fully past the table: 2x the object->table-edge distance,
	// measured along hd through the square canvas of half-size canvasHalf.
	wallDist := 2 * canvasHalf / max(abs(hd.Y), abs(hd.Z))

	up := cameraUp          // (1,0,0)
	normal := hd.Scaled(-1) // the wall faces back toward the camera
	rdir, _ := up.Cross(normal).Unit()
	r := geom.Vec(rdir)
	plane := hd.Scaled(wallDist) // a point on the vertical wall plane

	// Project the four view-frustum corner rays onto the wall plane and bound them
	// in the wall's (right, up) coordinates, so the wall just covers the rendered
	// view (camera position, focus direction, FOV and aspect ratio).
	fwd, right, upCam := cameraBasis(cameraLoc)
	tanH := tan(maxFOV * pi / 180 / 2)
	tanV := tanH / aspect
	minLx, maxLx := math.Inf(1), math.Inf(-1)
	minLy, maxLy := math.Inf(1), math.Inf(-1)
	for _, sh := range []float64{-1, 1} {
		for _, sv := range []float64{-1, 1} {
			d := fwd.Plus(right.Scaled(tanH * sh)).Plus(upCam.Scaled(tanV * sv))
			denom := d.Dot(normal)
			if abs(denom) < 1e-9 {
				continue
			}
			hit := cameraLoc.Plus(d.Scaled(plane.Minus(cameraLoc).Dot(normal) / denom))
			rel := hit.Minus(plane)
			lx, ly := rel.Dot(r), rel.Dot(up)
			minLx, maxLx = min(minLx, lx), max(maxLx, lx)
			minLy, maxLy = min(minLy, ly), max(maxLy, ly)
		}
	}

	// Slight margin all around, then force the bottom edge below the table.
	dx, dy := maxLx-minLx, maxLy-minLy
	minLx, maxLx = minLx-dx*margin, maxLx+dx*margin
	minLy, maxLy = minLy-dy*margin, maxLy+dy*margin
	if minLy > belowTable {
		minLy = belowTable
	}

	halfW, halfH := (maxLx-minLx)/2, (maxLy-minLy)/2
	center := plane.Plus(r.Scaled((minLx + maxLx) / 2)).Plus(up.Scaled((minLy + maxLy) / 2))

	// Row-major 4x4 whose columns are R*halfW, up*halfH, normal, and the center.
	return fmt.Sprintf("%g %g %g %g %g %g %g %g %g %g %g %g 0 0 0 1",
		r.X*halfW, up.X*halfH, normal.X, center.X,
		r.Y*halfW, up.Y*halfH, normal.Y, center.Y,
		r.Z*halfW, up.Z*halfH, normal.Z, center.Z)
}

func cameraPath(t float64) geom.Vec {
	return geom.Vec{cos(5*t)*2 + 1.5, 2 + sin(3*t)*6, 1 + cos(3*t)*6}
}

func focusPath(t float64) geom.Vec {
	return geom.Vec{0, 0, 1}
}

// lightElevation is how far the orbiting distant light sits above the table
// horizon. ~0 is exactly table height (pure grazing rays); a few degrees keeps
// the table top lit while staying strongly oblique.
var lightElevation = 12 * pi / 180

// lightDirection returns the travel (incoming) direction of the orbiting distant
// light: a parallel "sun" near table height that revolves around the up (+X)
// axis once per animation, so over the sequence the knot is raked from every
// oblique azimuth. The vector points FROM the light TOWARD the scene (the
// convention Mitsuba's directional "direction" expects).
func lightDirection(t float64) geom.Vec {
	az := t // orbit azimuth in the table plane (Y-Z)
	return geom.Vec{
		-sin(lightElevation),
		-cos(lightElevation) * cos(az),
		-cos(lightElevation) * sin(az),
	}
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
	envSize := int(pow(float64(desiredTriangles), .5)) * 5
	cameraLoc := cameraPath(t)
	focusPoint := focusPath(t)
	c := NewSLR2().MoveTo(cameraLoc).LookAt(focusPoint)
	distance := cameraLoc.Minus(focusPoint).Len()
	maxFOV := 35.0
	fmt.Printf("\nfocus(center): %v\ncameraLoc: %v\ndistance: %v\nFOV(deg): %v\ngroundR: %v\nt: %#v\n", focusPoint, cameraLoc, distance, maxFOV, maxFrameGroundRadius(cameraLoc), t)
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
		Up            geom.Vec
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
		LightDir      geom.Vec
		Abbe          float64
		FilmThickness float64
		FilmIOR       float64
		Roughness     float64
		Scale         float64
		G             float64
		Albedo        float64
		SigmaT        float64
		Illumination  float64
		WallMatrix    string
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
            <lookat origin="{{ .Camera.X }}, {{ .Camera.Y }}, {{ .Camera.Z }}" target="{{ .LookAt.X }}, {{ .LookAt.Y }}, {{ .LookAt.Z }}" up="{{ .Up.X }}, {{ .Up.Y }}, {{ .Up.Z }}"/>
        </transform>
        <sampler type="multijitter">
            <integer name="sample_count" value="240"/>
        </sampler>
        <film type="hdrfilm" id="film">
            <integer name="width" value="900"/>
            <integer name="height" value="1200"/>
            <rfilter type="gaussian"/>
        </film>
    </sensor>
	<shape type="sphere">
		<emitter type="area">
			<rgb name="radiance" value="1.0"/>
		</emitter>
		<transform name="to_world">
			<scale value=".24"/>
			<translate x="0" y="0" z="1"/>
	</transform>
	</shape>
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
<shape type="shapegroup" id="my_shape_group">
   <shape type="ply">
        <string name="filename" value="knot_789_final.ply"/>
			<bsdf type="roughdielectric">
			    <float name="alpha" value="{{ .Roughness }}"/>
				<float name="film_thickness" value="{{ .FilmThickness }}"/>
				<float name="film_ior" value="{{ .FilmIOR }}"/>
				<float name="int_ior" value="{{ .IntIOR }}"/>
				<float name="abbe" value="{{ .Abbe }}"/>
			</bsdf>
         <ref id="medium1" name="interior"/>
   </shape>
</shape>
<shape type="ply">
	<string name="filename" value="knot_345_seed_thin.ply"/>
	<bsdf type="conductor">
		<float name="film_thickness" value="{{ .FilmThickness }}"/>
		<float name="film_ior" value="{{ .FilmIOR }}"/>
	</bsdf>
	<transform name="to_world">
		<scale value="1.5"/>
		<translate x="0" y="0" z="1"/>
	</transform>
</shape>
{{range .Instances}}<shape type="instance"><ref id="my_shape_group"/><transform name="to_world"><scale x="{{ .Depth }}" y="{{ .Height }}" z="{{ .Width }}"/><translate x="{{ .Loc.X }}" y="{{ .Loc.Y }}" z="{{ .Loc.Z }}"/></transform></shape>{{end}}
</scene>
`)
	angle := 90.0
	instances := []instance{}
	num := 0

	for z := -2 * num; z <= 2*num; z++ {
		for y := -num; y <= num; y++ {
			loc := geom.Vec{0, float64(y), float64(z)}
			angle := 0.0
			instances = append(instances, instance{angle, loc, 1, 1, 1})
		}
	}
	fmt.Println(len(instances))

	intIor := 1.9 + sin(prime(14)*t)*.8
	extIor := 1.0
	if frameNumber%2 == 1 {
		extIor = intIor
		intIor = 1
	}
	abbe := pow(10, sin(prime(15)*t)/2+1.5)
	filmThickness := pow(10, sin(prime(16)*t)/2+2.5)
	filmIor := 1.8 + sin(prime(17)*t)*.7

	sensorTemplate.Execute(sensorFile, sensor{
		cameraLoc,
		focusPoint,
		cameraUp,
		distance,
		.00000000001,
		focusPoint.Minus(cameraLoc).Scaled(.5).Len(),
		angle,
		minZ,
		intIor,
		extIor,
		maxFOV,
		sin(2*t) * 175,
		sin(3*t) * 175,
		sin(5*t) * 175,
		lightDirection(t),
		abbe,
		filmThickness,
		filmIor,
		pow(10, cos(17*t)*2-3),
		pow(10, sin(7*t)*2-2),
		cos(11*t) * .95,
		sin(prime(7)*t)*.5 + .5,
		pow(10, 2*sin(prime(8)*t)-1),
		cos(7*t)*.15 + .15,
		wallMatrix(cameraLoc, maxFOV),
		instances})
}

func main() {
	frame := flag.Int("frame", 0, "Specify frame")
	pixels := flag.Int("pixels", 256, "Specify height and width of generated image")
	maxSubdivisions := flag.Int("maxsubdivisions", 1000, "Max subdivisions")
	maxFrames := flag.Int("maxframes", 256, "Max frames")
	desiredTriangles := flag.Int("desiredtriangles", 0, "The desired number of triangles to render")
	flag.Parse()
	fmt.Printf("frame: %v, pixels: %v, maxSubdivisions: %v, maxFrames: %v\n", *frame, *pixels, *maxSubdivisions, *maxFrames)
	dt := pi * 2 / float64(*maxFrames)
	renderSurfaces(*frame, *pixels, *maxSubdivisions, dt, *desiredTriangles)
}
