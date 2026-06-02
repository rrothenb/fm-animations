package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// IdealInfo reports the before/after metrics of an ideal-knot relaxation.
type IdealInfo struct {
	N                       int
	LenBefore, LenAfter     float64
	ThickBefore, ThickAfter float64 // tube radius (reach); curvature/separation min
	RopeBefore, RopeAfter   float64 // length / thickness (scale-invariant)
}

func circumRadius(a, b, c geom.Vec) float64 {
	A := b.Minus(a).Len()
	B := c.Minus(b).Len()
	C := a.Minus(c).Len()
	area2 := b.Minus(a).Cross(c.Minus(a)).Len() // = 2 * triangle area
	if area2 < 1e-12 {
		return 1e18 // collinear: infinite radius
	}
	return A * B * C / (2 * area2)
}

func polyLength(b []geom.Vec) float64 {
	n := len(b)
	L := 0.0
	for i := 0; i < n; i++ {
		L += b[(i+1)%n].Minus(b[i]).Len()
	}
	return L
}

// polyThickness is the discrete reach of a closed bead polygon: the min of the
// local curvature radius and half the closest genuine strand approach (local
// minima of the distance matrix, away from the diagonal -- same criterion as
// MaxTubeRadius, so the two agree).
func polyThickness(b []geom.Vec) (thick, curvR, sepHalf float64) {
	n := len(b)
	curvR = 1e18
	for i := 0; i < n; i++ {
		if R := circumRadius(b[(i-1+n)%n], b[i], b[(i+1)%n]); R < curvR {
			curvR = R
		}
	}
	d := func(i, j int) float64 { return b[(i%n+n)%n].Minus(b[(j%n+n)%n]).Len() }
	minSep := 1e18
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			gap := j - i
			if gap > n-gap {
				gap = n - gap
			}
			if gap < 3 {
				continue
			}
			dd := d(i, j)
			if dd < d(i-1, j) && dd <= d(i+1, j) && dd < d(i, j-1) && dd <= d(i, j+1) {
				if dd < minSep {
					minSep = dd
				}
			}
		}
	}
	sepHalf = minSep / 2
	thick = curvR
	if sepHalf < thick {
		thick = sepHalf
	}
	return
}

func resampleClosed(pts []geom.Vec, N int) []geom.Vec {
	n := len(pts)
	cum := make([]float64, n+1)
	for i := 1; i <= n; i++ {
		cum[i] = cum[i-1] + pts[i%n].Minus(pts[i-1]).Len()
	}
	total := cum[n]
	out := make([]geom.Vec, N)
	j := 0
	for i := 0; i < N; i++ {
		target := total * float64(i) / float64(N)
		for j < n && cum[j+1] < target {
			j++
		}
		seg := cum[j+1] - cum[j]
		f := 0.0
		if seg > 0 {
			f = (target - cum[j]) / seg
		}
		out[i] = pts[j%n].Lerp(pts[(j+1)%n], f)
	}
	return out
}

func sampleClosed(path func(float64) geom.Vec, dense int) []geom.Vec {
	pts := make([]geom.Vec, dense)
	for i := 0; i < dense; i++ {
		pts[i] = path(2 * math.Pi * float64(i) / float64(dense))
	}
	return pts
}

func projectSeparation(b []geom.Vec, tau float64, skip, passes int) {
	n := len(b)
	disp := make([]geom.Vec, n)
	for p := 0; p < passes; p++ {
		for i := range disp {
			disp[i] = geom.Vec{}
		}
		any := false
		for i := 0; i < n; i++ {
			for j := i + 1; j < n; j++ {
				gap := j - i
				if gap > n-gap {
					gap = n - gap
				}
				if gap <= skip {
					continue
				}
				dv := b[j].Minus(b[i])
				dist := dv.Len()
				if dist < 2*tau && dist > 1e-9 {
					push := (2*tau - dist) * 0.5
					dir := dv.Scaled(1 / dist)
					disp[i] = disp[i].Minus(dir.Scaled(push))
					disp[j] = disp[j].Plus(dir.Scaled(push))
					any = true
				}
			}
		}
		for i := range b {
			b[i] = b[i].Plus(disp[i])
		}
		if !any {
			break
		}
	}
}

func projectCurvature(b []geom.Vec, tau float64) {
	n := len(b)
	for i := 0; i < n; i++ {
		a := b[(i-1+n)%n]
		p := b[i]
		c := b[(i+1)%n]
		if circumRadius(a, p, c) >= tau {
			continue
		}
		m := a.Plus(c).Scaled(0.5)
		h := p.Minus(m)
		hl := h.Len()
		if hl < 1e-9 {
			continue
		}
		half := c.Minus(a).Len() * 0.5
		if tau*tau < half*half {
			continue // neighbors already > 2tau apart; radius can't be the issue
		}
		st := tau - math.Sqrt(tau*tau-half*half) // sagitta giving radius tau
		if st < hl {
			b[i] = m.Plus(h.Scaled(st / hl))
		}
	}
}

// RelaxIdealKnot tightens a closed centerline toward its ideal (minimal-
// ropelength) shape. It samples `path` (period 2*pi) into N beads, normalizes the
// thickness (tube radius / reach) to 1, then runs curve-shortening flow with hard
// projection of the two thickness constraints -- adjacent-strand separation >= 2
// and local curvature radius >= 1 -- which are exactly what MaxTubeRadius
// measures. The thickness floor keeps strands from passing through each other, so
// the knot type is preserved while length (hence ropelength = length/thickness)
// shrinks.
//
// Returns the initial and relaxed bead polygons (both scaled to thickness 1) plus
// before/after metrics. step ~0.1 is stable; iters in the hundreds.
func RelaxIdealKnot(path func(float64) geom.Vec, N, iters int, step float64) (before, after []geom.Vec, info IdealInfo) {
	before = resampleClosed(sampleClosed(path, 8000), N)
	t0, _, _ := polyThickness(before)
	// Normalize so thickness == 1.
	scale := 1.0 / t0
	for i := range before {
		before[i] = before[i].Scaled(scale)
	}

	tau := 1.0
	after = make([]geom.Vec, N)
	copy(after, before)

	tmp := make([]geom.Vec, N)
	for it := 0; it < iters; it++ {
		// Curve-shortening (Laplacian) flow: pull each bead toward its neighbors'
		// midpoint. Shrinks length and smooths.
		for i := 0; i < N; i++ {
			a := after[(i-1+N)%N]
			c := after[(i+1)%N]
			mid := a.Plus(c).Scaled(0.5)
			tmp[i] = after[i].Plus(mid.Minus(after[i]).Scaled(step))
		}
		copy(after, tmp)

		projectCurvature(after, tau)

		L := polyLength(after)
		s := L / float64(N)
		// Skip same-strand neighbors out to arc pi*tau (a min-radius U-turn brings
		// beads that far apart legitimately); genuine contacts are farther.
		skip := int(math.Pi*tau/s) + 2
		projectSeparation(after, tau, skip, 8)

		if it%20 == 19 {
			after = resampleClosed(after, N)
		}
	}

	tb, _, _ := polyThickness(before)
	ta, _, _ := polyThickness(after)
	info = IdealInfo{
		N:           N,
		LenBefore:   polyLength(before),
		LenAfter:    polyLength(after),
		ThickBefore: tb,
		ThickAfter:  ta,
	}
	info.RopeBefore = info.LenBefore / info.ThickBefore
	info.RopeAfter = info.LenAfter / info.ThickAfter
	return before, after, info
}
