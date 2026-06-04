package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// MaxTubeRadius estimates the largest tube radius that can be swept around the
// closed centerline `path` (as pathWrapper does) without the resulting surface
// self-intersecting. The curve is assumed closed over [0, period); for the
// torusKnot family that period is always 2*pi.
//
// It returns the overall safe radius and the two bounds it is the min of:
//
//	rCurv = 1/kappa_max  -- the tight-bend / pinch limit (local curvature)
//	rSep  = dcsd/2       -- the adjacent-strand merge limit (global approach)
//
// Use a value somewhat below rMax (e.g. 0.85*rMax) to leave a visible gap.
// `samples` controls accuracy; fast-winding (high-crossing) curves need tens of
// thousands -- the separation test uses a spatial grid so that stays ~O(n).
func MaxTubeRadius(path func(float64) geom.Vec, period float64, samples int) (rMax, rCurv, rSep float64) {
	h := period / float64(samples)
	pts := make([]geom.Vec, samples)
	for i := range pts {
		pts[i] = path(float64(i) * h)
	}
	rCurv = maxCurvatureRadius(pts, h)
	rSep = minStrutSeparation(pts) / 2
	rMax = rCurv
	if rSep < rMax {
		rMax = rSep
	}
	return
}

// maxCurvatureRadius returns 1/kappa_max over the closed sample polygon, with
// kappa = |C' x C''| / |C'|^3 from central differences (spacing h).
func maxCurvatureRadius(pts []geom.Vec, h float64) float64 {
	n := len(pts)
	maxKappa := 0.0
	for i := 0; i < n; i++ {
		p0 := pts[(i-1+n)%n]
		p1 := pts[i]
		p2 := pts[(i+1)%n]
		d1 := p2.Minus(p0).Scaled(1 / (2 * h))
		d2 := p2.Minus(p1.Scaled(2)).Plus(p0).Scaled(1 / (h * h))
		speed := d1.Len()
		if speed == 0 {
			continue
		}
		kappa := d1.Cross(d2).Len() / (speed * speed * speed)
		if kappa > maxKappa {
			maxKappa = kappa
		}
	}
	if maxKappa == 0 {
		return 1e18
	}
	return 1 / maxKappa
}

// isStrut reports whether (i,j) is a non-adjacent local minimum of the distance
// function -- i.e. a genuine strand-approach ("strut"), not along-curve bending.
// The 3-bead along-curve neighborhood is skipped; the test matches the criterion
// the old O(n^2) scan used.
func isStrut(pts []geom.Vec, i, j int) bool {
	n := len(pts)
	g := j - i
	if g < 0 {
		g = -g
	}
	if g > n/2 {
		g = n - g
	}
	if g < 3 {
		return false
	}
	d := func(a, b int) float64 { return pts[a].Minus(pts[b]).Len() }
	dij := d(i, j)
	return dij < d((i-1+n)%n, j) && dij <= d((i+1)%n, j) &&
		dij < d(i, (j-1+n)%n) && dij <= d(i, (j+1)%n)
}

// minStrutSeparation returns the smallest strut distance (doubly-critical
// self-distance) of the closed sample polygon -- the global nearest approach of
// the curve to a non-adjacent part of itself. A uniform spatial grid keeps this
// ~O(n) instead of O(n^2); the cell size is grown adaptively until the minimum
// found sits comfortably inside the search radius (so nothing closer was missed).
// Returns +Inf if the curve has no self-approach (e.g. a convex curve), in which
// case thickness is purely curvature-limited.
func minStrutSeparation(pts []geom.Vec) float64 {
	n := len(pts)
	if n < 6 {
		return math.Inf(1)
	}
	lo, hi := pts[0], pts[0]
	meanEdge := 0.0
	for i := 0; i < n; i++ {
		p := pts[i]
		if p.X < lo.X {
			lo.X = p.X
		}
		if p.Y < lo.Y {
			lo.Y = p.Y
		}
		if p.Z < lo.Z {
			lo.Z = p.Z
		}
		if p.X > hi.X {
			hi.X = p.X
		}
		if p.Y > hi.Y {
			hi.Y = p.Y
		}
		if p.Z > hi.Z {
			hi.Z = p.Z
		}
		meanEdge += pts[(i+1)%n].Minus(p).Len()
	}
	meanEdge /= float64(n)
	diag := hi.Minus(lo).Len()
	if diag == 0 {
		return math.Inf(1)
	}

	c := 8 * meanEdge
	if c <= 0 || c > diag {
		c = diag
	}
	for {
		best := gridMinStrut(pts, lo, c)
		if best < 0.7*c || c >= diag {
			return best
		}
		c *= 2
		if c > diag {
			c = diag
		}
	}
}

// gridMinStrut buckets the points into a uniform grid of cell size c and finds
// the smallest strut distance among point pairs within c (each point compares
// only against its own and 26 neighboring cells). Cell size c >= the true
// minimum guarantees both endpoints of the closest strut land in adjacent cells.
func gridMinStrut(pts []geom.Vec, lo geom.Vec, c float64) float64 {
	type key [3]int
	cellOf := func(p geom.Vec) key {
		return key{
			int(math.Floor((p.X - lo.X) / c)),
			int(math.Floor((p.Y - lo.Y) / c)),
			int(math.Floor((p.Z - lo.Z) / c)),
		}
	}
	grid := make(map[key][]int32, len(pts))
	for i, p := range pts {
		k := cellOf(p)
		grid[k] = append(grid[k], int32(i))
	}
	best := math.Inf(1)
	for i, p := range pts {
		ck := cellOf(p)
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				for dz := -1; dz <= 1; dz++ {
					for _, jj := range grid[key{ck[0] + dx, ck[1] + dy, ck[2] + dz}] {
						j := int(jj)
						if j <= i {
							continue
						}
						d := pts[i].Minus(pts[j]).Len()
						if d <= c && d < best && isStrut(pts, i, j) {
							best = d
						}
					}
				}
			}
		}
	}
	return best
}
