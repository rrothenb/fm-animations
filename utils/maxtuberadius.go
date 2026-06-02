package utils

import "github.com/hunterloftis/pbr/pkg/geom"

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
// `samples` controls accuracy; 4000 is plenty for visual work.
func MaxTubeRadius(path func(float64) geom.Vec, period float64, samples int) (rMax, rCurv, rSep float64) {
	h := period / float64(samples)
	pts := make([]geom.Vec, samples)
	for i := range pts {
		pts[i] = path(float64(i) * h)
	}

	// Curvature bound: kappa = |C' x C''| / |C'|^3 via central differences.
	maxKappa := 0.0
	for i := 0; i < samples; i++ {
		p0 := pts[(i-1+samples)%samples]
		p1 := pts[i]
		p2 := pts[(i+1)%samples]
		d1 := p2.Minus(p0).Scaled(1 / (2 * h))                   // C'
		d2 := p2.Minus(p1.Scaled(2)).Plus(p0).Scaled(1 / (h * h)) // C''
		speed := d1.Len()
		if speed == 0 {
			continue
		}
		kappa := d1.Cross(d2).Len() / (speed * speed * speed)
		if kappa > maxKappa {
			maxKappa = kappa
		}
	}
	rCurv = 1e18
	if maxKappa > 0 {
		rCurv = 1 / maxKappa
	}

	// Separation bound: smallest distance between non-adjacent strands, found as
	// the smallest local minimum of the distance matrix away from the diagonal.
	dist := func(i, j int) float64 { return pts[i].Minus(pts[j]).Len() }
	minSep := 1e18
	for i := 0; i < samples; i++ {
		for j := 0; j < samples; j++ {
			gap := j - i
			if gap < 0 {
				gap = -gap
			}
			if gap > samples/2 {
				gap = samples - gap
			}
			if gap < 3 { // along-curve neighborhood is the curvature regime
				continue
			}
			d := dist(i, j)
			if d < dist((i-1+samples)%samples, j) && d <= dist((i+1)%samples, j) &&
				d < dist(i, (j-1+samples)%samples) && d <= dist(i, (j+1)%samples) {
				if d < minSep {
					minSep = d
				}
			}
		}
	}
	rSep = minSep / 2

	rMax = rCurv
	if rSep < rMax {
		rMax = rSep
	}
	return
}
