package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// SatelliteLevel is one (p,q) winding in a nested torus-knot construction.
type SatelliteLevel struct {
	P, Q int
}

// ComposeSatellite builds the centerline of a satellite knot by winding each
// level on the previous one, starting from the unit Circle. levels[0] is the
// outermost winding (wound on the circle); the last level is the innermost curve
// that the visible tube is ultimately swept around. orbits[k] is the orbit
// radius of levels[k].
func ComposeSatellite(levels []SatelliteLevel, orbits []float64) func(float64) geom.Vec {
	path := Circle
	for k, lvl := range levels {
		path = TorusKnot(1, orbits[k], lvl.P, lvl.Q, path)
	}
	return path
}

// TightenSatellite searches the per-level orbit radii of a nested torus knot to
// maximize "tightness": the geometric mean of all orbit radii and the final tube
// radius, evaluated at the non-self-intersection limit. Maximizing the geometric
// mean (rather than the tube radius alone) keeps the windings prominent instead
// of collapsing them onto the base circle, which is what a fat-but-degenerate
// tube would do.
//
// It returns the chosen orbit radii (one per level), the maximum safe tube radius
// for that configuration (use ~0.85x of it for a visible gap), and the composed
// centerline ready to sweep.
//
// samples controls the curvature/separation accuracy inside MaxTubeRadius. Pass 0
// to auto-size it from the winding frequency (~600 x product of the q's, clamped
// to [4000, 12000]); nested knots oscillate fast, and undersampling makes the
// separation test MISS the tightest approach and over-report the safe radius.
// Cost is O(samples^2) per evaluation and there are ~70 evaluations, so this is a
// seconds-to-tens-of-seconds call, not something to put in a render loop.
func TightenSatellite(levels []SatelliteLevel, samples int) (orbits []float64, rTube float64, centerline func(float64) geom.Vec) {
	n := len(levels)
	if samples <= 0 {
		prodQ := 1
		for _, l := range levels {
			prodQ *= l.Q
		}
		samples = 600 * prodQ
		if samples < 4000 {
			samples = 4000
		}
		if samples > 12000 {
			samples = 12000
		}
	}

	orbits = make([]float64, n)
	for k := range orbits { // geometric seed: each level ~1/3 of its parent
		orbits[k] = 0.45 * math.Pow(0.33, float64(k))
	}

	score := func(o []float64) (float64, float64) {
		rt, _, _ := MaxTubeRadius(ComposeSatellite(levels, o), 2*math.Pi, samples)
		prod := rt
		for _, r := range o {
			prod *= r
		}
		return math.Pow(prod, 1/float64(n+1)), rt
	}

	// Coordinate ascent: each pass line-searches one orbit radius at a time.
	const lo, hi = 0.02, 0.55
	const steps = 16
	for pass := 0; pass < 3; pass++ {
		for k := 0; k < n; k++ {
			bestS, _ := score(orbits)
			bestR := orbits[k]
			for i := 0; i <= steps; i++ {
				orbits[k] = lo + (hi-lo)*float64(i)/float64(steps)
				if s, _ := score(orbits); s > bestS {
					bestS, bestR = s, orbits[k]
				}
			}
			orbits[k] = bestR
		}
	}
	// Fine local refinement around the coarse optimum.
	for k := 0; k < n; k++ {
		bestS, _ := score(orbits)
		center, bestR := orbits[k], orbits[k]
		for d := -0.04; d <= 0.0401; d += 0.005 {
			r := center + d
			if r < 0.01 {
				continue
			}
			orbits[k] = r
			if s, _ := score(orbits); s > bestS {
				bestS, bestR = s, r
			}
		}
		orbits[k] = bestR
	}

	_, rTube = score(orbits)
	return orbits, rTube, ComposeSatellite(levels, orbits)
}
