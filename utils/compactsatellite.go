package utils

import (
	"math"
	"math/rand"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// CompactSatellite searches the per-level orbit radii of a nested torus knot (the
// (p,q)-tower in `levels`) to MINIMIZE ropelength = centerline length / tube
// radius -- the standard measure of how compact a knot is. The result is the
// densest constant-radius embedding of that knot type within the nested-torus
// family, with no self-intersection.
//
// Unlike TightenSatellite (which maximizes the geometric mean of the radii to
// keep every winding visually prominent), this targets density directly: short,
// fat rope. The family cannot collapse to a plain torus knot -- shrinking an
// inner orbit drives that winding's curvature up and its tube radius to zero, so
// ropelength blows up; a too-large orbit self-intersects. The minimum sits in
// between.
//
// samples controls MaxTubeRadius accuracy; pass 0 to auto-size it from the
// winding frequency (~600 x product of the q's, clamped to [4000, 12000]).
// Returns the orbit radii (one per level), the safe tube radius at the optimum
// (use ~0.9x for a visible gap), the achieved ropelength, and the composed
// centerline ready to sweep.
func CompactSatellite(levels []SatelliteLevel, samples int) (orbits []float64, rTube, ropelength float64, centerline func(float64) geom.Vec) {
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

	// ropelength of a candidate radius set (lower = more compact). +Inf if the
	// tube radius collapses (degenerate / self-touching configuration).
	rope := func(o []float64) (float64, float64) {
		path := ComposeSatellite(levels, o)
		rt, _, _ := MaxTubeRadius(path, 2*math.Pi, samples)
		if rt <= 1e-6 {
			return math.Inf(1), rt
		}
		return polyLengthClosed(path, samples) / rt, rt
	}

	// The ropelength landscape over the orbit radii is highly non-convex: between
	// the good basins are spikes where adjacent strands nearly touch and the tube
	// radius collapses (ropelength -> thousands). A single coordinate descent gets
	// trapped, so we run it from many starts (basin hopping) and keep the best.
	const lo, hi = 0.02, 0.95
	descend := func(start []float64) ([]float64, float64) {
		o := append([]float64(nil), start...)
		const steps = 20
		for pass := 0; pass < 4; pass++ {
			for k := 0; k < n; k++ {
				best, _ := rope(o)
				bestR := o[k]
				for i := 0; i <= steps; i++ {
					o[k] = lo + (hi-lo)*float64(i)/float64(steps)
					if s, _ := rope(o); s < best {
						best, bestR = s, o[k]
					}
				}
				o[k] = bestR
			}
		}
		// Fine local refinement around the coarse optimum.
		for k := 0; k < n; k++ {
			best, _ := rope(o)
			center, bestR := o[k], o[k]
			for d := -0.04; d <= 0.0401; d += 0.004 {
				r := center + d
				if r < 0.01 {
					continue
				}
				o[k] = r
				if s, _ := rope(o); s < best {
					best, bestR = s, r
				}
			}
			o[k] = bestR
		}
		s, _ := rope(o)
		return o, s
	}

	// Start from the geometric seed (each level ~1/3 of its parent) plus a batch
	// of random restarts. Fixed RNG seed -> deterministic result.
	seed := make([]float64, n)
	for k := range seed {
		seed[k] = 0.45 * math.Pow(0.33, float64(k))
	}
	orbits, bestScore := descend(seed)
	rng := rand.New(rand.NewSource(1))
	const restarts = 16
	for r := 0; r < restarts; r++ {
		start := make([]float64, n)
		for k := range start {
			start[k] = lo + (hi-lo)*rng.Float64()
		}
		o, s := descend(start)
		if s < bestScore {
			orbits, bestScore = o, s
		}
	}

	ropelength, rTube = rope(orbits)
	return orbits, rTube, ropelength, ComposeSatellite(levels, orbits)
}

// CompactSatellitePhased extends CompactSatellite with a per-level orbit PHASE.
// Letting each level's windings rotate relative to its parent lets them nestle
// into one another's gaps ("threading"), which can reach tighter configurations
// than the all-in-phase family CompactSatellite searches. It minimizes the same
// ropelength (length / tube radius) over BOTH the orbit radii and the inter-level
// phases; phases[0] (the outermost) is held at 0 since it is only a rigid
// rotation.
//
// samples controls MaxTubeRadius accuracy (0 auto-sizes as in CompactSatellite).
// Returns the orbit radii, the phases, the safe tube radius (use ~0.9x for a
// gap), the ropelength, and the composed centerline.
func CompactSatellitePhased(levels []SatelliteLevel, samples int) (orbits, phases []float64, rTube, ropelength float64, centerline func(float64) geom.Vec) {
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

	rope := func(o, ph []float64) (float64, float64) {
		path := ComposeSatellitePhased(levels, o, ph)
		rt, _, _ := MaxTubeRadius(path, 2*math.Pi, samples)
		if rt <= 1e-6 {
			return math.Inf(1), rt
		}
		return polyLengthClosed(path, samples) / rt, rt
	}

	const lo, hi = 0.02, 0.95
	// One climb = coordinate descent over orbit radii then inner phases, coarse
	// sweeps followed by fine refinement. o and ph are left at the local optimum.
	descend := func(o, ph []float64) float64 {
		const steps, phSteps = 20, 16
		best, _ := rope(o, ph)
		for pass := 0; pass < 3; pass++ {
			for k := 0; k < n; k++ {
				keep := o[k]
				for i := 0; i <= steps; i++ {
					o[k] = lo + (hi-lo)*float64(i)/float64(steps)
					if s, _ := rope(o, ph); s < best {
						best, keep = s, o[k]
					}
				}
				o[k] = keep
			}
			for k := 1; k < n; k++ { // phases[0] is a rigid rotation, leave at 0
				keep := ph[k]
				for i := 0; i < phSteps; i++ {
					ph[k] = 2 * math.Pi * float64(i) / float64(phSteps)
					if s, _ := rope(o, ph); s < best {
						best, keep = s, ph[k]
					}
				}
				ph[k] = keep
			}
		}
		for k := 0; k < n; k++ { // fine refine radii
			keep, c := o[k], o[k]
			for d := -0.04; d <= 0.0401; d += 0.004 {
				r := c + d
				if r < 0.01 {
					continue
				}
				o[k] = r
				if s, _ := rope(o, ph); s < best {
					best, keep = s, r
				}
			}
			o[k] = keep
		}
		for k := 1; k < n; k++ { // fine refine phases
			keep, c := ph[k], ph[k]
			for d := -0.2; d <= 0.2001; d += 0.02 {
				ph[k] = c + d
				if s, _ := rope(o, ph); s < best {
					best, keep = s, ph[k]
				}
			}
			ph[k] = keep
		}
		return best
	}

	// Seed (all in phase) plus random restarts over radii and phases. Fixed RNG
	// seed -> deterministic result.
	orbits = make([]float64, n)
	phases = make([]float64, n)
	for k := range orbits {
		orbits[k] = 0.45 * math.Pow(0.33, float64(k))
	}
	bestScore := descend(orbits, phases)
	rng := rand.New(rand.NewSource(2))
	const restarts = 16
	for r := 0; r < restarts; r++ {
		o := make([]float64, n)
		ph := make([]float64, n)
		for k := range o {
			o[k] = lo + (hi-lo)*rng.Float64()
			if k > 0 {
				ph[k] = 2 * math.Pi * rng.Float64()
			}
		}
		if s := descend(o, ph); s < bestScore {
			bestScore = s
			copy(orbits, o)
			copy(phases, ph)
		}
	}

	ropelength, rTube = rope(orbits, phases)
	return orbits, phases, rTube, ropelength, ComposeSatellitePhased(levels, orbits, phases)
}

// polyLengthClosed is the arc length of a closed 2*pi-periodic centerline,
// approximated by `samples` chords.
func polyLengthClosed(path func(float64) geom.Vec, samples int) float64 {
	h := 2 * math.Pi / float64(samples)
	prev := path(0)
	L := 0.0
	for i := 1; i <= samples; i++ {
		p := path(float64(i) * h)
		L += p.Minus(prev).Len()
		prev = p
	}
	return L
}

// VariableTubeRadius finds the linear radius profile r(p) = a + b*|p| (|p| =
// distance from the origin, which the nested-torus knots are centered on) that
// maximizes the rope volume swept around `centerline` with no self-intersection.
//
// A positive b makes the tube thinner near the crowded center and fatter toward
// the rim, packing more rope into the same knot shape than any single constant
// radius can. Returns a, b and the maximized volume; evaluate the profile as
// r := a + b*p.Len(). (In practice the gain over a constant radius is modest --
// clearance in these cables is fairly uniform -- but a non-zero b is nearly free.)
//
// Method: the non-self-intersection constraints are linear in (a, b) --
// r_i + r_j <= dist for each close-approach pair, and r_i <= 1/kappa_i for
// curvature -- so for any fixed slope b the largest feasible intercept a is just
// the min over those caps. Volume rises with a, so we take a = a_max(b) and scan
// b over a 1-D range.
func VariableTubeRadius(centerline func(float64) geom.Vec, samples int) (a, b, volume float64) {
	if samples <= 0 {
		samples = 8000
	}
	h := 2 * math.Pi / float64(samples)
	pts := make([]geom.Vec, samples)
	for i := range pts {
		pts[i] = centerline(float64(i) * h)
	}
	dist := func(i, j int) float64 { return pts[i].Minus(pts[j]).Len() }

	// Per-point arc-length weight, distance from origin, and curvature radius.
	ds := make([]float64, samples)
	rad := make([]float64, samples)   // |p_i|
	rCurv := make([]float64, samples) // 1/kappa_i (curvature pinch limit)
	for i := 0; i < samples; i++ {
		ds[i] = dist(i, (i+1)%samples)
		rad[i] = pts[i].Len()
		p0 := pts[(i-1+samples)%samples]
		p1 := pts[i]
		p2 := pts[(i+1)%samples]
		d1 := p2.Minus(p0).Scaled(1 / (2 * h))
		d2 := p2.Minus(p1.Scaled(2)).Plus(p0).Scaled(1 / (h * h))
		speed := d1.Len()
		rc := math.Inf(1)
		if speed > 0 {
			kappa := d1.Cross(d2).Len() / (speed * speed * speed)
			if kappa > 0 {
				rc = 1 / kappa
			}
		}
		rCurv[i] = rc
	}

	// Close-approach pairs: local minima of the distance matrix off the diagonal
	// (same criterion MaxTubeRadius uses for its separation bound).
	type pair struct {
		i, j int
		d    float64
	}
	var pairs []pair
	for i := 0; i < samples; i++ {
		for j := i + 1; j < samples; j++ {
			gap := j - i
			if gap > samples/2 {
				gap = samples - gap
			}
			if gap < 3 {
				continue
			}
			d := dist(i, j)
			if d < dist((i-1+samples)%samples, j) && d <= dist((i+1)%samples, j) &&
				d < dist(i, (j-1+samples)%samples) && d <= dist(i, (j+1)%samples) {
				pairs = append(pairs, pair{i, j, d})
			}
		}
	}

	pmin, pmax := math.Inf(1), 0.0
	for _, r := range rad {
		if r < pmin {
			pmin = r
		}
		if r > pmax {
			pmax = r
		}
	}
	_ = pmin

	// Largest feasible intercept for a given slope: min over all linear caps.
	aMaxFor := func(b float64) (float64, bool) {
		aMax := math.Inf(1)
		for _, pr := range pairs {
			cap := (pr.d - b*(rad[pr.i]+rad[pr.j])) / 2 // 2a + b(|pi|+|pj|) <= d
			if cap < aMax {
				aMax = cap
			}
		}
		for i := 0; i < samples; i++ {
			cap := rCurv[i] - b*rad[i] // a + b|pi| <= 1/kappa_i
			if cap < aMax {
				aMax = cap
			}
		}
		if math.IsInf(aMax, 1) {
			return 0, false
		}
		return aMax, true
	}
	vol := func(a, b float64) (float64, bool) {
		V := 0.0
		for i := 0; i < samples; i++ {
			r := a + b*rad[i]
			if r < 0 {
				return 0, false // radius can't go negative anywhere
			}
			V += math.Pi * r * r * ds[i]
		}
		return V, true
	}

	a0, ok := aMaxFor(0)
	if !ok {
		return 0, 0, 0
	}
	span := pmax - pmin
	if span <= 0 {
		span = 1
	}
	// Allow the radius to swing by up to ~2*a0 across the radial span either way.
	bMax := 2 * a0 / span
	const nb = 240
	volume = -1
	for bi := 0; bi <= nb; bi++ {
		bb := -bMax + 2*bMax*float64(bi)/float64(nb)
		am, ok := aMaxFor(bb)
		if !ok {
			continue
		}
		V, ok := vol(am, bb)
		if !ok {
			continue
		}
		if V > volume {
			volume, a, b = V, am, bb
		}
	}
	return a, b, volume
}
