package utils

import (
	"math"
	"math/rand"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// ComposeSatellitePhased is ComposeSatellite with a per-level orbit phase. There
// must be one phase per level; phases[0] (the outermost) is only a rigid rotation
// and does not affect self-intersection.
func ComposeSatellitePhased(levels []SatelliteLevel, orbits, phases []float64) func(float64) geom.Vec {
	path := Circle
	for k, lvl := range levels {
		path = TorusKnotPhased(1, orbits[k], lvl.P, lvl.Q, phases[k], path)
	}
	return path
}

// TightenSatelliteGlobal does a basin-hopping global search over BOTH the
// per-level orbit radii and the inter-level phase offsets, with NO hierarchical
// cap on the radii. Where TightenSatellite is a smooth coordinate ascent confined
// to the tubular-neighborhood basin (inner winding inside its parent's clear
// tube), this can also land in disconnected "threading" islands: configurations
// where an inner winding LARGER than its parent's clear tube still avoids itself
// by passing through the gaps where the parent's tube would have self-collided.
//
// It maximizes the geometric mean of the orbit radii and the final tube radius,
// using `restarts` random initializations each followed by a coordinate-ascent
// climb. The outermost phase is held at 0 (rigid rotation only).
//
// samples=0 auto-sizes as in TightenSatellite; restarts<=0 defaults to 24. Each
// climb is many O(samples^2) MaxTubeRadius evaluations, so keep samples modest
// for the search and re-confirm the winner at high resolution.
func TightenSatelliteGlobal(levels []SatelliteLevel, samples, restarts int, seed int64) (orbits, phases []float64, rTube float64, centerline func(float64) geom.Vec) {
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
	if restarts <= 0 {
		restarts = 24
	}
	rng := rand.New(rand.NewSource(seed))

	score := func(o, ph []float64) float64 {
		rt, _, _ := MaxTubeRadius(ComposeSatellitePhased(levels, o, ph), 2*math.Pi, samples)
		prod := rt
		for _, r := range o {
			prod *= r
		}
		return math.Pow(prod, 1/float64(n+1))
	}

	const orbLo, orbHi = 0.03, 0.80
	const orbSteps, phSteps = 12, 12

	// One climb = a couple of coordinate-ascent sweeps (orbits, then inner phases).
	climb := func(o, ph []float64) float64 {
		best := score(o, ph)
		for pass := 0; pass < 2; pass++ {
			for k := 0; k < n; k++ {
				keep := o[k]
				for i := 0; i <= orbSteps; i++ {
					o[k] = orbLo + (orbHi-orbLo)*float64(i)/orbSteps
					if s := score(o, ph); s > best {
						best, keep = s, o[k]
					}
				}
				o[k] = keep
			}
			for k := 1; k < n; k++ { // phases[0] fixed at 0
				keep := ph[k]
				for i := 0; i < phSteps; i++ {
					ph[k] = 2 * math.Pi * float64(i) / phSteps
					if s := score(o, ph); s > best {
						best, keep = s, ph[k]
					}
				}
				ph[k] = keep
			}
		}
		return best
	}

	orbits = make([]float64, n)
	phases = make([]float64, n)
	best := -1.0
	for r := 0; r < restarts; r++ {
		o := make([]float64, n)
		ph := make([]float64, n)
		for k := range o {
			o[k] = orbLo + rng.Float64()*(orbHi-orbLo)
			if k > 0 {
				ph[k] = rng.Float64() * 2 * math.Pi
			}
		}
		if s := climb(o, ph); s > best {
			best = s
			copy(orbits, o)
			copy(phases, ph)
		}
	}

	rTube, _, _ = MaxTubeRadius(ComposeSatellitePhased(levels, orbits, phases), 2*math.Pi, samples)
	return orbits, phases, rTube, ComposeSatellitePhased(levels, orbits, phases)
}
