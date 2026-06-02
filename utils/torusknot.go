package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// TorusKnot returns the centerline of a (p,q) torus knot of orbit radius r wound
// around the companion curve `path` (scaled by R). It is the canonical
// swept-frame construction shared by the series mains: the companion is advanced
// at q*t, a Frenet-ish frame is built from its tangent, and the knot orbits that
// frame at p*t. Nest it (path = another TorusKnot) to build cable/satellite
// knots.
//
// The frame uses z as the reference "up"; if the tangent is parallel to z (which
// would make the ring degenerate to NaN) it falls back to x, matching the guard
// in pathWrapper.
func TorusKnot(R, r float64, p, q int, path func(float64) geom.Vec) func(float64) geom.Vec {
	pf := float64(p)
	qf := float64(q)
	const delta = .01
	return func(t float64) geom.Vec {
		center := path(qf * t).Scaled(R)
		normal, _ := path(qf*t+delta).Scaled(R).Minus(path(qf*t-delta).Scaled(R)).Unit()
		sinVec, ok := normal.Cross(geom.Dir{X: 0, Y: 0, Z: 1})
		if !ok { // tangent parallel to z: avoid 0/0 -> NaN ring
			sinVec, _ = normal.Cross(geom.Dir{X: 1, Y: 0, Z: 0})
		}
		cosVec, _ := normal.Cross(sinVec)
		return cosVec.Scaled(r * math.Cos(pf*t)).Plus(sinVec.Scaled(r * math.Sin(pf*t))).Plus(center)
	}
}
