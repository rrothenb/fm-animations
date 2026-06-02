package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// Circle is the unit circle in the XY plane, the default companion path for a
// plain torus knot.
func Circle(x float64) geom.Vec {
	return geom.Vec{X: math.Sin(x), Y: math.Cos(x), Z: 0}
}
