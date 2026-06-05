package utils

import (
	"github.com/hunterloftis/pbr/pkg/geom"
	"math"
)

func LissajousKnot(xN, yN, zN int) func(float64) geom.Vec {
	return func(t float64) geom.Vec {
		return geom.Vec{math.Sin(float64(xN) * t), math.Sin(float64(yN) * t), math.Cos(float64(zN) * t)}
	}
}
