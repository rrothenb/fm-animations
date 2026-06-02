package utils

import "github.com/hunterloftis/pbr/pkg/geom"

// CalculatedSurface is an abstraction for a parametric surface with per-(u,v,t)
// radius and material. Relocated from the old top-level calculatedSurface.go;
// currently unreferenced but preserved for future use.
type CalculatedSurface interface {
	MaxRadius() float64
	MinRadius() float64
	Radius(u, v, t float64) float64
	Material(u, v, t float64) float64
	BasicShape(u, v, t float64) geom.Vec
}
