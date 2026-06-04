//go:build ignore

package main

import (
	"fmt"
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/rrothenb/fm-animations/utils"
)

func pathLen(p func(float64) geom.Vec, samples int) float64 {
	prev := p(0)
	L := 0.0
	for i := 1; i <= samples; i++ {
		q := p(2 * math.Pi * float64(i) / float64(samples))
		L += q.Minus(prev).Len()
		prev = q
	}
	return L
}

// Read a ridgerunner-tightened curve back in and mesh it. The mesh is built
// directly on the ridgerunner beads (WriteBeadTubePLY), so it is shape-agnostic:
// it works for satellite/cable knots, not just torus knots. (A truncated Fourier
// fit would low-pass away the cable windings -- K=15 only suffices for a trefoil.)
func main() {
	beads, err := utils.ReadVECT("knot.rr/knot.final.vect")
	if err != nil {
		panic(err)
	}
	fmt.Printf("read %d vertices from ridgerunner output\n", len(beads))

	// Size the tube from the true centerline. Fit enough harmonics to be
	// near-lossless (K ~ N/4) -- this is only for MEASURING radius/length, not
	// for the mesh geometry, so over-resolving is safe.
	K := len(beads) / 4
	if K < 15 {
		K = 15
	}
	fk := utils.FitFourier(beads, K)
	path := fk.Path()
	rMax, _, _ := utils.MaxTubeRadius(path, 2*math.Pi, 8000)
	L := pathLen(path, 12000)
	fmt.Printf("Fourier(%d) fit: length %.3f, tube %.4f, ropelength %.2f\n",
		K, L, rMax, L/rMax)

	// Mesh the raw ridgerunner beads directly -- exact geometry, any knot type.
	if err := utils.WriteBeadTubePLY("knot.ply", beads, 0.95*rMax, 48); err != nil {
		panic(err)
	}
	fmt.Println("wrote knot.ply (open in 3dviewer.net)")
}
