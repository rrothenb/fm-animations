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

// Read a ridgerunner-tightened curve back in, turn it into a smooth analytic
// centerline (Fourier fit -> pathWrapper-ready), and mesh it.
func main() {
	beads, err := utils.ReadVECT("trefoil.rr/trefoil.final.vect")
	if err != nil {
		panic(err)
	}
	fmt.Printf("read %d vertices from ridgerunner output\n", len(beads))

	// Fourier-fit -> infinitely smooth path with clean tangents for pathWrapper.
	fk := utils.FitFourier(beads, 15)
	path := fk.Path()

	rMax, _, _ := utils.MaxTubeRadius(path, 2*math.Pi, 8000)
	L := pathLen(path, 12000)
	fmt.Printf("Fourier(15) trefoil: length %.3f, tube %.4f, ropelength %.2f (ideal ~32.74)\n",
		L, rMax, L/rMax)

	if err := utils.WriteTubePLY("trefoil_ridge.ply", path, 0.95*rMax, 48, 2000); err != nil {
		panic(err)
	}
	fmt.Println("wrote trefoil_ridge.ply (open in 3dviewer.net)")
}
