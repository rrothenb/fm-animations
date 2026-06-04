//go:build ignore

package main

import (
	"fmt"

	"github.com/hunterloftis/pbr/pkg/geom"
	"github.com/rrothenb/fm-animations/utils"
)

// Compact satellite-knot demo: given a (p,q)-tower, find the orbit radii that
// minimize ropelength (most compact, no self-intersection), then fit a linear
// radius profile r = a + b*|p| for extra packing density. Writes one PLY for the
// constant-radius solution and one for the variable-radius solution.
func main() {
	levels := []utils.SatelliteLevel{{P: 2, Q: 3}, {P: 3, Q: 2}} // (2,3),(3,2) cable

	orbits, rTube, rope, path := utils.CompactSatellite(levels, 0)
	fmt.Printf("compact %v: orbits=%.4f  tube=%.4f  ropelength=%.2f\n",
		levels, orbits, rTube, rope)

	// Constant-radius mesh, 0.9x the safe radius for a visible gap.
	if err := utils.WriteTubePLY("compact_const.ply", path, 0.9*rTube, 48, 4000); err != nil {
		panic(err)
	}
	fmt.Println("wrote compact_const.ply")

	// Variable-radius profile r = a + b*|p| that maximizes packed volume.
	a, b, varVol := utils.VariableTubeRadius(path, 8000)
	// Constant-radius packed volume = pi*rTube^2*L, and L = ropelength*rTube.
	constVol := 3.141592653589793 * rope * rTube * rTube * rTube
	fmt.Printf("variable profile r = %.4f + %.4f*|p|  -> volume +%.1f%% vs best constant\n",
		a, b, 100*(varVol/constVol-1))

	radius := func(p geom.Vec) float64 { return 0.9 * (a + b*p.Len()) }
	if err := utils.WriteVarTubePLY("compact_var.ply", path, radius, 48, 4000); err != nil {
		panic(err)
	}
	fmt.Println("wrote compact_var.ply")

	// Phase-aware search: let each level's windings rotate to thread the gaps.
	po, pph, prTube, prope, ppath := utils.CompactSatellitePhased(levels, 0)
	fmt.Printf("\nphased %v: orbits=%.4f  phases=%.4f  tube=%.4f  ropelength=%.2f  (%+.1f%% vs in-phase)\n",
		levels, po, pph, prTube, prope, 100*(prope/rope-1))
	if err := utils.WriteTubePLY("compact_phased.ply", ppath, 0.9*prTube, 48, 4000); err != nil {
		panic(err)
	}
	fmt.Println("wrote compact_phased.ply")
}
