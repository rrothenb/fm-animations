//go:build ignore

package main

import (
	"fmt"

	"github.com/rrothenb/fm-animations/utils"
)

// Relax the algebraic (3,5),(5,3) cable toward its ideal (minimal-ropelength)
// shape and write before/after tube meshes. Run: `go run idealrelax.go`.
func main() {
	levels := []utils.SatelliteLevel{{P: 3, Q: 5}, {P: 5, Q: 3}}
	orbits, _, _ := utils.TightenSatellite(levels, 0)
	path := utils.ComposeSatellite(levels, orbits)

	before, after, info := utils.RelaxIdealKnot(path, 1200, 500, 0.1)

	fmt.Printf("beads=%d\n", info.N)
	fmt.Printf("           length    thickness   ropelength(L/thick)\n")
	fmt.Printf("algebraic  %8.2f   %.4f      %8.2f\n", info.LenBefore, info.ThickBefore, info.RopeBefore)
	fmt.Printf("relaxed    %8.2f   %.4f      %8.2f\n", info.LenAfter, info.ThickAfter, info.RopeAfter)
	fmt.Printf("ropelength reduction: %.1f%%\n", 100*(1-info.RopeAfter/info.RopeBefore))

	r := 0.95 // thickness is normalized to 1; leave a small visible gap
	if err := utils.WriteBeadTubePLY("ideal_before.ply", before, r, 32); err != nil {
		fmt.Println("PLY write failed:", err)
		return
	}
	if err := utils.WriteBeadTubePLY("ideal_after.ply", after, r, 32); err != nil {
		fmt.Println("PLY write failed:", err)
		return
	}
	fmt.Println("wrote ideal_before.ply and ideal_after.ply (open in 3dviewer.net)")
}
