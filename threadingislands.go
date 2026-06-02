//go:build ignore

package main

import (
	"fmt"
	"math"

	"github.com/rrothenb/fm-animations/utils"
)

// Way-2 exploration: does an inner winding larger than its parent's clear tube
// ever thread cleanly through the gaps? Compares the hierarchical optimum (Way 1)
// against a global radii+phase search (Way 2), then prints an island map.
// Run: `go run threadingislands.go`.
func main() {
	levels := []utils.SatelliteLevel{{P: 3, Q: 5}, {P: 5, Q: 3}}

	o1, t1, way1 := utils.TightenSatellite(levels, 0)
	fmt.Printf("Way 1 (hierarchical): orbits=%.3f  tube<=%.4f\n", o1, t1)

	orbits, phases, t2, center := utils.TightenSatelliteGlobal(levels, 4000, 24, 1)
	hi, _, _ := utils.MaxTubeRadius(center, 2*math.Pi, 16000)
	fmt.Printf("Way 2 (global):       orbits=%.3f  phases=%.3f  tube<=%.4f (hi-res %.4f)\n\n",
		orbits, phases, t2, hi)

	// Island map: hold the outer orbit at the Way-1 value, scan inner orbit r1
	// (well past the clear-tube limit) against inner phase. Any sizable tube at
	// r1 beyond the hierarchical limit is a threading island.
	r0 := o1[0]
	nph := 12
	// Track the best cell that lies past the hierarchical clear-tube limit
	// (r1 >= 0.30): that is a genuine threading island, not the Way-1 basin.
	bestTube := 0.0
	var bestR1, bestPh float64
	fmt.Printf("island map at r0=%.3f  (rows r1, cols phase1)\n", r0)
	fmt.Printf("%7s", "r1\\ph")
	for j := 0; j < nph; j++ {
		fmt.Printf("  %5.2f", 2*math.Pi*float64(j)/float64(nph))
	}
	fmt.Println()
	for r1 := 0.10; r1 <= 0.801; r1 += 0.05 {
		fmt.Printf("%7.2f", r1)
		for j := 0; j < nph; j++ {
			ph := 2 * math.Pi * float64(j) / float64(nph)
			c := utils.ComposeSatellitePhased(levels, []float64{r0, r1}, []float64{0, ph})
			rt, _, _ := utils.MaxTubeRadius(c, 2*math.Pi, 4000)
			fmt.Printf("  %5.3f", rt)
			if r1 >= 0.30 && rt > bestTube {
				bestTube, bestR1, bestPh = rt, r1, ph
			}
		}
		fmt.Println()
	}

	// Emit meshes: the hierarchical optimum and the best threading island.
	if err := utils.WriteTubePLY("threading_way1.ply", way1, 0.9*t1, 48, 3000); err != nil {
		fmt.Println("PLY write failed:", err)
		return
	}
	fmt.Printf("\nwrote threading_way1.ply (Way 1: orbits=%.3f, tube=%.4f)\n", o1, 0.9*t1)

	island := utils.ComposeSatellitePhased(levels, []float64{r0, bestR1}, []float64{0, bestPh})
	if err := utils.WriteTubePLY("threading_island.ply", island, 0.9*bestTube, 48, 4000); err != nil {
		fmt.Println("PLY write failed:", err)
		return
	}
	fmt.Printf("wrote threading_island.ply (Way 2: r0=%.3f, r1=%.3f, phase=%.3f, tube=%.4f)\n",
		r0, bestR1, bestPh, 0.9*bestTube)
	fmt.Println("open either in 3dviewer.net")
}
