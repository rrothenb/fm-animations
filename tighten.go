//go:build ignore

package main

import (
	"fmt"

	"github.com/rrothenb/fm-animations/utils"
)

// Standalone helper: print the tightest orbit + tube radii for a nested
// (satellite/cable) torus knot. Activate with the one-char space toggle on the
// build line, or just `go run tighten.go`.
//
// Edit `levels` for the knot you want; levels[0] is wound on the unit circle.
func main() {
	levels := []utils.SatelliteLevel{
		{P: 3, Q: 5},
		{P: 5, Q: 3},
	}

	orbits, rTube, centerline := utils.TightenSatellite(levels, 0)

	fmt.Println("tightest configuration:")
	for k, lvl := range levels {
		fmt.Printf("  level %d  (%d,%d)  orbit r = %.4f\n", k, lvl.P, lvl.Q, orbits[k])
	}
	tube := 0.9 * rTube
	fmt.Printf("  tube radius <= %.4f  (using %.4f for the mesh)\n", rTube, tube)

	const out = "tighten.ply"
	if err := utils.WriteTubePLY(out, centerline, tube, 48, 3000); err != nil {
		fmt.Println("PLY write failed:", err)
		return
	}
	fmt.Printf("wrote %s (open in 3dviewer.net)\n", out)
}
