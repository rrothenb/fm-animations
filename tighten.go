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

	orbits, rTube, _ := utils.TightenSatellite(levels, 0)

	fmt.Println("tightest configuration:")
	for k, lvl := range levels {
		fmt.Printf("  level %d  (%d,%d)  orbit r = %.4f\n", k, lvl.P, lvl.Q, orbits[k])
	}
	fmt.Printf("  tube radius <= %.4f  (use ~%.4f for a visible gap)\n", rTube, 0.85*rTube)
}
