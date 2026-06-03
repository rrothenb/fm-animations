//go:build ignore

package main

import (
	"fmt"

	"github.com/rrothenb/fm-animations/utils"
)

func main() {
	tref := utils.TorusKnot(1, 0.5, 2, 3, utils.Circle) // (2,3) torus = trefoil
	beads := utils.SampleClosedPath(tref, 300)
	if err := utils.WriteVECT("trefoil.vect", beads, true); err != nil {
		panic(err)
	}
	fmt.Printf("wrote trefoil.vect (%d verts, closed)\n", len(beads))
}
