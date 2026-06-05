// go:build ignore

package main

import (
	"fmt"
	"github.com/rrothenb/fm-animations/utils"
)

func main() {
	beads := utils.SampleClosedPath(utils.LissajousKnot(11, 12, 13), 4600)
	if err := utils.WriteVECT("knot.vect", beads, true); err != nil {
		panic(err)
	}
	fmt.Printf("wrote knot.vect (%d verts, closed)\n", len(beads))
}
