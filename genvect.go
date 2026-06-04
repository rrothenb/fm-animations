// go:build ignore

package main

import (
	"fmt"
	"github.com/rrothenb/fm-animations/utils"
)

func main() {
	// (3,2)-cable of a (2,3) trefoil. The cable winds at radius 0.15; the inner
	// radius must sit in [0.35, 0.75] for a clean (non-self-intersecting) start
	// -- below that the windings collide, above ~0.80 the inner trefoil does.
	// 0.45 is comfortably interior.
	tref := utils.TorusKnot(1, 0.15, 3, 2, utils.TorusKnot(1, .45, 2, 3, utils.Circle))
	// A few thousand beads resolves the cable's fast windings; more is worse,
	// not better for ridgerunner -- extra beads add near-parallel struts that
	// ill-condition its tsnnls solver. Run with --EqOn (equilateralize) to keep
	// the constraint matrix well-conditioned and avoid the linear-algebra crash
	// that intrinsically threatens tightly-wound cables.
	beads := utils.SampleClosedPath(tref, 2500)
	if err := utils.WriteVECT("knot.vect", beads, true); err != nil {
		panic(err)
	}
	fmt.Printf("wrote knot.vect (%d verts, closed)\n", len(beads))
}
