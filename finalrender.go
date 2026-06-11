//go:build ignore

package main

import (
	"flag"
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

// Final-quality render of a ridgerunner-tightened knot: ~targetVerts vertices
// with SQUARE faces. The smooth Lissajous centerline is captured near-losslessly
// by a K~N/4 Fourier fit, so we sweep the tube along fk.Path() at high nV.
// Square faces require nV/nU = ropelength/(2*pi); with nU*nV = targetVerts that
// gives nU = sqrt(targetVerts * 2*pi / ropelength).
func main() {
	in := flag.String("in", "knot.rr/knot.final.vect", "ridgerunner .vect to mesh")
	out := flag.String("out", "knot_789_final.ply", "output PLY path")
	target := flag.Float64("verts", 50e6, "approximate target vertex count")
	flag.Parse()

	beads, err := utils.ReadVECT(*in)
	if err != nil {
		panic(err)
	}
	fmt.Printf("read %d vertices from %s\n", len(beads), *in)

	K := len(beads) / 4
	if K < 15 {
		K = 15
	}
	fk := utils.FitFourier(beads, K)
	path := fk.Path()
	rMax, _, _ := utils.MaxTubeRadius(path, 2*math.Pi, 8000)
	L := pathLen(path, 12000)
	rho := L / rMax
	fmt.Printf("Fourier(%d) fit: length %.3f, tube %.4f, ropelength %.2f\n", K, L, rMax, rho)

	// Square faces: nV/nU = rho/(2pi), nU*nV = target.
	nU := int(math.Round(math.Sqrt(*target * 2 * math.Pi / rho)))
	if nU%2 == 1 {
		nU++ // even cross-section reads cleaner
	}
	nV := int(math.Round(float64(nU) * rho / (2 * math.Pi)))
	fmt.Printf("square-face mesh: nU=%d (around) nV=%d (along) -> %.2fM verts\n",
		nU, nV, float64((nU+1)*(nV+1))/1e6)

	// Aspect check: circumferential seg vs longitudinal seg (want ~1.0).
	segU := 2 * math.Pi * (0.95 * rMax) / float64(nU)
	segV := L / float64(nV)
	fmt.Printf("face aspect (segV/segU): %.4f\n", segV/segU)

	if err := utils.WriteTubePLY(*out, path, 0.95*rMax, nU, nV); err != nil {
		panic(err)
	}
	fmt.Printf("wrote %s\n", *out)
}
