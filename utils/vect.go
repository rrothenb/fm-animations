package utils

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// SampleClosedPath samples a 2*pi-periodic centerline into N points spaced
// uniformly by arc length -- a good polygon to hand to ridgerunner.
func SampleClosedPath(path func(float64) geom.Vec, N int) []geom.Vec {
	return resampleClosed(sampleClosed(path, 8000), N)
}

func sampleClosed(path func(float64) geom.Vec, dense int) []geom.Vec {
	pts := make([]geom.Vec, dense)
	for i := 0; i < dense; i++ {
		pts[i] = path(2 * math.Pi * float64(i) / float64(dense))
	}
	return pts
}

// resampleClosed redistributes a closed polygon to N points uniform in arc length.
func resampleClosed(pts []geom.Vec, N int) []geom.Vec {
	n := len(pts)
	cum := make([]float64, n+1)
	for i := 1; i <= n; i++ {
		cum[i] = cum[i-1] + pts[i%n].Minus(pts[i-1]).Len()
	}
	total := cum[n]
	out := make([]geom.Vec, N)
	j := 0
	for i := 0; i < N; i++ {
		target := total * float64(i) / float64(N)
		for j < n && cum[j+1] < target {
			j++
		}
		seg := cum[j+1] - cum[j]
		f := 0.0
		if seg > 0 {
			f = (target - cum[j]) / seg
		}
		out[i] = pts[j%n].Lerp(pts[(j+1)%n], f)
	}
	return out
}

// WriteVECT writes a bead polygon as a Geomview VECT file, the format ridgerunner
// reads. A closed loop is signaled by a negative vertex count (Geomview
// convention) -- ridgerunner needs that or it tightens an open arc with free
// ends instead of a knot.
func WriteVECT(filename string, beads []geom.Vec, closed bool) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()

	n := len(beads)
	count := n
	if closed {
		count = -n
	}
	// VECT: 1 polyline, n vertices, 1 color.
	fmt.Fprintln(w, "VECT")
	fmt.Fprintf(w, "1 %d 1\n", n)
	fmt.Fprintf(w, "%d\n", count) // vertices in the (closed) polyline
	fmt.Fprintln(w, "1")          // one color spans the polyline
	for _, p := range beads {
		fmt.Fprintf(w, "%.10g %.10g %.10g\n", p.X, p.Y, p.Z)
	}
	fmt.Fprintln(w, "0 0 0 1") // color rgba
	return nil
}

// ReadVECT reads the vertices of the first polyline from a Geomview VECT file
// (e.g. ridgerunner output). Comments (#...) are ignored; the closed/open flag
// and colors are skipped -- only the vertex coordinates are returned.
func ReadVECT(filename string) ([]geom.Vec, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}
	// Tokenize, dropping '#' comments and the leading VECT keyword.
	var toks []string
	for _, line := range strings.Split(string(data), "\n") {
		if i := strings.IndexByte(line, '#'); i >= 0 {
			line = line[:i]
		}
		for _, t := range strings.Fields(line) {
			if t == "VECT" {
				continue
			}
			toks = append(toks, t)
		}
	}
	if len(toks) < 3 {
		return nil, fmt.Errorf("vect: header too short in %s", filename)
	}
	readInt := func(i int) (int, error) { return strconv.Atoi(toks[i]) }
	nPoly, err := readInt(0)
	if err != nil {
		return nil, fmt.Errorf("vect: bad polyline count: %w", err)
	}
	nVert, err := readInt(1)
	if err != nil {
		return nil, fmt.Errorf("vect: bad vertex count: %w", err)
	}
	nColor, _ := readInt(2)
	pos := 3
	// nPoly vertex-counts, then nPoly color-counts.
	vc := make([]int, nPoly)
	for p := 0; p < nPoly; p++ {
		v, err := readInt(pos)
		if err != nil {
			return nil, fmt.Errorf("vect: bad per-polyline vertex count: %w", err)
		}
		if v < 0 {
			v = -v
		}
		vc[p] = v
		pos++
	}
	pos += nPoly // skip color-counts
	_ = nColor

	// Vertices for the FIRST polyline.
	verts := make([]geom.Vec, 0, vc[0])
	for i := 0; i < vc[0]; i++ {
		if pos+2 >= len(toks) {
			return nil, fmt.Errorf("vect: ran out of coordinates (got %d of %d)", i, vc[0])
		}
		x, e1 := strconv.ParseFloat(toks[pos], 64)
		y, e2 := strconv.ParseFloat(toks[pos+1], 64)
		z, e3 := strconv.ParseFloat(toks[pos+2], 64)
		if e1 != nil || e2 != nil || e3 != nil {
			return nil, fmt.Errorf("vect: bad coordinate near token %d", pos)
		}
		verts = append(verts, geom.Vec{X: x, Y: y, Z: z})
		pos += 3
	}
	_ = nVert
	return verts, nil
}
