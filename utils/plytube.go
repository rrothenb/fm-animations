package utils

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"math"
	"os"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// tubePoint sweeps a plain circular cross-section of radius r around `path`: u is
// the angle around the ring, v the position along the centerline. It is the
// artistic-term-free version of the series' pathWrapper, so the mesh shows the
// solution's true geometry.
func tubePoint(path func(float64) geom.Vec, r, u, v float64) geom.Vec {
	const delta = .01
	center := path(v)
	normal, _ := path(v+delta).Minus(path(v - delta)).Unit()
	sinVec, ok := normal.Cross(geom.Dir{X: 0, Y: 0, Z: 1})
	if !ok { // tangent parallel to z: avoid 0/0 -> NaN ring
		sinVec, _ = normal.Cross(geom.Dir{X: 1, Y: 0, Z: 0})
	}
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * math.Cos(u)).Plus(sinVec.Scaled(r * math.Sin(u))).Plus(center)
}

// WriteTubePLY sweeps a tube of radius `tubeRadius` around the closed centerline
// `path` (period 2*pi) and writes it as a binary little-endian PLY that opens
// directly in 3dviewer.net. nU is the cross-section resolution (ring around the
// tube), nV the resolution along the centerline. Nested knots wiggle fast, so nV
// usually needs to be in the low thousands to look smooth.
func WriteTubePLY(filename string, path func(float64) geom.Vec, tubeRadius float64, nU, nV int) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()

	const twoPi = 2 * math.Pi
	const du, dv = 0.001, 0.001
	type vtx struct{ px, py, pz, nx, ny, nz float32 }
	verts := make([]vtx, 0, (nU+1)*(nV+1))
	for ui := 0; ui <= nU; ui++ {
		u := twoPi * float64(ui) / float64(nU)
		for vi := 0; vi <= nV; vi++ {
			v := twoPi * float64(vi) / float64(nV)
			p := tubePoint(path, tubeRadius, u, v)
			pu := tubePoint(path, tubeRadius, u+du, v)
			pv := tubePoint(path, tubeRadius, u, v+dv)
			n, _ := pu.Minus(p).Cross(pv.Minus(p)).Unit()
			// Orient the normal outward (away from the centerline).
			c := path(v)
			if n.X*(p.X-c.X)+n.Y*(p.Y-c.Y)+n.Z*(p.Z-c.Z) < 0 {
				n.X, n.Y, n.Z = -n.X, -n.Y, -n.Z
			}
			verts = append(verts, vtx{
				float32(p.X), float32(p.Y), float32(p.Z),
				float32(n.X), float32(n.Y), float32(n.Z),
			})
		}
	}

	numFaces := nU * nV * 2
	header := fmt.Sprintf(`ply
format binary_little_endian 1.0
element vertex %d
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
element face %d
property list uint8 int32 vertex_index
end_header
`, len(verts), numFaces)
	if _, err := w.WriteString(header); err != nil {
		return err
	}
	for _, vv := range verts {
		binary.Write(w, binary.LittleEndian, vv)
	}
	idx := func(ui, vi int) int32 { return int32(ui*(nV+1) + vi) }
	for ui := 0; ui < nU; ui++ {
		for vi := 0; vi < nV; vi++ {
			tr := idx(ui, vi)
			tl := idx(ui+1, vi)
			br := idx(ui, vi+1)
			bl := idx(ui+1, vi+1)
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{tr, bl, tl})
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{tr, br, bl})
		}
	}
	return nil
}

// tubePointVar is tubePoint with a position-dependent radius: the cross-section
// radius at the centerline point is radius(center).
func tubePointVar(path func(float64) geom.Vec, radius func(geom.Vec) float64, u, v float64) geom.Vec {
	const delta = .01
	center := path(v)
	r := radius(center)
	normal, _ := path(v+delta).Minus(path(v - delta)).Unit()
	sinVec, ok := normal.Cross(geom.Dir{X: 0, Y: 0, Z: 1})
	if !ok {
		sinVec, _ = normal.Cross(geom.Dir{X: 1, Y: 0, Z: 0})
	}
	cosVec, _ := normal.Cross(sinVec)
	return cosVec.Scaled(r * math.Cos(u)).Plus(sinVec.Scaled(r * math.Sin(u))).Plus(center)
}

// WriteVarTubePLY is WriteTubePLY with a position-dependent radius: `radius`
// returns the tube radius to use at each centerline point. Pair it with the
// profile from VariableTubeRadius, e.g. radius := func(p geom.Vec) float64 {
// return a + b*p.Len() }.
func WriteVarTubePLY(filename string, path func(float64) geom.Vec, radius func(geom.Vec) float64, nU, nV int) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()

	const twoPi = 2 * math.Pi
	const du, dv = 0.001, 0.001
	type vtx struct{ px, py, pz, nx, ny, nz float32 }
	verts := make([]vtx, 0, (nU+1)*(nV+1))
	for ui := 0; ui <= nU; ui++ {
		u := twoPi * float64(ui) / float64(nU)
		for vi := 0; vi <= nV; vi++ {
			v := twoPi * float64(vi) / float64(nV)
			p := tubePointVar(path, radius, u, v)
			pu := tubePointVar(path, radius, u+du, v)
			pv := tubePointVar(path, radius, u, v+dv)
			n, _ := pu.Minus(p).Cross(pv.Minus(p)).Unit()
			c := path(v)
			if n.X*(p.X-c.X)+n.Y*(p.Y-c.Y)+n.Z*(p.Z-c.Z) < 0 {
				n.X, n.Y, n.Z = -n.X, -n.Y, -n.Z
			}
			verts = append(verts, vtx{
				float32(p.X), float32(p.Y), float32(p.Z),
				float32(n.X), float32(n.Y), float32(n.Z),
			})
		}
	}

	numFaces := nU * nV * 2
	header := fmt.Sprintf(`ply
format binary_little_endian 1.0
element vertex %d
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
element face %d
property list uint8 int32 vertex_index
end_header
`, len(verts), numFaces)
	if _, err := w.WriteString(header); err != nil {
		return err
	}
	for _, vv := range verts {
		binary.Write(w, binary.LittleEndian, vv)
	}
	idx := func(ui, vi int) int32 { return int32(ui*(nV+1) + vi) }
	for ui := 0; ui < nU; ui++ {
		for vi := 0; vi < nV; vi++ {
			tr := idx(ui, vi)
			tl := idx(ui+1, vi)
			br := idx(ui, vi+1)
			bl := idx(ui+1, vi+1)
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{tr, bl, tl})
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{tr, br, bl})
		}
	}
	return nil
}

// WriteBeadTubePLY sweeps a tube of radius r around a closed bead polygon (e.g. a
// RelaxIdealKnot result) and writes a binary PLY. nU is the cross-section
// resolution. Both directions wrap (closed tube around a closed curve). The ring
// frame is built from the tangent and z, matching the rest of the codebase.
func WriteBeadTubePLY(filename string, beads []geom.Vec, r float64, nU int) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	defer w.Flush()

	n := len(beads)
	type vtx struct{ px, py, pz, nx, ny, nz float32 }
	verts := make([]vtx, 0, n*nU)
	for i := 0; i < n; i++ {
		tangent, _ := beads[(i+1)%n].Minus(beads[(i-1+n)%n]).Unit()
		sinVec, ok := tangent.Cross(geom.Dir{X: 0, Y: 0, Z: 1})
		if !ok {
			sinVec, _ = tangent.Cross(geom.Dir{X: 1, Y: 0, Z: 0})
		}
		cosVec, _ := tangent.Cross(sinVec)
		for k := 0; k < nU; k++ {
			u := 2 * math.Pi * float64(k) / float64(nU)
			radial := cosVec.Scaled(math.Cos(u)).Plus(sinVec.Scaled(math.Sin(u)))
			p := radial.Scaled(r).Plus(beads[i])
			verts = append(verts, vtx{
				float32(p.X), float32(p.Y), float32(p.Z),
				float32(radial.X), float32(radial.Y), float32(radial.Z),
			})
		}
	}

	numFaces := n * nU * 2
	header := fmt.Sprintf(`ply
format binary_little_endian 1.0
element vertex %d
property float32 x
property float32 y
property float32 z
property float32 nx
property float32 ny
property float32 nz
element face %d
property list uint8 int32 vertex_index
end_header
`, len(verts), numFaces)
	if _, err := w.WriteString(header); err != nil {
		return err
	}
	for _, vv := range verts {
		binary.Write(w, binary.LittleEndian, vv)
	}
	idx := func(i, k int) int32 { return int32((i%n)*nU + k%nU) }
	for i := 0; i < n; i++ {
		for k := 0; k < nU; k++ {
			a := idx(i, k)
			b := idx(i+1, k)
			c := idx(i+1, k+1)
			d := idx(i, k+1)
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{a, b, c})
			binary.Write(w, binary.LittleEndian, byte(3))
			binary.Write(w, binary.LittleEndian, []int32{a, c, d})
		}
	}
	return nil
}
