package utils

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"math"
	"os"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// unitVec normalizes a vector, returning +X for a zero vector.
func unitVec(v geom.Vec) geom.Vec {
	l := v.Len()
	if l == 0 {
		return geom.Vec{X: 1}
	}
	return v.Scaled(1 / l)
}

// rotateAbout rotates v around the unit axis k by ang (Rodrigues' formula).
func rotateAbout(v, k geom.Vec, ang float64) geom.Vec {
	c, s := math.Cos(ang), math.Sin(ang)
	return v.Scaled(c).Plus(k.Cross(v).Scaled(s)).Plus(k.Scaled(k.Dot(v) * (1 - c)))
}

// closedFrame builds a seamless cross-section frame (e1[i], e2[i] orthonormal and
// perpendicular to tangents[i]) for a CLOSED sequence of unit tangents. It uses
// parallel transport -- a rotation-minimizing frame -- so there is NO world-axis
// reference and hence no seam where the centerline tangent turns vertical (the
// old tangent x z-hat frame seamed at every such point). The loop-closure twist
// (holonomy) is then distributed uniformly around the loop so the frame also
// matches up where the curve closes (frame[n] == frame[0]).
func closedFrame(tangents []geom.Vec) (e1, e2 []geom.Vec) {
	n := len(tangents)
	e1 = make([]geom.Vec, n)
	e2 = make([]geom.Vec, n)

	// Seed: any unit vector perpendicular to tangents[0].
	t0 := tangents[0]
	a := geom.Vec{X: 1}
	if math.Abs(t0.Dot(a)) > 0.9 {
		a = geom.Vec{Y: 1}
	}
	r := unitVec(a.Minus(t0.Scaled(t0.Dot(a))))
	e1[0] = r

	// Parallel-transport r along the tangents.
	transport := func(r, ti, tn geom.Vec) geom.Vec {
		axis := ti.Cross(tn)
		if sinL := axis.Len(); sinL > 1e-12 {
			ang := math.Atan2(sinL, ti.Dot(tn))
			r = rotateAbout(r, axis.Scaled(1/sinL), ang)
		}
		// Re-orthogonalize against the new tangent to kill numeric drift.
		return unitVec(r.Minus(tn.Scaled(tn.Dot(r))))
	}
	for i := 0; i < n-1; i++ {
		r = transport(r, tangents[i], tangents[i+1])
		e1[i+1] = r
	}

	// Holonomy: transport once more across the closing seam (tangents[n-1] ->
	// tangents[0]) and measure the residual twist about tangents[0].
	rEnd := transport(e1[n-1], tangents[n-1], tangents[0])
	phi := math.Atan2(e1[0].Cross(rEnd).Dot(tangents[0]), e1[0].Dot(rEnd))

	// Spread -phi over the loop so the wrapped frame equals e1[0].
	for i := 0; i < n; i++ {
		e1[i] = rotateAbout(e1[i], tangents[i], -phi*float64(i)/float64(n))
		e2[i] = unitVec(tangents[i].Cross(e1[i]))
	}
	return e1, e2
}

// writeTubeMesh writes a binary little-endian PLY tube: ring `nU` segments around,
// `nV` rings along, with cross-section frame (e1,e2) and per-ring radius from
// radiusAt. centers/e1/e2 have length nV and wrap (ring nV == ring 0), giving a
// seamless closed tube. Normals are the outward radial direction.
func writeTubeMesh(filename string, centers, e1, e2 []geom.Vec, radiusAt func(i int) float64, nU, nV int) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriterSize(f, 1<<20)
	defer w.Flush()

	const twoPi = 2 * math.Pi
	nVerts := (nU + 1) * (nV + 1)
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
`, nVerts, numFaces)
	if _, err := w.WriteString(header); err != nil {
		return err
	}

	// Stream vertices directly into raw little-endian bytes (no reflection) so
	// 50M-vertex meshes write in seconds, not minutes, and without holding the
	// whole vertex array in memory. Order is ui-major / vi-minor so the vertex
	// index is exactly ui*(nV+1)+vi, matching the face indices below.
	vb := make([]byte, 24)
	putF := func(off int, val float64) {
		binary.LittleEndian.PutUint32(vb[off:], math.Float32bits(float32(val)))
	}
	for ui := 0; ui <= nU; ui++ {
		u := twoPi * float64(ui) / float64(nU)
		c, s := math.Cos(u), math.Sin(u)
		for vi := 0; vi <= nV; vi++ {
			j := vi % nV // ring nV reuses ring 0 -> exact closure
			rx := e1[j].X*c + e2[j].X*s
			ry := e1[j].Y*c + e2[j].Y*s
			rz := e1[j].Z*c + e2[j].Z*s
			r := radiusAt(j)
			putF(0, centers[j].X+rx*r)
			putF(4, centers[j].Y+ry*r)
			putF(8, centers[j].Z+rz*r)
			putF(12, rx) // radial is unit -> outward normal
			putF(16, ry)
			putF(20, rz)
			if _, err := w.Write(vb); err != nil {
				return err
			}
		}
	}

	fb := make([]byte, 13)
	fb[0] = 3
	putI := func(off int, v int32) { binary.LittleEndian.PutUint32(fb[off:], uint32(v)) }
	idx := func(ui, vi int) int32 { return int32(ui*(nV+1) + vi) }
	for ui := 0; ui < nU; ui++ {
		for vi := 0; vi < nV; vi++ {
			tr, tl, br, bl := idx(ui, vi), idx(ui+1, vi), idx(ui, vi+1), idx(ui+1, vi+1)
			putI(1, tr)
			putI(5, bl)
			putI(9, tl)
			if _, err := w.Write(fb); err != nil {
				return err
			}
			putI(1, tr)
			putI(5, br)
			putI(9, bl)
			if _, err := w.Write(fb); err != nil {
				return err
			}
		}
	}
	return nil
}

// sampleCenterline samples a closed 2*pi-periodic centerline at nV points,
// returning the centers and unit tangents (central differences).
func sampleCenterline(path func(float64) geom.Vec, nV int) (centers, tangents []geom.Vec) {
	const twoPi = 2 * math.Pi
	const delta = 1e-3
	centers = make([]geom.Vec, nV)
	tangents = make([]geom.Vec, nV)
	for i := 0; i < nV; i++ {
		v := twoPi * float64(i) / float64(nV)
		centers[i] = path(v)
		tangents[i] = unitVec(path(v + delta).Minus(path(v - delta)))
	}
	return centers, tangents
}

// WriteTubePLY sweeps a tube of radius `tubeRadius` around the closed centerline
// `path` (period 2*pi) and writes it as a binary little-endian PLY that opens
// directly in 3dviewer.net. nU is the cross-section resolution (ring around the
// tube), nV the resolution along the centerline. Nested knots wiggle fast, so nV
// usually needs to be in the low thousands to look smooth. The cross-section uses
// a seamless rotation-minimizing frame (no reference-axis seams).
func WriteTubePLY(filename string, path func(float64) geom.Vec, tubeRadius float64, nU, nV int) error {
	centers, tangents := sampleCenterline(path, nV)
	e1, e2 := closedFrame(tangents)
	return writeTubeMesh(filename, centers, e1, e2, func(int) float64 { return tubeRadius }, nU, nV)
}

// WriteVarTubePLY is WriteTubePLY with a position-dependent radius: `radius`
// returns the tube radius to use at each centerline point. Pair it with the
// profile from VariableTubeRadius, e.g. radius := func(p geom.Vec) float64 {
// return a + b*p.Len() }.
func WriteVarTubePLY(filename string, path func(float64) geom.Vec, radius func(geom.Vec) float64, nU, nV int) error {
	centers, tangents := sampleCenterline(path, nV)
	e1, e2 := closedFrame(tangents)
	return writeTubeMesh(filename, centers, e1, e2, func(i int) float64 { return radius(centers[i]) }, nU, nV)
}

// WriteBeadTubePLY sweeps a tube of radius r around a closed bead polygon (e.g. a
// ridgerunner result) and writes a binary PLY. nU is the cross-section
// resolution. Both directions wrap (closed tube around a closed curve), and the
// cross-section uses the same seamless rotation-minimizing frame as WriteTubePLY.
func WriteBeadTubePLY(filename string, beads []geom.Vec, r float64, nU int) error {
	n := len(beads)
	tangents := make([]geom.Vec, n)
	for i := 0; i < n; i++ {
		tangents[i] = unitVec(beads[(i+1)%n].Minus(beads[(i-1+n)%n]))
	}
	e1, e2 := closedFrame(tangents)
	return writeTubeMesh(filename, beads, e1, e2, func(int) float64 { return r }, nU, n)
}
