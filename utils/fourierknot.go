package utils

import (
	"math"

	"github.com/hunterloftis/pbr/pkg/geom"
)

// FourierKnot is a closed space curve stored as a truncated Fourier series of
// vectors -- the representation the Knot Atlas uses for its ideal knots:
//
//	P(t) = A0 + sum_{m=1..K} Cos[m-1]*cos(m t) + Sin[m-1]*sin(m t),  t in [0, 2pi)
//
// Cos[m-1] and Sin[m-1] are the (vector) coefficients of the m-th harmonic.
// External coefficient tables (e.g. Knot Atlas downloads) drop straight into this
// struct; FitFourier produces one from a sampled curve.
type FourierKnot struct {
	A0       geom.Vec
	Cos, Sin []geom.Vec
}

// FitFourier computes the K-harmonic Fourier series of a closed bead polygon
// sampled at uniform parameter t_i = 2*pi*i/N (so feed it an arc-length-resampled
// curve, e.g. a RelaxIdealKnot result). Truncating to K harmonics is a low-pass
// filter: more harmonics = closer to the input (see RMSError).
func FitFourier(beads []geom.Vec, K int) FourierKnot {
	n := len(beads)
	fk := FourierKnot{Cos: make([]geom.Vec, K), Sin: make([]geom.Vec, K)}
	var sum geom.Vec
	for _, p := range beads {
		sum = sum.Plus(p)
	}
	fk.A0 = sum.Scaled(1 / float64(n))
	for m := 1; m <= K; m++ {
		var c, s geom.Vec
		for i := 0; i < n; i++ {
			t := float64(m) * 2 * math.Pi * float64(i) / float64(n)
			c = c.Plus(beads[i].Scaled(math.Cos(t)))
			s = s.Plus(beads[i].Scaled(math.Sin(t)))
		}
		fk.Cos[m-1] = c.Scaled(2 / float64(n))
		fk.Sin[m-1] = s.Scaled(2 / float64(n))
	}
	return fk
}

// Eval returns P(t).
func (fk FourierKnot) Eval(t float64) geom.Vec {
	p := fk.A0
	for m := 1; m <= len(fk.Cos); m++ {
		mt := float64(m) * t
		p = p.Plus(fk.Cos[m-1].Scaled(math.Cos(mt))).Plus(fk.Sin[m-1].Scaled(math.Sin(mt)))
	}
	return p
}

// Path adapts the series to the centerline signature used by the tube writers.
func (fk FourierKnot) Path() func(float64) geom.Vec {
	return func(t float64) geom.Vec { return fk.Eval(t) }
}

// RMSError is the root-mean-square distance between the truncated series and the
// beads it was fit to, evaluated at the bead parameters.
func (fk FourierKnot) RMSError(beads []geom.Vec) float64 {
	n := len(beads)
	var se float64
	for i := 0; i < n; i++ {
		t := 2 * math.Pi * float64(i) / float64(n)
		d := fk.Eval(t).Minus(beads[i]).Len()
		se += d * d
	}
	return math.Sqrt(se / float64(n))
}
