# Sphereish — Generalization 1: Arbitrary-Length Coefficient Arrays

## Goal

Generalize the existing `sphereish(u, v, a, b, c)` function so that each of the three scalar parameters becomes an arbitrary-length array of Fourier coefficients. The original three-parameter form is recovered as the special case where each array has length 1.

This generalization preserves:

- The bounding-cube property (surface always inscribed in `[-1, 1]³`, touching at the six axis points).
- Closed-form derivatives, hence closed-form vertex normals.
- Cheap, sufficient self-intersection constraints (linear in the number of coefficients).
- The baseline symmetry group **C₂ₕ** (C₂ rotation around z plus horizontal mirror).

It does *not* automatically preserve the original D₄ₕ symmetry — that was specific to having a single `sin(2u)` term in the b-modulator. Higher symmetry orders are achievable only with specific coefficient structures (see "Symmetry notes" below).

---

## Mathematical formulation

### Original (3-parameter) form

```
S(u, v) = ( ρ(v) · cos(u − b·sin(2u)),
            ρ(v) · sin(u + b·sin(2u)),
            ζ(v) )

where  ρ(v) = sin(v/2 + a·sin(v))
       ζ(v) = cos(v/2 − c·sin(v))
```

### Generalized form

Replace each scalar modulator with a sum of harmonics:

```
α(v) = v/2 + Σₖ aₖ · sin(k·v)              (k = 1, 2, 3, …, len(A))
γ(v) = v/2 − Σₖ cₖ · sin(k·v)              (k = 1, 2, 3, …, len(C))
β(u) =       Σₖ bₖ · sin(2k·u)             (k = 1, 2, 3, …, len(B))

ρ(v) = sin(α(v))
ζ(v) = cos(γ(v))

S(u, v) = ( ρ(v) · cos(u − β(u)),
            ρ(v) · sin(u + β(u)),
            ζ(v) )
```

The b-modulator uses frequencies `2k` (not `k`) intentionally — these are the frequencies that vanish at the cardinal points `u = 0, π/2, π, 3π/2`, preserving the property that the surface always passes through `(±1, 0, 0)` and `(0, ±1, 0)`.

---

## Self-intersection constraints

Sufficient (conservative) bounds, to be checked before each render:

```
Σ k · |aₖ|     < 1/2         for k = 1, …, len(A)
Σ 2k · |bₖ|    < 1            for k = 1, …, len(B)
Σ k · |cₖ|     < 1/2         for k = 1, …, len(C)
```

Derivation: `Σ k·|aₖ| < 1/2` keeps `α'(v) > 0` near `v = 0`, hence `ρ(v) ≥ 0`. Similarly for c. `Σ 2k·|bₖ| < 1` keeps `φ'(u) = 1 − β'(u) > 0` and `ψ'(u) = 1 + β'(u) > 0` everywhere, preventing folds in the angular parametrization.

These are sufficient, not tight. A coefficient array that exceeds the bound may still produce a non-self-intersecting surface, but the guarantee is lost. For a curation workflow, exceeding the bound and discarding bad results is acceptable; for automated pipelines, enforce the bound.

### Recommended API

Provide a function that scales an array down to fit the bound, returning the scale factor used:

```go
// ScaleToFit reduces coeffs in place so that Σ kMul·k·|coeffs[k-1]| ≤ maxNorm.
// Returns the scale factor (1.0 if no scaling was needed).
func ScaleToFit(coeffs []float64, kMul, maxNorm float64) float64
```

with `kMul = 1` for A and C, `kMul = 2` for B, and `maxNorm` slightly less than the bound (e.g., `0.49` for A/C, `0.99` for B).

---

## Closed-form derivatives

### Building blocks

```
α'(v)  = 1/2 + Σₖ k · aₖ · cos(k·v)
γ'(v)  = 1/2 − Σₖ k · cₖ · cos(k·v)
β'(u)  =        Σₖ 2k · bₖ · cos(2k·u)

ρ'(v)  = cos(α(v)) · α'(v)
ζ'(v)  = −sin(γ(v)) · γ'(v)
φ'(u)  = 1 − β'(u)
ψ'(u)  = 1 + β'(u)
```

### Surface partial derivatives

```
∂S/∂u = ( −ρ(v) · sin(u − β(u)) · φ'(u),
           ρ(v) · cos(u + β(u)) · ψ'(u),
           0 )

∂S/∂v = (  ρ'(v) · cos(u − β(u)),
           ρ'(v) · sin(u + β(u)),
           ζ'(v) )
```

### Outward normal

```
n(u, v) = ∂S/∂v × ∂S/∂u    (in this order — outward orientation)
```

The standard cross-product formula. Normalize before use.

---

## Reference Go implementation

```go
package sphereish

import (
    "math"
)

// Modulators carries the Fourier coefficient arrays for the three modulators.
// A[k-1] is the coefficient of sin(k·v) in the radial modulator (frequencies k).
// B[k-1] is the coefficient of sin(2k·u) in the angular modulator (frequencies 2k).
// C[k-1] is the coefficient of sin(k·v) in the vertical modulator (frequencies k).
type Modulators struct {
    A []float64
    B []float64
    C []float64
}

// Vec3 is a 3D vector. Replace with whatever the project uses (geom.Vec).
type Vec3 [3]float64

// Surface returns S(u, v).
func Surface(u, v float64, m Modulators) Vec3 {
    alpha, gamma := v/2, v/2
    for i, ak := range m.A {
        k := float64(i + 1)
        alpha += ak * math.Sin(k*v)
    }
    for i, ck := range m.C {
        k := float64(i + 1)
        gamma -= ck * math.Sin(k*v)
    }
    beta := 0.0
    for i, bk := range m.B {
        k := float64(i + 1)
        beta += bk * math.Sin(2*k*u)
    }
    rho := math.Sin(alpha)
    zeta := math.Cos(gamma)
    return Vec3{
        rho * math.Cos(u-beta),
        rho * math.Sin(u+beta),
        zeta,
    }
}

// Normal returns the unit outward normal at (u, v). For points within
// `epsilon` of the poles (v ≈ 0, 2π) the analytic limits (0,0,+1) and
// (0,0,-1) are substituted to avoid the parametric pole degeneracy.
func Normal(u, v float64, m Modulators) Vec3 {
    const epsilon = 1e-6
    if v < epsilon || (2*math.Pi-v) < epsilon {
        if v < epsilon {
            return Vec3{0, 0, 1}
        }
        return Vec3{0, 0, -1}
    }

    // v-side accumulations
    alpha, alphaP := v/2, 0.5
    gamma, gammaP := v/2, 0.5
    for i, ak := range m.A {
        k := float64(i + 1)
        alpha += ak * math.Sin(k*v)
        alphaP += k * ak * math.Cos(k*v)
    }
    for i, ck := range m.C {
        k := float64(i + 1)
        gamma -= ck * math.Sin(k*v)
        gammaP -= k * ck * math.Cos(k*v)
    }
    rho := math.Sin(alpha)
    rhoP := math.Cos(alpha) * alphaP
    zetaP := -math.Sin(gamma) * gammaP

    // u-side accumulations
    beta, betaP := 0.0, 0.0
    for i, bk := range m.B {
        k := float64(i + 1)
        beta += bk * math.Sin(2*k*u)
        betaP += 2 * k * bk * math.Cos(2*k*u)
    }
    phi := u - beta
    psi := u + beta
    phiP := 1 - betaP
    psiP := 1 + betaP

    // ∂S/∂u
    su := Vec3{
        -rho * math.Sin(phi) * phiP,
        rho * math.Cos(psi) * psiP,
        0,
    }
    // ∂S/∂v
    sv := Vec3{
        rhoP * math.Cos(phi),
        rhoP * math.Sin(psi),
        zetaP,
    }
    // n = sv × su (outward)
    n := Vec3{
        sv[1]*su[2] - sv[2]*su[1],
        sv[2]*su[0] - sv[0]*su[2],
        sv[0]*su[1] - sv[1]*su[0],
    }

    // Normalize
    mag := math.Sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2])
    if mag < 1e-12 {
        // Fold in parametrization (only possible if self-intersection
        // constraints are violated). Return up as a fallback; ideally
        // the caller's coefficient arrays satisfy the safety bounds and
        // this branch is unreachable.
        return Vec3{0, 0, 1}
    }
    return Vec3{n[0] / mag, n[1] / mag, n[2] / mag}
}

// LipschitzNorm returns Σ kMul·k·|coeffs[k-1]|.
func LipschitzNorm(coeffs []float64, kMul float64) float64 {
    s := 0.0
    for i, c := range coeffs {
        k := float64(i + 1)
        s += kMul * k * math.Abs(c)
    }
    return s
}

// SafeBounds reports whether the modulators satisfy the conservative
// sufficient conditions for non-self-intersection.
func (m Modulators) SafeBounds() bool {
    return LipschitzNorm(m.A, 1) < 0.5 &&
        LipschitzNorm(m.B, 2) < 1.0 &&
        LipschitzNorm(m.C, 1) < 0.5
}

// ScaleToFit reduces coeffs in place so that LipschitzNorm(coeffs, kMul) ≤ maxNorm.
// Returns the scale factor applied (1.0 if no scaling was needed).
func ScaleToFit(coeffs []float64, kMul, maxNorm float64) float64 {
    n := LipschitzNorm(coeffs, kMul)
    if n <= maxNorm {
        return 1.0
    }
    s := maxNorm / n
    for i := range coeffs {
        coeffs[i] *= s
    }
    return s
}
```

---

## Symmetry notes

The original 3-parameter `sphereish` has symmetry group **D₄ₕ** (order 16). The generalized form does *not* automatically preserve this — D₄ₕ depends on a single `sin(2u)` term in the b-modulator, and adding higher b-harmonics breaks the antisymmetry property `β(u + π/2) = −β(u)` that produces the 4-fold rotation.

What the generalized form *does* always preserve:

- **C₂ rotation around z** (because shifting u by π leaves β unchanged when β is a sum of `sin(2k·u)` terms).
- **Horizontal mirror through the xy-plane** (because the v ↔ 2π − v involution is a symmetry of any sum-of-sines modulator on the v-modulators).
- Combined: **C₂ₕ**, order 4.

Higher symmetry orders are recoverable for specific coefficient structures:

- **D₄ₕ**: only `b₁` nonzero (other bₖ zero) — i.e., the original case.
- **D_{4k,h}**: only a single `bₖ` nonzero for some k.
- Mixing multiple b-harmonics generally collapses to C₂ₕ.

This is something to be aware of when designing animations: if you want the rendered optical patterns (caustics, dispersion, thin-film) to read as visually coherent due to surface symmetry, restrict the b-modulator to a single nonzero coefficient. If you want subtle asymmetry for a more "physical-object" feel, mix b-harmonics.

---

## Implementation notes for the Claude Code handoff

1. The existing `sphereish(u, v, a, b, c)` function should be retained for backward compatibility. The new generalized function lives alongside it, taking a `Modulators` struct.

2. The `Modulators` struct uses Go slices of arbitrary length. Empty slices are valid and reduce to the trivial case (sphere if all empty).

3. Vertex normals should use the closed-form `Normal(u, v, m)` function, not numerical differentiation. This is important for the optical effects (dispersion, caustics, thin-film) where normal accuracy directly affects the visual outcome.

4. The pole handling at v ≈ 0 and v ≈ 2π should match whatever convention the existing pipeline uses. If it currently averages adjacent vertex normals at the poles, that approach also works — just keep it consistent.

5. The self-intersection constraints should be enforced at the parameter-generation stage (inside the `sin(prime·t)` modulator-coefficient generator), not at the surface-evaluation stage. By the time `Surface` or `Normal` is called, coefficients should already satisfy the bounds.

6. Performance: the per-vertex cost is O(N) where N is the number of coefficients across all three arrays. For typical N ≤ 10 and tessellation 1024² this is well under a second on a single core; threading is not needed unless mesh generation later proves to be a bottleneck (it almost certainly won't be next to render time).

7. Tests worth writing: (a) `Surface(u, v, Modulators{})` matches the unit sphere parametrization to floating-point precision; (b) `Surface` with single-element arrays matches the original `sphereish` function; (c) `Normal` magnitude is 1 to within 1e-9 for non-pole points satisfying `SafeBounds`; (d) `Normal` agrees with finite-difference numerical normals to within 1e-6 for non-pole points satisfying `SafeBounds`.
