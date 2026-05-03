# Sphereish — Generalization 1: Arbitrary-Length Coefficient Arrays

## Goal

Generalize the existing `sphereish(u, v, a, b, c)` function so that each of the three scalar parameters becomes an arbitrary-length array of Fourier coefficients. The original three-parameter form is recovered as the special case where each array has the appropriate single nonzero entry.

This generalization preserves:

- The bounding cube `[-1, 1]³` (always inscribed; tight at six face points, though some touch points drift — see "Bounding cube notes" below).
- Closed-form derivatives, hence closed-form vertex normals.
- Cheap, sufficient self-intersection constraints (linear in the number of coefficients).

It does *not* automatically preserve the original D₄ₕ symmetry — that depended on the b-modulator being a single `sin(2u)` term. Higher symmetry orders are achievable only with specific coefficient structures (see "Symmetry notes" below).

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

Each modulator becomes a sum of harmonics. **All three modulators use frequencies `k = 1, 2, 3, …`**:

```
α(v) = v/2 + Σₖ aₖ · sin(k·v)         (k = 1, 2, …, len(A))
γ(v) = v/2 − Σₖ cₖ · sin(k·v)         (k = 1, 2, …, len(C))
β(u) =       Σₖ bₖ · sin(k·u)         (k = 1, 2, …, len(B))

ρ(v) = sin(α(v))
ζ(v) = cos(γ(v))

S(u, v) = ( ρ(v) · cos(u − β(u)),
            ρ(v) · sin(u + β(u)),
            ζ(v) )
```

**Note on B indexing:** the original `sphereish(u, v, a, b, c)` used `b·sin(2u)`. In the generalized form this corresponds to `B = []float64{0, b}` — i.e., the original scalar `b` is the coefficient at index 1 (the `sin(2u)` term), not index 0. This is a behavioral change from any earlier draft that used `sin(2k·u)` for B: B[0] now means the coefficient of `sin(u)`.

The reason for using `k` rather than `2k` for B: it lets you express both even and odd harmonics. Even-only is recoverable by setting odd-indexed entries to zero; odd harmonics enable shapes with intentional 1-fold (directional) asymmetry, which `2k` could not express.

---

## Well-formed coefficient arrays

A set of modulator arrays `(A, B, C)` is **well-formed** if and only if all three of these inequalities hold:

```
Σₖ k · |aₖ|  <  1/2          for k = 1, …, len(A)
Σₖ k · |bₖ|  <  1            for k = 1, …, len(B)
Σₖ k · |cₖ|  <  1/2          for k = 1, …, len(C)
```

When these bounds hold, the surface is guaranteed to be a smooth, non-self-intersecting closed surface inscribed in `[-1, 1]³`.

Additional notes on what counts as well-formed:

- Empty arrays are valid: `A = nil` or `A = []float64{}` is treated as `α(v) = v/2`, contributing nothing.
- Arrays of any length are valid, provided the Lipschitz bound is satisfied.
- Leading zeros are valid: `B = []float64{0, 0.4}` is a well-formed array specifying only a `0.4·sin(2u)` term.
- Trailing zeros are valid but waste computation.
- Coefficients may be negative; `|·|` in the bound is the absolute value.

The bounds are **sufficient, not necessary**. Coefficient arrays that exceed the bound *may* still produce non-self-intersecting surfaces, but the guarantee is lost and case-by-case verification is required. For a curation workflow that filters bad outputs, exceeding the bound is acceptable. For automated pipelines (or any context where a malformed surface would corrupt downstream stages), enforce the bound.

### Bound derivations

- `Σ k·|aₖ| < 1/2` keeps `α'(v) > 0` everywhere, hence `α(v)` is monotonically increasing from `0` to `π` as `v` goes from `0` to `2π`, hence `ρ(v) = sin(α(v)) ≥ 0` with equality only at `v = 0, 2π`. (No collapse through the z-axis.)
- `Σ k·|cₖ| < 1/2` keeps `γ'(v) > 0` everywhere, hence `ζ(v)` is monotonically decreasing from `1` to `−1`. (Each horizontal slice of the surface lies at a unique z, so rings cannot intersect each other.)
- `Σ k·|bₖ| < 1` keeps `|β'(u)| < 1` everywhere, hence `φ'(u) = 1 − β'(u) > 0` and `ψ'(u) = 1 + β'(u) > 0` everywhere. (No fold in the angular parametrization.)

### Recommended enforcement API

Provide a function that scales an array down to fit its bound:

```go
// LipschitzNorm returns Σ k·|coeffs[k-1]|.
func LipschitzNorm(coeffs []float64) float64

// SafeBounds reports whether m satisfies the sufficient
// non-self-intersection conditions.
func (m Modulators) SafeBounds() bool

// ScaleToFit reduces coeffs in place so that LipschitzNorm(coeffs) ≤ maxNorm.
// Returns the scale factor applied (1.0 if no scaling was needed).
// Use maxNorm = 0.49 for A and C, maxNorm = 0.99 for B.
func ScaleToFit(coeffs []float64, maxNorm float64) float64
```

---

## Closed-form derivatives

### Building blocks

```
α'(v)  = 1/2 + Σₖ k · aₖ · cos(k·v)
γ'(v)  = 1/2 − Σₖ k · cₖ · cos(k·v)
β'(u)  =        Σₖ k · bₖ · cos(k·u)

ρ'(v)  =  cos(α(v)) · α'(v)
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

Normalize before use.

---

## Reference Go implementation

```go
package sphereish

import "math"

// Modulators carries the Fourier coefficient arrays for the three modulators.
//
//   A[k-1] is the coefficient of sin(k·v) in the radial modulator.
//   B[k-1] is the coefficient of sin(k·u) in the angular modulator.
//   C[k-1] is the coefficient of sin(k·v) in the vertical modulator.
//
// All three modulators use frequencies 1, 2, 3, ... (matching array index + 1).
// Note: the original sphereish(u, v, a, b, c) used b·sin(2u), which corresponds
// to B = []float64{0, b} here (coefficient at index 1, not index 0).
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
        beta += bk * math.Sin(k*u)
    }
    rho := math.Sin(alpha)
    zeta := math.Cos(gamma)
    return Vec3{
        rho * math.Cos(u-beta),
        rho * math.Sin(u+beta),
        zeta,
    }
}

// Normal returns the unit outward normal at (u, v). Near the poles
// (v ≈ 0 or v ≈ 2π) the analytic limits (0,0,+1) and (0,0,-1) are
// substituted to avoid the parametric pole degeneracy.
func Normal(u, v float64, m Modulators) Vec3 {
    const epsilon = 1e-6
    if v < epsilon {
        return Vec3{0, 0, 1}
    }
    if (2*math.Pi - v) < epsilon {
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

    // u-side accumulations (frequencies are k, NOT 2k)
    beta, betaP := 0.0, 0.0
    for i, bk := range m.B {
        k := float64(i + 1)
        beta += bk * math.Sin(k*u)
        betaP += k * bk * math.Cos(k*u)
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
    // n = sv × su (outward orientation)
    n := Vec3{
        sv[1]*su[2] - sv[2]*su[1],
        sv[2]*su[0] - sv[0]*su[2],
        sv[0]*su[1] - sv[1]*su[0],
    }

    // Normalize
    mag := math.Sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2])
    if mag < 1e-12 {
        // Fold in parametrization (only reachable if SafeBounds is violated).
        // Fallback to up; the caller should be enforcing SafeBounds upstream.
        return Vec3{0, 0, 1}
    }
    return Vec3{n[0] / mag, n[1] / mag, n[2] / mag}
}

// LipschitzNorm returns Σ k·|coeffs[k-1]|.
func LipschitzNorm(coeffs []float64) float64 {
    s := 0.0
    for i, c := range coeffs {
        k := float64(i + 1)
        s += k * math.Abs(c)
    }
    return s
}

// SafeBounds reports whether the modulators satisfy the sufficient
// non-self-intersection conditions (i.e., are well-formed).
func (m Modulators) SafeBounds() bool {
    return LipschitzNorm(m.A) < 0.5 &&
        LipschitzNorm(m.B) < 1.0 &&
        LipschitzNorm(m.C) < 0.5
}

// ScaleToFit reduces coeffs in place so that LipschitzNorm(coeffs) ≤ maxNorm.
// Returns the scale factor applied (1.0 if no scaling was needed).
// Use maxNorm = 0.49 for A and C arrays, maxNorm = 0.99 for B arrays.
func ScaleToFit(coeffs []float64, maxNorm float64) float64 {
    n := LipschitzNorm(coeffs)
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

## Bounding cube notes

For any well-formed `(A, B, C)`, the surface is inscribed in `[-1, 1]³` and touches each of the six faces.

The touch points on the **x and z faces are pinned**:

- `(1, 0, 0)` is touched at `(u, v) = (0, π)`.
- `(-1, 0, 0)` is touched at `(u, v) = (π, π)`.
- `(0, 0, 1)` is touched at `v = 0`.
- `(0, 0, -1)` is touched at `v = 2π`.

These are independent of `(A, B, C)` because `β(0) = β(π) = 0` regardless of B (every `sin(k·u)` term vanishes at `u = 0` and `u = π`), and the pole behavior depends only on the `v = 0` and `v = 2π` endpoints.

The touch points on the **y faces drift** when B contains odd-index nonzero entries:

- With even-only B (only B[1], B[3], B[5], … nonzero — `sin(2u)`, `sin(4u)`, `sin(6u)`, …): y-touch points are at `(0, 1, 0)` and `(0, -1, 0)`.
- With odd-index B nonzero: y-touch points are at `(x_drift, ±1, 0)` for some `x_drift ≠ 0`. The bounding cube is still tight on the y faces (the surface still reaches `y = ±1`), the touch points are just no longer at face centers.

This drift is purely cosmetic for bounding-box purposes (the cube `[-1, 1]³` is still correct), but it is the underlying reason that odd-index b-coefficients break rotational symmetry — see next section.

---

## Symmetry notes

The original 3-parameter `sphereish` has symmetry group **D₄ₕ** (order 16). The generalized form does *not* automatically preserve this — D₄ₕ is recovered only when B specifies exactly the original structure (only B[1] nonzero, i.e., only the `sin(2u)` term).

What the generalized form *always* preserves, for any well-formed coefficients:

- **Horizontal mirror** through the xy-plane (z → −z, via v → 2π − v). This is preserved because every modulator on the v-axis is built from `sin(k·v)` terms, which transform under v → 2π − v in a way that flips ζ while preserving ρ.

That's the *only* always-preserved symmetry. Baseline group: **C_s**, order 2.

Conditional symmetries:

| B structure | Additional symmetry | Total group | Order |
|---|---|---|---|
| Empty / all zero | SO(2) rotation about z | **D_∞h** | ∞ |
| Only B[1] nonzero (sin(2u) only) | C₄ rotation, σᵥ planes | **D₄ₕ** | 16 |
| Only B[2k−1] nonzero, single k (sin(2k·u) only) | C_{2k} rotation, σᵥ planes | **D_{2k,h}** | 8k |
| Any even-only B (only B[1], B[3], B[5], … nonzero) | At least C₂ rotation about z | **C₂ₕ** or higher | ≥ 4 |
| Any B with odd-index entry nonzero | None beyond horizontal mirror | **C_s** | 2 |

The key fact: **odd-index B entries (B[0], B[2], B[4], …, corresponding to `sin(u)`, `sin(3u)`, `sin(5u)`, …) break C₂ rotational symmetry around z.** Because `sin(k·(u+π)) = (−1)^k · sin(k·u)`, only even k values leave β unchanged under u → u + π.

For animation design:

- If you want optical effects (caustics, dispersion fringes, thin-film patterns) to read as visually coherent due to surface symmetry, use even-only B with all coefficients aligned to a single dominant rotational order (i.e., only one nonzero entry, or multiple entries whose indices share a common factor).
- If you want subtle directional asymmetry for a more "physical-object" feel, include small odd-index B coefficients. The geometric asymmetry will be small, but optical effects can amplify it visibly.

---

## Implementation notes for the Claude Code handoff

1. The existing `sphereish(u, v, a, b, c)` function should be retained for backward compatibility. The new generalized function `Surface(u, v, m)` lives alongside it, taking a `Modulators` struct.

2. The `Modulators` struct uses Go slices of arbitrary length. Empty slices are valid and reduce to the trivial case (sphere if all empty).

3. Vertex normals should use the closed-form `Normal(u, v, m)` function, not numerical differentiation. This matters for the optical effects (dispersion, caustics, thin-film), where normal accuracy directly affects the visual outcome.

4. Pole handling at v ≈ 0 and v ≈ 2π should match whatever convention the existing pipeline uses. The reference implementation substitutes (0,0,±1) within `epsilon` of the poles; if the existing pipeline averages adjacent vertex normals at the poles, that approach also works — just keep it consistent.

5. The well-formedness constraint should be enforced at the parameter-generation stage (inside the `sin(prime·t)` modulator-coefficient generator), not at the surface-evaluation stage. By the time `Surface` or `Normal` is called, coefficients should already be well-formed.

6. **Migration warning for existing series**: any existing series ported to the new form by writing `B = []float64{b}` (with a single scalar) will silently produce a *different* shape than the original `sphereish(u, v, a, b, c)`. The original used `b·sin(2u)`; the new `B = []float64{b}` means `b·sin(u)`. The correct migration is `B = []float64{0, b}`. Worth a one-time grep across the existing series files.

7. Performance: the per-vertex cost is O(N) where N is the total number of coefficients across all three arrays. For typical N ≤ 10 and tessellation 1024² this is well under a second on a single core; threading is unnecessary unless mesh generation later proves to be a bottleneck (it almost certainly won't be next to render time).

8. Tests worth writing:
   - `Surface(u, v, Modulators{})` matches the unit sphere parametrization to floating-point precision.
   - `Surface(u, v, Modulators{A: []float64{a}, B: []float64{0, b}, C: []float64{c}})` matches the original `sphereish(u, v, a, b, c)` to floating-point precision.
   - For any well-formed `m`, `Normal(u, v, m)` has magnitude 1 to within 1e-9 (excluding pole regions).
   - For any well-formed `m`, `Normal` agrees with finite-difference numerical normals to within 1e-6 (excluding pole regions).
   - `(Modulators{}).SafeBounds()` returns true.
   - Coefficients exactly at the bound (e.g., `A = []float64{0.5}`) return false from `SafeBounds` — the bound is strict inequality.
