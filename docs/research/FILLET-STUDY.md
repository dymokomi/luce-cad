# Fillet Study: G2 corners, mixed corners, speed, slivers, edge cases

This study follows [CAD-MODELING-STUDY.md](CAD-MODELING-STUDY.md) and does not repeat it. That study
covers the ChFi3d outline, the KPart table, the General Fuse structure and the basic papers. This one
answers five specific questions about luce-cad's fillets as they stand in [../MODELING.md](../MODELING.md):
tool solids swept along edges and combined by booleans, one boolean per edge, with miter and ball
corners, and G2/G3 profiles swept straight.

OCCT paths below are relative to
`https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/`. They were read from the
master branch on 2026-10-08.

## 0. Summary

| Problem | Recommendation |
|---|---|
| 1. G2/G3 corners and rims | Build G2 corners directly, not from tool booleans: a setback corner whose six-sided hole is filled by one trimmed B-spline, fitted by constrained least squares with a thin-plate fairing term. With the cross-boundary direction fixed, the G2 conditions against planes and straight sweeps are **linear**. G2 rims are exact NURBS surfaces of revolution, and every coaxial intersection with them is closed form (circles). Add one general SSI: a B-spline against an implicit surface, traced in the B-spline's (u,v). Contact curves are always known from the construction; never compute them by intersection. |
| 2. Concave and mixed corners | All-concave corner: the same ball (sphere) as the convex case, mirrored. Mixed corners come out of **ordering plus tangent-chain propagation**. With one convex and two concave edges, fillet the convex edge first; the concave chain then runs line, arc, line and gives cylinder, torus, cylinder. With two convex and one concave, fillet the concave edge first; the convex chain gives a torus of major radius R−r, a sphere when R = r, and an error (or a setback) when r > R. This is OCCT's `ToricRotule` and `ComputeCorner` torus case. |
| 3. Speed | (a) Cheap wins in the boolean: a sort-and-sweep pair cull, copying faces outside the tool's box untouched, and classifying once per connected block of pieces (OCCT's connexity blocks). Together they make each per-edge boolean cost depend on the faces near the edge, not on the whole body. (b) Then local face replacement for planar bodies: trim each plane's polygon in 2D by its contact lines, generate the blend and corner faces, and stitch them by generator keys. Keep the booleans as the fallback. |
| 4. Slivers | A G2 contact is a **zero-angle cusp** in the face's domain, so slivers there cannot be avoided by better triangles. Mesh cusp regions as a zipper strip that pairs samples by arc length. Mesh sphere corner patches on a slerped barycentric grid rather than a (u,v) chart. Store vertices in float32 relative to a body origin and check orientation after the conversion. |
| 5. Edge cases | By how often users hit them: tangent chains; blends on plane/cylinder line edges and cone rims; radius too large (face consumed, edge shorter than its setback); holes near edges; cylinder/cylinder edges; chamfer setback triangle; variable radius; >3-edge vertices. Details and kernel behaviour in §5. |

## 1. G2/G3 fillets at corners and on rims

### 1.1 What the kernels do

**OpenCASCADE (FreeCAD).** There are no G2 fillets. `BlendFunc/` holds constant radius, evolving
radius (`EvolRad`), chord (`Corde`), chamfers and throat chamfers; every cross-section is a circle or a
line. Corners are G1, and they are built directly as surface data, never by booleans.
`ChFi3d/ChFi3d_Builder.cxx::PerformFilletOnVertex` chooses the corner method by how many stripes end
at the vertex and how many sharp edges it has (`ChFi3d_NumberOfSharpEdges`):

| Stripes at the vertex | Sharp edges ≤ 3 | Otherwise |
|---|---|---|
| 1 | `PerformOneCorner` (`ChFi3d_Builder_C1.cxx`): intersect the end with the face beyond, or close with a GeomFill cap | `PerformIntersectionAtEnd` |
| 2 | `PerformTwoCorner` (`ChFi3d_FilBuilder_C2.cxx`) | `PerformMoreThreeCorner` (plate) |
| 3 | `PerformThreeCorner` (`ChFi3d_FilBuilder_C3.cxx`) | `PerformMoreThreeCorner` |
| ≥4 or degenerate | `PerformMoreThreeCorner` (`ChFi3d_Builder_CnCrn.cxx`, GeomPlate) | |

Inside `PerformThreeCorner` there is a cascade:
1. **The KPart direct corner.** `ChFiKPart_ComputeData::ComputeCorner` builds a sphere (`_Rotule`,
   `_Sphere`) or, when two of the fillets have equal radius and the "pivot" face is a plane square to
   the spine (`ToricCorner`), a torus.
2. **A rolling-ball fillet between the pivot stripe's surface and the opposite face,** walked along a
   circular guide (`ChFi3d_CircularSpine`). This is an ordinary blend computation with the fillet
   surface as one of its two supports.
3. **`GeomFill_ConstrainedFilling`.** A 3- or 4-sided Coons-type patch with tangency
   ("boundary with surface") constraints: `fil(11, 20)` gives max degree 11 and 20 segments.
4. **GeomPlate,** when no pivot is found.

`GeomPlate_BuildPlateSurface` (TKGeomAlgo) deforms an initial surface to satisfy G0, G1 and **G2**
curve constraints (`GeomPlate_CurveConstraint` order −1..2). It is a thin-plate solve over the
constraints sampled as points (`Plate/`). `GeomPlate_MakeApprox` then converts the result to a
B-spline within tolerance. The result meets its neighbors only within tolerance, and the edges carry
that tolerance.

**Parasolid, ACIS and ShapeManager (Fusion, SolidWorks, NX, Onshape).** The documentation is
user-level, but it is consistent:
- **Fusion.** Continuity is G1 or G2 per edge set (`continuity`, replacing `isG2` in the API). Corner
  type is Rolling Ball or Setback, and asymmetric fillets always get a setback corner
  ([Fusion Fillet](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-FILLET-SOLID.htm),
  [FilletFeatureInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/FilletFeatureInput.htm)).
  A rolling-ball corner can't be G2 (the ball is a sphere), so a G2 fillet set meeting at a vertex
  needs a setback-style patch for the corner to be smooth.
- **SolidWorks.** A "Curvature continuous" profile, plus setback parameters: one distance per edge
  from the vertex where "the fillet starts to blend into the three faces"
  ([Constant Size Fillets](https://help.solidworks.com/2019/english/SolidWorks/sldworks/r_constant_size_fillets.htm)).
- **Parasolid,** as T-FLEX documents it. Vertex blends with an offset (setback) for three or more
  edges, with optional transitional collar faces. Edges of opposite convexity get their vertex
  blended automatically. Processing order can be convex-first or concave-first
  ([T-FLEX vertex blend](https://www.tflex.com/help/eng/T-FLEX%20CAD/17/specifics_of_blending_sets_of_.htm)).
  KeyCreator suggests a setback of 1.5 times the largest radius
  ([KeyCreator Vertex Blend](https://help.kubotekkosmos.com/keycreator?id=161181)).
- **Rhino 9 (beta).** FilletEdge/BlendEdge setback corners are **multi-blends**: "a number of
  surfaces that meet at a common central point and close a hole with multiple edges". They have
  4- and 6-edge corners, and BlendEdge is the G2 variant
  ([McNeel forum, 2025-12](https://discourse.mcneel.com/t/rhino-beta-feature-filletedge-and-blendedge-corners/213727)).

**The literature.** Setback vertex blends are built as 2n-sided patches (Várady and Rockwood,
[*A geometric construction for setback vertex blending*, CAD 29(6) 1997](https://eprints.sztaki.hu/1451/);
Várady and Hoffmann, [*Vertex blending: problems and solutions*, 1998](https://eprints.sztaki.hu/1756/)).
G2 n-sided filling uses parabolic ribbons and transfinite patches (Salvi and Várady,
[*G2 surface interpolation over general topology curve networks*, CGF 33(7) 2014](https://diglib.eg.org/handle/10.1111/v33i7pp151-160)).
G1 n-sided holes are filled with Gregory or Coons-Gregory patches (Gregory 1974), or with n
rectangular patches around a center point (Hahn, *Filling polygonal holes with rectangular patches*,
1989; this is Rhino's multi-blend).

**Conclusion.** No kernel builds G1 or G2 corners by booleans of tool solids. Corners are surfaces
built against the stripes' end data, with tolerance at the seams.

### 1.2 Intersections with B-spline surfaces

#### (a) A B-spline against a plane, cylinder, sphere, cone or torus

Use the implicit form of the analytic surface, `f(x) = 0`, and work in the B-spline's own
parameters. This is OCCT's `IntPatch_ImpPrmIntersection` approach; luce-cad's `march.lucb` already
does it in 3D for analytic pairs.

```
g(u,v)   = f(S(u,v))                       # zero set = intersection, in S's domain
∇g       = (∇f·S_u, ∇f·S_v)
tangent  d = (-g_v, g_u) / |(-g_v, g_u)|   # in (u,v)
3D speed |T| = |S_u d_u + S_v d_v|
```

**Exclusion tests on Bézier pieces.** Extract the Bézier pieces by knot insertion and test each one:
- **Plane.** `g` is a polynomial of S's degree, and its Bernstein coefficients are
  `n·P_ij − d` (non-rational S). All coefficients of one sign means no intersection.
- **Sphere and cylinder.** `f` is quadratic, so `g` is a polynomial of twice the degree. Its Bernstein
  coefficients come from the degree-elevated product formula (Farouki and Rajan), or use a cheaper
  conservative test: the distance from the center or axis to the control net's box against the radius.
- **Rational pieces** (revolved profiles): test `f` on the homogeneous form, or just the control
  points' box against the surface.

**Start points.**
1. **Boundary points.** On each boundary iso-curve of each piece, and on each trim edge, find where
   `f(C(t))` changes sign. Use 1D sampling plus bisection, then Newton.
2. **Closed loops inside a piece.** A closed loop of `g = 0` encloses a critical point of `g`
   (∇g = 0) by Morse theory. If the Bernstein coefficients of `g_u` or of `g_v` keep one sign over the
   piece, no loop lies inside it. Otherwise subdivide until they do, or until the piece is below the
   tolerance. In a surviving piece, run Newton on `∇g = 0` to find the critical point, then walk from
   it along `∇g` to `g = 0` for a start. This is the parametric analogue of Sederberg and Meyers'
   normal-cone loop test (*Loop detection in surface patch intersections*, CAGD 1988).

**Tracing.** Predictor and corrector in (u,v):

```
h  = sqrt(8·δ/κ)           # δ: chord tolerance (≤ tol), κ: 3D curvature of the curve, estimated
                           # from the circle through the last three points (start with h = size/50)
uv' = uv + (h/|T|)·d
repeat ≤ 6: uv' -= g(uv')·∇g / |∇g|²        # Newton onto g = 0, minimal-norm step
accept if |g| < tol and the turn angle between steps < 10°; else halve h
```

Stop when the curve leaves the domain (solve exactly on the boundary iso-curve), when it comes back
within `h` of its start in the same direction (a closed loop), or when it reaches a start point
already used.

#### (b) Two B-splines

Use Barnhill and Kersey's four-parameter marching
([CAGD 1990](https://scholars.georgiasouthern.edu/en/publications/a-marching-method-for-parametric-surfacesurface-intersection-3/)),
which OCCT implements as `TKGeomAlgo/IntWalk/IntWalk_PWalking.cxx` with start points from
`IntPolyh/` (two triangulations intersected).

```
unknowns x = (u1, v1, u2, v2)
F(x) = [ S1(u1,v1) − S2(u2,v2) ;            # 3 equations
         (S1(u1,v1) − P0)·T0 − h ]          # step plane: distance h along the last tangent
J    = [ S1u  S1v  −S2u  −S2v ;  T0·S1u  T0·S1v  0  0 ]     # 4×4
T    = n1 × n2 / |n1 × n2|
```

For start points, subdivide both into Bézier pieces, keep pairs whose boxes overlap (a BVH, see §3),
and recurse until each piece is flat within the tolerance. Then intersect the two triangles of each
flat piece pair, and finish with Gauss–Newton on the three equations, using the minimal-norm
(pseudo-inverse) step.

#### (c) Tangential and near-tangential cases

This is the case that matters for G2 fillets.

- **Contact curves must never come from an intersection.** A blend's boundary iso-curves
  (`v = 0`, `v = 1`) lie on the faces it blends into by construction. Store them as edges with a curve
  on each face: OCCT keeps `ChFiDS_FaceInterference`, the curve on the face plus the curve on the
  blend. The boolean or stitcher must take them as given, as luce-cad already does for coincident
  edges.
- **How large the tolerance band is.** Let a profile leave a face with contact order k (G1: k = 2,
  G2: k = 3, G3: k = 4). Its distance from the face then grows like `c·s^k`, with `c ≈ κ/2`, `κ'/6`
  and `κ''/24`. Points within the arc length `s_band = (tol/c)^(1/k)` of the contact are numerically
  on the face. For r = 1, tol = 1e-7 and c ≈ 1/r^(k−1) (order of magnitude):

  | Contact | s_band |
  |---|---|
  | G1 | ≈ 4.5e-4 |
  | G2 | ≈ 8e-3 |
  | G3 | ≈ 4e-2 |

  So a G3 profile is "on" the face for about **4% of the radius**. Every crossing or sign change
  found inside that band is spurious. The rule: a root within `s_band` of a known contact edge snaps
  to that edge's end, which generalizes luce-cad's "crossings snap" rule. The same band explains the
  slivers in §4.
- **Tangent points of unknown curves** (|n1 × n2| < ε). Find the direction from the surfaces' second
  fundamental forms, written in one orthonormal frame (e1, e2) of the shared tangent plane with the
  normals oriented alike:

  ```
  (L1−L2)·a² + 2(M1−M2)·a·b + (N1−N2)·b² = 0,   t = a·e1 + b·e2
  ```

  The quadratic has three outcomes:
  - **No real root:** an isolated touching point. Emit a vertex, no curve.
  - **Two roots:** a branch point. Trace both directions.
  - **Identically zero:** the surfaces are tangent along a curve. Trace it with the tangent from the
    third-order terms, or better, reject it and require that such curves be known (as above).

  The theory is in Ye and Maekawa, *Differential geometry of intersection curves of two surfaces*,
  CAGD 16(8), 1999, and in Patrikalakis and Maekawa,
  [*Shape Interrogation*](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/), §5.8 (marching,
  singular points) and §6.4.

#### (d) Closed forms worth adding before general marching

- **A plane crossing a straight sweep.** An affine copy of the profile (already done).
- **Two straight sweeps of one profile, mirror images across a plane P.** Their intersection lies in
  P, so it is P ∩ sweep: an affine copy again. This happens when two equal G2 fillets meet at a
  corner of two faces symmetric about the bisector. A box corner with equal radii is the common case.
  This gives **G2 miters with no marching,** by the same argument luce-cad's crossing-cylinder
  ellipses use.
- **A profile revolved about an axis A, against any surface of revolution about A** (plane square to
  A, coaxial cylinder, cone, sphere, torus). Write the profile in its meridian plane as `(ρ(t), z(t))`
  and the other surface as `ρ = R(z)`; for a plane `z = c`, solve `z(t) = c`. Each root is a circle.
  The roots are those of a polynomial of the profile's degree, found by Bernstein subdivision plus
  Newton. Roots in the contact band snap (above). This extends the "surfaces of revolution about one
  axis" row of `intersect.lucb` to revolved B-spline profiles, and covers every G2/G3 rim.

### 1.3 A G2 corner where three G2 fillets meet

> **As built (2026-10-08):** the single trimmed patch below was tried and did not converge closely
> enough. A numpy prototype gave boundary errors of 1e-3 to 1e-4 of the corner's size at m = 8–16,
> and 1e-6 only at m = 24 for G0 alone. luce-cad instead fills the hole with six Bezier quads, one
> per hole corner, with exact outer sides (rows on the sweep or in the face's plane). The seams are
> G1 by fixed coefficients (μ = 1, ν = t, the six-valent vertex's 2 cos 60°), and G2 is held by
> least squares. See MODELING.md, "Setback corners". Degree 5 became the element for everything
> freeform, so G3 is not degree 7 as below. Its section is a degree 5 B-spline of three spans,
> eight control points, and its quads are degree 5 nets of two spans.


A ball corner is only G1 against G2 fillets: the fillets' end sections are not circular arcs, so no
sphere is tangent to them. A smooth G2 corner needs a **setback**.

**The construction.** It is shown for three planar faces P1, P2, P3 at a vertex V, and edges
e1, e2, e3 with G2 profiles.

1. **Setback.** Each stripe i is cut by the plane through V + s_i·ê_i square to ê_i, where ê_i is the
   unit edge direction away from V. Default `s_i = 1.5·max_j d_j`, where d_j is fillet j's contact
   distance `r / tan(θ_j/2)`. The cut is the stripe's end section, an affine copy of the profile:
   curve `A_i`, of degree 5 for G2.
2. **The face curves.** On face P_k, between stripes i and j, the face curve `B_k` runs from stripe
   i's contact end to stripe j's contact end. Make it a planar Bézier of degree 5. For G2 continuation
   of the straight contact lines, put three control points on each end along the contact line, as the
   profile already does. Its interior points set the corner's footprint on the face.
3. **The hole.** It is bounded by six curves alternating A, B, A, B, A, B: Várady's 2n-sided setback
   patch with n = 3.
4. **The boundary data.** Each boundary sample k carries a point Q_k, a unit normal N_k, a boundary
   tangent p̂_k and second-order data:
   - **On B (a plane):** the second fundamental form is 0.
   - **On A (a straight sweep):** in the frame (ê, p̂), II = diag(0, κ_profile). The ruling direction
     ê has zero normal curvature, and the mixed term is 0, because the normal of a cylindrical
     surface doesn't change along the ruling.
5. **The fit.** Fit one tensor-product B-spline `S(u,v)` of degree (5,5) with m×m control points, m = 8
   to start. Map the parameters by projecting the hole's boundary onto the plane square to
   `n̂ = normalize(N1 + N2 + N3)` and scaling into [0,1]². First check that the projected boundary is
   a simple polygon and that every boundary normal has `N·n̂ > 0`, otherwise refuse. Each boundary
   sample gets (u_k, v_k) and the inward domain normal `w_k` (a unit vector in (u,v)). Write
   `D_w S = S_u w_u + S_v w_v` and `D_ww S = S_uu w_u² + 2 S_uv w_u w_v + S_vv w_v²`. All of these are
   linear in the control points.

   | Constraint | Equations (each linear in P_ij) |
   |---|---|
   | G0 | `S(u_k,v_k) = Q_k` |
   | G1 | `N_k·S_u = 0`, `N_k·S_v = 0` |
   | Cross direction fixed (A only) | `p̂_k·D_w S = 0`, so the domain normal maps onto ê |
   | G2 on A | `N_k·D_ww S = 0` (II(ê,ê) = 0), `N_k·D_t D_w S = 0` (II(ê,p̂) = 0) |
   | G2 on B | `N_k·D_ww S = 0`, `N_k·D_t D_w S = 0` |

   II along the boundary tangent matches automatically once G0 and G1 hold (Meusnier). The last
   column is the key result: **because the neighbors are planes and straight sweeps, G2 is linear.**
   Fixing the cross-boundary direction at A costs a little freedom and removes the only nonlinearity.

   Minimize

   ```
   E = Σ_k w_c·|constraint residuals|² + λ·∫∫ (|S_uu|² + 2|S_uv|² + |S_vv|²) du dv
   ```

   - **Fairing.** The integral is thin-plate energy, an exact quadratic form in the control points
     (integrals of B-spline basis products) over the whole square.
   - **The solve.** Use the normal equations or QR on 3m² ≈ 200 unknowns. The coordinates couple
     through N·, so solve them jointly, densely.
   - **Weights.** w_c is about 1e4 relative to λ. Raise it until G0 is below tol/10.
6. **Checking.** At 4× the sample density, require all of:
   - G0 < tol;
   - normal angle < 0.05°;
   - relative curvature mismatch < 5%.

   On failure, insert knots (m += 2) and refit; give up after m = 16. OCCT's GeomPlate criteria are the
   same idea (`GeomPlate_PlateG0Criterion`, `G1Criterion`).
7. **Topology.** The corner face is S trimmed by the six boundary curves, whose curves on S come from
   the projection. Its edges are the exact A and B curves, with the fit deviation as edge tolerance
   (OCCT stores it the same way). The three planar faces are trimmed by B_k: a planar 2D operation
   (§3.2). The stripes end at A_i.

**The cost** is the fit, done once per corner: about 200 unknowns, cheap. The work is in
(i) parameterization checks and (ii) the topology insertion, which is why §3.2 comes first.

**For G3,** add the third-derivative conditions (`N·D_www S = 0` and the matching mixed terms), and use
degree 7 and m = 10. These are linear by the same argument.

**Alternatives considered:**
- **A Coons/Gregory patch with exact boundaries** (OCCT's `GeomFill_ConstrainedFilling`, 3–4 sides):
  exact G0, but G1 only, and it needs a 4-sided split of the 6-gon.
- **n quads around a center (Rhino's multi-blend, Hahn):** exact G0, but G2 across the inner seams
  needs the vertex-enclosure conditions at the center. Leave it until a fit is not good enough.
- **A Salvi–Várady G2 transfinite patch:** best quality, much more code.

The single fitted patch is the smallest correct thing.

**When the setback is 0, or the user asks for Rolling Ball with G2:** build the G1 sphere corner (the
existing `Rotule`) and report that the corner is only G1. Fusion only promises G2 along the fillet
transitions.

### 1.4 A G2 fillet on a circular rim

Take a rim where a plane meets a cylinder of radius R, with axis A. The G2 profile is a degree-5
Bézier in the meridian half-plane, from the contact point on the plane to the contact point on the
cylinder, built exactly as for a straight edge (same control-point fractions). The surface is
**profile × full circle**: a NURBS surface of revolution. In u it is the circle (rational quadratic,
9 points, 4 spans); in v it is the profile. It is exact; no approximation is involved.

The two routes:
- **By booleans:** the tool is the region between the profile and the corner, revolved. Its faces are
  the revolved profile, an annulus in the plane and a cylinder band, and every pair is coaxial, so
  they are circles (§1.2(d)). The only hazard is the G2 contact circles, where the roots are
  tangential. Snap them: the profile's end parameters are known exactly.
- **Locally (better):** shrink the planar face's circular loop to radius `R + d` (a hole's rim) or
  `R − d` (a boss's rim), cut the cylinder band at height `±d'`, and insert the revolved face between
  the two contact circles. There are no intersections at all.

**G2 rims meeting G2 straight fillets** (a slot's end, say: line, arc, line, a tangent chain) need
nothing at the joints. Both profiles are built from the same normalized control fractions, so the
sweep's end section and the revolve's meridian at the joint are the same curve, and the joint is
G2-consistent along the chain. The tangent chain is the stripe (§5.1).

### 1.5 Verdict: booleans of tool solids for G2 corners?

**No,** except in the symmetric miter case of §1.2(d):
- **The contact is tangential.** A G2 corner tool's faces meet the body's faces with third-order
  contact along whole curves. Every intersection there falls inside the s_band of §1.2(c). The result
  can only be right if the contact curves are injected as known, and once they have to be known, the
  boolean is doing nothing but stitching.
- **No tool shape gives a smooth G2 corner.** Combining G2 tools gives creases (miters) or needs a
  corner tool, and the corner tool's surface is exactly the patch of §1.3.

Build corners directly, as OCCT does: stripes, then corner surfaces, then topology. For the planar
case luce-cad can do this without OCCT's TopOpeBRep machinery (§3.2).

## 2. Inside (concave) fillets meeting other fillets

### 2.1 What surfaces result

Notation: a vertex with three edges. **+** is convex (material removed), **−** is concave (material
added). All fillets have radius r unless stated. R is the radius of a filleted edge that another
fillet runs around.

| Edges at the vertex | Typical part | Corner surface |
|---|---|---|
| + + + | Box corner | Sphere radius r, center at the offset-plane meet (done: `Rotule`) |
| − − − | Inside a pocket's corner | **Sphere** radius r, the ball in the air: the same construction mirrored. The patch is the sphere part facing V; material is added. |
| + + (third sharp) | | Miter: two cylinders crossing in ellipses (done) |
| − − (third sharp) | Pocket floor and wall, vertical corner sharp | Miter: the same ellipses, union instead of subtraction |
| + − − | A boss on a plate: vertical convex edge, two base edges concave | **Torus.** The ball rolls between the plate and the convex vertical fillet (cylinder R): axis vertical, major radius R + r, minor r, a quarter turn. The vertical fillet stops at the height r above the plate where it meets the torus tangentially. |
| + + − | Top edges of an L-block over its inside vertical edge | **Torus** with major radius R − r around the concave vertical fillet (cylinder R); a **sphere** when R = r; **impossible** as a rolling ball when r > R (a spindle torus that crosses itself) |
| + − (third sharp) | | The concave tool's end meets the convex blend: OCCT's `PerformOneCorner` cases 2–3, a small GeomFill cap or an extended end face |

The + − − and + + − rows are the same KPart case: a plane against a cylinder whose axis is normal to
the plane (`ChFiKPart_ComputeData_FilPlnCyl.cxx`: offset plane ∩ offset cylinder `R ± r` = the
circular spine, giving a torus). OCCT reaches it at corners through
`ChFiKPart_ComputeData::ComputeCorner(..., minRad, majRad, ...)`, which demands a plane and builds the
torus about the pivot fillet's section circle.

### 2.2 How OCCT decides

`ChFi3d_FilBuilder::PerformThreeCorner` analyses the concavities first, in its own words: "two
concavities identic and one inverted", or "three concavities identic".
- **Mixed (two alike, one inverted):** the inverted stripe is the **pivot**. The corner is the blend
  between the pivot fillet's surface and the face opposite it: the KPart torus when it applies,
  otherwise a rolling-ball walk along a circular guide, otherwise `GeomFill_ConstrainedFilling`.
- **All alike:** `SearchPivot` picks the stripe whose end section is cut by the other two. If none is
  found, OCCT calls `PerformMoreThreeCorner` (plate).

`PerformTwoCorner` tests `ToricRotule`: three planes, two constant fillets of equal radius, and the
third face square to the other two. That gives a torus between them; otherwise it uses GeomFill.

### 2.3 Fusion and Parasolid

Parasolid blends a vertex automatically when edges of opposite convexity meet there, and lets the
caller pick convex-first or concave-first (T-FLEX, above). Fusion's "Rolling Ball" corner gives the
torus results of the table. Its "Setback" gives a patch, and asymmetric fillets always use setback.
When r > R in the + + − case, Fusion reports a failure, or with Setback makes a patch (observable
behaviour, not documented). Report it as an error first.

### 2.4 The algorithm for luce-cad: order, chains and three new tools

luce-cad gets most of the table by **ordering plus chain propagation,** keeping the
one-boolean-per-stripe structure:

```
blend(body, edges, r):
    classify each vertex V where ≥2 chosen edges meet: signs s(e) ∈ {+,−}, n = count
    order = []
    for each vertex of type (+,−,−): put its + edge before its − edges
    for each vertex of type (+,+,−): put its − edge before its + edges
    the rest: +-edges first, then −-edges (today's rule)
    topologically sort the edges by these constraints;
    a cycle (+,−,− at one end and +,+,− at the other on one edge) → refuse this pair for now
    for each edge e in order:
        chain = tangent_chain(body', e)      # after earlier blends, e may now continue
                                             # through new arcs (§5.1)
        tool  = sweep(profile, chain)        # line → cylinder, arc → torus (rim tool), joined G1
        body' = body' − tool  or  body' ∪ tool
```

- **(+,−,−).** After the + edge is filleted, the plate/boss base edge chain is line, quarter arc
  (radius R), line. The concave tool sweeps: prism, torus (a rim tool revolved a quarter turn about
  the fillet's axis), prism. A partial rim tool means a revolve through an angle: MODELING lists
  partial revolves as missing, and this needs them.
- **(+,+,−).** After the − edge is filleted (radius R), the top convex chain is line, concave arc,
  line. The torus around the arc has major radius `R − r`:

  | Radii | Tool |
  |---|---|
  | R > r | Torus |
  | R = r | Sphere of radius r centered on the arc's axis (the torus with major radius 0) |
  | R < r | Refuse: "fillet radius r exceeds the inside radius R" |

- **(−,−,−).** `corners.lucb` already handles "all inside". Check that its union tool, (cell beyond
  the planes) minus ball, is mirrored correctly, and add a test.

None of this needs new SSI. Plane/torus with the axis across the plane, coaxial cylinder/torus and
torus/torus about one axis are all in the table. The sweep joints are tangent, so neighboring tool
faces meet along shared circles or lines, and these must be imprinted as shared, like the existing
vertex/edge interference.

## 3. Performance

### 3.1 How OCCT does many edges at once

`ChFi3d_Builder::Compute` (`ChFi3d_Builder.cxx`):
1. Build every stripe (`PerformSetOfSurf`). Each stripe is a list of `ChFiDS_SurfData`: a blend
   surface, the curves on it and on both faces, and its end points.
2. Build every vertex corner (`PerformFilletOnVertex`).
3. Check stripes against each other (`ChFi3d_StripeEdgeInter`).
4. Write everything into one `TopOpeBRepDS` data structure (`ChFi3d_FilDS`): the new surfaces,
   curves and points, plus *interferences* saying "curve C splits face F here".
5. Run `TopOpeBRepBuild_HBuilder` (`myCoup->Perform(myDS)`, then `MergeSolid(..., TopAbs_IN)`) once.
   It splits every touched face along its contact curves and rebuilds the solid.

**Untouched faces are never visited, and there is no face/face intersection at all.** The contacts
are known from the blend computation. Only the blend-to-blend checks are new intersections.

Parasolid and ACIS do the same at a higher level: "local operations" that replace face geometry and
re-trim neighbors. CAD-MODELING-STUDY §5 phase 6 already lists them.

### 3.2 Local face replacement for planar-faced bodies with cylinder and sphere blends

> **As built (2026-10-08):** `local_blend.lucb` and `local_corners.lucb`. Corners are an end (cut by
> the third face), a miter (symmetric only), a ball, or a setback patch, all outside or all inside.
> Faces are turned by propagation from the body's, and the result is checked closed. Anything else
> falls back to the booleans. Every edge of a box at G2 went from 1 s to 44 ms.

This generalizes `rounded.lucb` (every edge of a convex polyhedron) to any chosen set of edges between
planes. It is about OCCT's `ChFi3d` + `TopOpeBRepBuild` for the KPart cases, in a few hundred lines.

```
local_blend(B, E, r):                        # E: chosen edges, all between planar faces
  # 1. Stripes (generator keys: edge id; contact keys: (edge, face))
  for e in E with faces F1 (n1·x = d1), F2 (n2·x = d2), convexity σ ∈ {+1,−1}:
      θ      = interior dihedral angle
      L_e    = {n1·x = d1 − σr} ∩ {n2·x = d2 − σr}          # ball-center line
      c1, c2 = L_e moved by +σr·n1, +σr·n2                    # contact lines on F1, F2
      t      = r / tan(θ/2)                                   # contact distance from e in each face
      blend  = cylinder(axis L_e, radius r)
  # 2. Vertices: decide each corner (sphere, miter, torus, setback patch, or a sharp end)
  for each vertex V of an edge in E:
      classify (signs, how many chosen, how many sharp)
      compute the end of each stripe at V:
          sharp end face G (plane): end curve = cylinder ∩ G   (circle/ellipse arc, exact table)
          sphere corner:            end = great circle in the plane ⟂ L_e through the center
          miter:                    end = ellipse in the bisector plane
          unsupported:              mark V → fallback
  # 3. Trim faces in 2D, in each plane's own coordinates
  for each planar face F touched by E:
      for each edge a of F's loops:
          if a ∈ E: replace a's line by the contact line c_F(a)
          else:     keep a's line (sharp)
      new vertices = intersect consecutive lines; at a corner vertex use the corner's
                     boundary on F (a point for a sphere, an arc end for a miter or torus)
      validate: same orientation as before, every edge length > tol, no self-crossing,
                inner loops (holes) entirely inside the new outer loop with clearance > tol
      any failure → mark F → fallback
  # 4. Assemble: faces keyed by generator; edges keyed by (generator pair);
  #    vertices keyed by (generator triple). Each key is computed once, so shared edges are
  #    shared by construction (the pave-block idea without intersections).
  # 5. Validate: each edge used twice in opposite directions; V − E + F = 2 − 2g as before;
  #    else fallback.
  fallback: run today's per-edge boolean path for the marked edges only, on the local result
```

What it gives:
- **Speed:** O(touched faces), with no SSI, no splitting and no classification.
- **Exactness:** every new vertex is the meet of three known surfaces.
- **The G2 corners of §1.3:** step 2 calls the patch fit and step 3 trims by the B curves.

The validation in step 3 also detects the face-consumed and edge-shorter-than-setback cases of §5:
those are the edges whose new length is ≤ 0.

### 3.3 Cheap speedups to the existing boolean

In order of payoff:

1. **Tool-box locality.** Before the face/face loop, a body face whose box doesn't meet the tool's
   box (plus tol) can't be split, and is entirely outside the tool. Copy it through untouched and
   skip classifying it. With one small tool per edge, almost every face of a large body takes this
   path, so each boolean costs O(faces near the edge).
2. **Classification by connected blocks.** After splitting, group the pieces into blocks connected
   through edges that are **not** section edges. All pieces of a block are on the same side, so
   classify one point per block. OCCT does this with `BOPTools_AlgoTools::MakeConnexityBlocks` and
   `BOPAlgo_Tools::ClassifyFaces` (TKBO), which also checks boxes before classifying. Ray casts drop
   from one per piece to one per block, usually 2–4.
3. **Pair culling** in O(n log n + k) instead of the double loop over faces in `boolean.lucb`:
   - **Sort and sweep:** sort the face boxes by min-x, sweep, and test y/z overlap on the active set.
     It is about 30 lines.
   - **A BVH** (median split on the longest axis) when one argument is much larger. OCCT uses a BVH
     (`BOPTools_BoxTree`, used by `BOPDS_Iterator`), and optionally oriented boxes; OBB reduces false
     pairs on long slanted faces such as fillet bands.
4. **Exact classification for B-spline faces.** `rays.lucb` already classifies by exact rays for
   analytic faces. It falls back to a mesh when a solid has B-spline faces, which G2 fillets always
   create. Add ray/B-spline: Bézier pieces, a ray-box cull, then 2D Newton on
   `S(u,v) − (o + t·d) ⟂ d` from the piece's center, then the trim winding test. This removes the last
   tessellation from booleans.
5. **Batching tools (an N-argument General Fuse).** Tools that don't touch each other (box test) can
   go into one boolean: `body − (T1 ∪ T2 ∪ …)` as a General Fuse of N+1 arguments, with each piece
   classified against every argument. Selection:
   - keep body pieces outside every tool;
   - keep tool pieces inside the body and outside the other tools, reversed.

   OCCT's `BOPAlgo_Builder` and `BOPAlgo_CellsBuilder` take N arguments; see the
   [Boolean spec](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html).
   Tools that touch (corners) are best kept apart until (1)–(3) are done: they are where the risky
   intersections are, and sequential order makes failures attributable to one edge.
6. **Caches across operations.** Keep face boxes and charts in the shape, invalidating only the faces
   an operation produced.

Order (1), (2), (3) first: they are small changes inside the existing structure and should make
"fillet 20 edges of a 200-face part" roughly 20× local work instead of 20× global work.

## 4. Display tessellation slivers

### 4.1 Why they appear

1. **G2/G3 contact is a cusp.** Where a trim curve meets a boundary tangentially, the face's domain
   has a 0° interior angle there. The two chains separate like `c·s^k` (§1.2(c)). Any triangulation
   that respects both chains has triangles whose angle → 0 at the cusp. Within s_band the chains
   coincide to within tolerance, so a grid line or an ear can cross between them, and the triangle
   flips. Better ear choice can't fix this: the domain itself is degenerate.
2. **Poles.** In a (longitude, latitude) chart, the pole is a whole edge of the domain mapped to one
   point. Triangles touching it have zero 3D area or flip. Rows near the pole are crowded in 3D but
   not in (u,v).
3. **float32.** Vertices stored in float32 relative to a far-away origin, or triangles whose area is
   below about (ulp·size)², change orientation when rounded.

### 4.2 What other tessellators do

**OCCT BRepMesh** (`TKMesh/BRepMesh/`;
[mesh guide](https://dev.opencascade.org/doc/overview/html/occt_user_guides__mesh.html)):
- **Edges first.** Each edge is discretized once (`BRepMesh_EdgeDiscret`, `BRepMesh_CurveTessellator`)
  and both faces use those nodes, so faces conform.
- **Merging and size limits.** `BRepMesh_VertexTool` merges nodes within a (u,v) tolerance.
  `IMeshTools_Parameters::MinSize` (default 0.1 × the linear deflection, `RelMinSize`) stops
  refinement below a size, and `AdjustMinSize` ties it to the edges.
- **Domain scaling.** `BRepMesh_DefaultRangeSplitter::computeDelta` scales (u,v) by
  `range / surface length` so that Delaunay sees an approximately isometric domain. This is a
  diagonal metric.
- **Spheres.** `BRepMesh_SphereRangeSplitter` places interior nodes in rows at
  `0.7 × ArcAngularStep(R, deflection, angle, MinSize)`, staggering every other row by half a step,
  which gives near-equilateral triangles. The poles are degenerated edges of the face, discretized
  like any edge.
- **Repair and refinement.** `BRepMesh_ModelHealer` finds self-intersecting discrete wires (from
  close or tangent edges) and re-discretizes those edges finer. `BRepMesh_DelaunayDeflectionControlMeshAlgo`
  splits triangles that deviate from the surface.

**Delaunay refinement** (Ruppert/Chew; Shewchuk's
[Triangle](https://www.cs.cmu.edu/~quake/triangle.html)) guarantees a minimum angle of about 20.7°
except at small input angles. Shewchuk's *concentric shells* handle those: split the segments meeting
at a small-angle apex at power-of-two distances from it, so refinement stops, and accept the
unavoidable small angles in the apex's own triangles
([*Mesh generation for domains with small angles*, SoCG 2000](https://people.eecs.berkeley.edu/~jrs/papers/small.pdf)).
The orientation tests use Shewchuk's robust predicates
([robust.html](https://www.cs.cmu.edu/~quake/robust.html)); luce-cad's `trim_predicates.lucb` already
does the same.

**Gmsh** meshes parametric surfaces with frontal-Delaunay in (u,v) under the metric of the first
fundamental form, which is the anisotropic form of BRepMesh's diagonal scaling
([reference manual](https://gmsh.info/doc/texinfo/gmsh.html)).

Parasolid and ACIS facetting is not public in detail. Their options (minimum facet width, maximum
facet width, chord and angle tolerance) show the same strategy: a lower size bound plus edge-first
conformity.

### 4.3 What to do in luce-cad's grid-cut tessellator

1. **Detect cusps.** At each trim vertex, compute the angle between the incoming and outgoing chains
   in the lifted (3D) tangent plane. An angle below about 2° (and exactly 0 for known G1/G2/G3 contact
   vertices, which the modeler can flag) marks a cusp.
2. **Mesh the cusp region as a zipper strip.** It replaces grid cutting there:

   ```
   cusp at vertex V with chains a(s), b(s) (arc length from V)
   s_end = first s where separation(a(s), b(s)) ≥ the local grid pitch, or the chain's end
   s_1   = smallest s with separation ≥ 20·tol           # beyond the band; avoids float folding
   samples s_1 < s_2 < … < s_end, geometric ratio ≈ 1.5 (finer near V)
   triangles: (V, a(s_1), b(s_1)) then quads (a_i, a_{i+1}, b_{i+1}, b_i) split on the shorter diagonal
   ```

   Pairing by arc length means both chains advance together, so no triangle can cross between them.
   Only the first triangle is thin, by necessity. Exclude the region `s < s_end` from the grid cut;
   it becomes a hole bounded by `a`, `b` and the segment `a(s_end)–b(s_end)`. The samples on `a` and
   `b` are also the shared edge samples of the neighboring faces, so conformity holds.
3. **Boundary spacing rule.** Two boundary samples belonging to different chains, closer than
   `ρ = 0.25 × the local pitch` but farther apart than tol, are a sliver seed. Move the sample on the
   more flexible chain: an interior trim before a CAD edge, and never a CAD vertex. Do it in the edge
   discretization, before faces see the samples, so both sides agree. This is BRepMesh's
   ModelHealer idea, done ahead of time.
4. **Sphere corner patches.** Don't use a (u,v) chart for a 3-sided sphere patch, such as a rounded
   box corner bounded by three great-circle arcs.
   - Mesh it on a **barycentric grid,** with points interpolated spherically:
     `P(i,j,k) = C + R·normalize(i·A + j·B + k·C')/n` for corners A, B, C' (unit vectors from the
     center C), so `i + j + k = n`.
   - For a boundary point to lie on the great circle, use **slerp** along each side, so the samples
     sit at equal angles and match the neighboring cylinder's arc samples exactly.
   - For the interior, normalize the barycentric combination.

   Every triangle is close to equilateral, there is no pole and no seam, and a patch beyond an
   octant (an obtuse corner) gets n chosen from its longest arc. A general trimmed sphere face keeps
   `sphere_chart.lucb`'s rotated chart: put the pole on the side opposite the patch's centroid.
5. **Torus and cylinder strips** (blend faces between two contact lines) are structured: rows along
   the spine and columns across the profile. Mesh them as a grid in their own (u,v) with the shared
   edge samples as rows. Never cut them with an unrelated grid.
6. **float32.**
   - Translate each body's vertices by its box center before converting to float32 (ideally each
     face's, if the renderer takes a per-draw offset).
   - Take normals from the exact surface, not from triangle cross products.
   - After conversion, recompute each triangle's orientation in float32 against its float64
     orientation. Where it flips (it can only happen at near-degenerate triangles), collapse the
     shortest edge of that triangle if it is interior. If it is on a boundary, mark the triangle as
     degenerate and let the shading normals carry it. Never move a CAD-edge vertex.

## 5. Other edge cases, by how often users hit them

| # | Case | What kernels do | What luce-cad should do |
|---|---|---|---|
| 1 | **Tangent chains** (the edges of a rounded slot, edges around an earlier fillet) | OCCT gathers mutually tangent edges into one spine (`ChFi3d_Builder_1.cxx`, "find all mutually tangent edges"; `ChFiDS_Spine`). Fusion's Tangent Chain is on by default. | Gather edges whose tangents agree at shared vertices and whose two face pairs continue G1. Sweep one tool along the chain: line → prism, arc → torus or rim revolve, joined tangentially. Needed by §2.4. |
| 2 | **Blends on plane/cylinder edges along a ruling** (a slot side meeting a cylinder tangent plane, a D-shaft flat) and **cone rims** | KPart: `FilPlnCyl` (parallel case gives a cylinder), `FilPlnCon`, `ChPlnCon` | Plane/cylinder with a parallel axis: offset plane ∩ offset cylinder (R ± r) = line(s), so the blend is a cylinder parallel to the axis. Cone rim (plane square to the axis): the ball center circle is where the offset plane meets the offset cone (a cone whose apex has moved along the axis by r/sin α); the blend is a torus. All closed form. |
| 3 | **Radius too large: a face is consumed, or an edge is shorter than its setback or contact distance** | OCCT mostly fails with a stripe error (`ChFiDS_ErrorStatus`). It can continue the walk onto the next face ("bypass of obstacle", `ChFi3d_Builder_6.cxx`). Parasolid has overflow modes: smooth overflow, retain cliff edges (a ball tangent to one face, rolling on the kept edge), and notch ([T-FLEX overflow](https://tflex.com/help/eng/T-FLEX%20CAD/17/blendedges_overflowprocessing.htm)). Fusion fails, or rolls onto the next face in simple cases. | First, detect it in §3.2 step 3 (a trimmed edge of length ≤ 0) and in the boolean path (a contact line leaving its face), with a precise error: "radius 5 is larger than face X allows (max 3.2)". Compute the max from the 2D offset. Later, the "rolls onto the next face" case for planes: when contact line c1 leaves F1 across edge a into F1', recompute the stripe against F1' (a new plane/plane cylinder); the junction is a ball corner touching the edge a. |
| 4 | **Holes or other features near the edge** (a contact line crosses an inner loop) | Kernels split the stripe where it meets the hole (TopOpeBRep handles it). It works in Fusion when the hole's rim is outside the blend surface. | The boolean path handles this already. The local path detects it (an inner loop crossing the new outer loop) and falls back. |
| 5 | **Edges between two curved faces** (a pipe tee: cylinder/cylinder; a boss on a cylinder; cylinder/sphere) | A rolling-ball walk: `BRepBlend_Walking` with `BlendFunc_ConstRad`, approximated by `AppBlend` | The ball center curve is the meet of the two offset surfaces (an offset cylinder is a cylinder of R ± r), which `march.lucb` already traces. At each center point, the contact points are the feet on each surface (closed form for quadrics). Interpolate the circular arcs into a B-spline (or a rational sweep). Booleans with it then need §1.2(a). |
| 6 | **Chamfer corner setback triangle** (Fusion's default chamfer corner "Chamfer"; "Miter" is what luce-cad makes) | Fusion: Chamfer, Miter, Blend corner types ([Fusion Chamfer](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-CHAMFER-SOLID.htm)). OCCT: `ChFi3d_ChBuilder_C3.cxx`, planar where it can, else `GeomFill_ConstrainedFilling`. | At a three-chamfer corner, each chamfer face is cut by the plane through the three points where neighboring chamfers' edges meet the original edges. A three-chamfer corner of equal distance on a box: the triangle through the three points at distance d along each edge from V, an extra planar face. In §3.2 this is one more corner type. With booleans, a tetrahedral corner tool. |
| 7 | **Seam, smooth and degenerate edges** | Fillets aren't offered on smooth (G1) edges, and seams aren't edges to the user. OCCT rejects edges whose faces are tangent (`ChFi3d_Builder_1.cxx`, tangent-face tests). | Exclude edges whose two faces are the same face (seams) or are tangent along the edge (G1). "Every edge" already takes only sharp straight edges. |
| 8 | **Variable radius** | OCCT `BlendFunc_EvolRad` with a radius law. Fusion: points along the edge. | Linear radius on plane/plane: the envelope of balls of linearly varying radius along a line is a **cone** (exact). A general law: a B-spline sweep of circular arcs whose radius follows the law, with the cross-section plane normal to the ball-center path C'(t), not to the edge. |
| 9 | **Setback corners and unequal radii at three-edge corners** | OCCT: pivot plus torus, else GeomFill or plate. Fusion: setback. | §1.3's fit with the G1 rows only (G2 rows dropped) gives G1 setback corners for any radii, from the same code. |
| 10 | **More than three edges at a vertex** (a pyramid apex, the end of a ridge) | OCCT `PerformMoreThreeCorner` (GeomPlate). Rhino 9: 4- and 6-edge multi-blends. | The same fit over a 2n-sided hole. Parameterize by projecting along the mean normal; refuse when it isn't injective. |
| 11 | **Fillet radius larger than a concave face's curvature radius** (the + + − case with r > R; a convex fillet along a concave cylinder of smaller radius) | Offset surface crosses itself; kernels fail | Detect it in the stripe construction (offset radius ≤ 0 or a spindle torus) and report "radius exceeds the inside curve's radius R". |
| 12 | **Non-manifold input or output** | Kernels refuse | Already refused (edge used ≠ 2). |

## 6. Recommended implementation order

The days below are focused engineering days for one person or agent, including tests.

| Step | Work | Unlocks | Effort |
|---|---|---|---|
| 1 | Boolean speedups §3.3 (1)–(3): tool-box locality, connexity-block classification, sort-and-sweep pair cull | Fast sequential fillets now; everything later runs faster | 2–3 d |
| 2 | Tangent chain gathering and chain sweeps (prism + partial-revolve torus joined G1); partial revolve as a tool primitive | §2.4, §5 #1, slot and boss chains | 4–5 d |
| 3 | Mixed corners by ordering (§2.4): (+,−,−) torus, (+,+,−) torus/sphere/refuse; a (−,−,−) test | Inside-corner fillets meeting others | 2–3 d |
| 4 | Precise "radius too large" diagnostics (§5 #3, #11) | Users understand failures | 1–2 d |
| 5 | Tessellation: cusp zipper, boundary spacing rule, sphere barycentric patches, float32 local origin and flip check (§4.3) | No slivers on current G2/G3 and ball corners | 4–6 d |
| 6 | KPart rows: plane/cylinder along a ruling, cone rims, chamfer setback triangle (§5 #2, #6) | The most common remaining fillet and chamfer refusals | 3–4 d |
| 7 | G2/G3 rims: revolved profile surface plus coaxial closed-form intersections (§1.2(d), §1.4); G2 symmetric miters by the bisector plane | G2 on holes and bosses; G2 two-fillet corners in the common case | 3–4 d |
| 8 | Local face replacement for planar bodies (§3.2), with boolean fallback per edge | Fast multi-edge fillets; the base for direct corners | 6–8 d |
| 9 | Ray/B-spline classification (§3.3 (4)) | No meshing inside booleans | 2 d |
| 10 | B-spline/implicit SSI in (u,v) (§1.2(a), (c)) | B-spline faces against analytic faces in booleans: G2 fillets meeting holes and other features; extruded splines cut by cylinders | 6–8 d |
| 11 | Setback corner fit (§1.3): G1 first (§5 #9), then G2 and G3 rows; then n > 3 (§5 #10) | G2/G3 three-fillet corners; unequal-radius corners; 4+ edge vertices | 8–12 d |
| 12 | B-spline/B-spline marching (§1.2(b)) | G2 fillets crossing each other or other spline faces in general | 8–12 d |
| 13 | General rolling-ball blends on curved faces (§5 #5), then variable radius (§5 #8) | Pipe tees, bosses on cylinders, variable fillets | 10–15 d |
| 14 | Overflow: continue onto the next face (§5 #3) | Large radii on small faces | 5–8 d |

Steps 1–7 (about 20–27 days) remove the refusals users meet most, with no new general SSI. Step 8 is
the architectural turn: from then on fillets are local operations with booleans as the fallback,
which is how ChFi3d and Parasolid work. Steps 10–12 bring general B-spline SSI, needed for G2 corners
interacting with arbitrary features.

## 7. Sources

- **OCCT (LGPL), read 2026-10-08:**
  - [TKFillet](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKFillet):
    `ChFi3d/ChFi3d_Builder.cxx` (`Compute`, `PerformFilletOnVertex`), `ChFi3d_Builder_C1.cxx`
    (`PerformOneCorner`, `PerformIntersectionAtEnd`), `ChFi3d_FilBuilder_C2.cxx` (`ToricRotule`,
    `PerformTwoCorner`), `ChFi3d_FilBuilder_C3.cxx` (`ToricCorner`, `PerformThreeCorner`, concavity
    and pivot analysis), `ChFi3d_Builder_CnCrn.cxx` (`PerformMoreThreeCorner`, GeomPlate),
    `ChFi3d_Builder_6.cxx` (bypass of obstacle), `ChFi3d_ChBuilder_C3.cxx`, `ChFiKPart/*`,
    `BlendFunc/*`.
  - [TKGeomAlgo](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKGeomAlgo):
    `IntPatch/IntPatch_ImpPrmIntersection.cxx`, `IntPatch_PrmPrmIntersection.cxx`,
    `IntWalk/IntWalk_PWalking.cxx`, `IntPolyh/`, `GeomPlate/`, `GeomFill/GeomFill_ConstrainedFilling.cxx`.
  - [TKBO](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKBO):
    `BOPDS/BOPDS_Iterator.cxx`, `BOPTools/BOPTools_BoxTree.hxx`,
    `BOPTools_AlgoTools::MakeConnexityBlocks`, `BOPAlgo/BOPAlgo_Tools::ClassifyFaces`,
    `BOPAlgo_CellsBuilder`.
  - [TKMesh](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKMesh):
    `BRepMesh_SphereRangeSplitter.cxx`, `BRepMesh_DefaultRangeSplitter.cxx`, `BRepMesh_ModelHealer`,
    `BRepMesh_VertexTool`, `IMeshTools_Parameters.hxx`.
  - Documentation: [Boolean spec](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html),
    [mesh guide](https://dev.opencascade.org/doc/overview/html/occt_user_guides__mesh.html).
- **Products:** [Fusion Fillet](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-FILLET-SOLID.htm),
  [Fusion Chamfer](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-CHAMFER-SOLID.htm),
  [FilletFeatureInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/FilletFeatureInput.htm),
  [SolidWorks constant size fillets](https://help.solidworks.com/2019/english/SolidWorks/sldworks/r_constant_size_fillets.htm),
  [T-FLEX/Parasolid vertex blend](https://www.tflex.com/help/eng/T-FLEX%20CAD/17/specifics_of_blending_sets_of_.htm),
  [T-FLEX/Parasolid overflow](https://tflex.com/help/eng/T-FLEX%20CAD/17/blendedges_overflowprocessing.htm),
  [KeyCreator vertex blend](https://help.kubotekkosmos.com/keycreator?id=161181),
  [Rhino 9 FilletEdge/BlendEdge corners](https://discourse.mcneel.com/t/rhino-beta-feature-filletedge-and-blendedge-corners/213727).
- **Papers and books:**
  - Barnhill and Kersey, [*A marching method for parametric surface/surface intersection*](https://scholars.georgiasouthern.edu/en/publications/a-marching-method-for-parametric-surfacesurface-intersection-3/), CAGD 7, 1990.
  - Patrikalakis, Maekawa and Cho, [*Shape Interrogation for CAD/CAM*](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/), §5.8, §6.4, ch. 11 (offsets).
  - Ye and Maekawa, *Differential geometry of intersection curves of two surfaces*, CAGD 16(8), 1999.
  - Sederberg and Meyers, *Loop detection in surface patch intersections*, CAGD 5, 1988.
  - Várady and Rockwood, [*A geometric construction for setback vertex blending*](https://eprints.sztaki.hu/1451/), CAD 29(6), 1997.
  - Várady and Hoffmann, [*Vertex blending: problems and solutions*](https://eprints.sztaki.hu/1756/), 1998.
  - Salvi and Várady, [*G2 surface interpolation over general topology curve networks*](https://diglib.eg.org/handle/10.1111/v33i7pp151-160), CGF 33(7), 2014.
  - J. A. Gregory, *Smooth interpolation without twist constraints*, 1974; J. Hahn, *Filling polygonal holes with rectangular patches*, 1989.
  - Shewchuk, [Triangle](https://www.cs.cmu.edu/~quake/triangle.html), [robust predicates](https://www.cs.cmu.edu/~quake/robust.html), [*Mesh generation for domains with small angles*](https://people.eecs.berkeley.edu/~jrs/papers/small.pdf), 2000.
  - [Gmsh reference manual](https://gmsh.info/doc/texinfo/gmsh.html) (surface meshing in parametric space).
  - Farouki and Rajan, *Algorithms for polynomials in Bernstein form*, CAGD 5, 1988 (Bernstein products for the exclusion tests).
