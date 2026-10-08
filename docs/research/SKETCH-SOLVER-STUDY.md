# Sketch Solver Study: SolveSpace and FreeCAD PlaneGCS

Two open-source 2D constraint solvers, studied for understanding only. Our solver is written from scratch, with no code copied and our own linear algebra.

**Reference prefixes** (all under `.donors/sketch/`):
`ss/` = `solvespace/src/`, `gcs/` = `freecad/src/Mod/Sketcher/App/planegcs/`, `sk/` = `freecad/src/Mod/Sketcher/App/` (the Sketcher layer that drives PlaneGCS).

---

## 1. Unknowns and parameterization

| Entity | SolveSpace | PlaneGCS |
|---|---|---|
| Point | `POINT_IN_2D`: params (u, v) in the workplane (`ss/sketch.h:424`) | `Point { double *x, *y }`, pointers into a shared parameter array (`gcs/Geo.h:39-52`) |
| Line | two point entities, no params of its own | `Line { p1, p2 }` (`gcs/Geo.h:204-208`) |
| Circle | center point + a `DISTANCE` entity holding the radius param (`ss/entity.cpp:129-135`, `:232-235`) | `center` + `rad` param (`gcs/Geo.h:216-220`) |
| Arc | center, start and end points (6 params). The radius is implicit. The entity adds one equation, \|s−c\| − \|e−c\| = 0 (`ss/entity.cpp:946-971`), and skips it when start and end are coincident. The sweep is always CCW from start to end (`ss/entity.cpp:147-172`) | 9 params: start, end, center, `rad`, `startAngle`, `endAngle` (`sk/Sketch.cpp:984-1040`, `gcs/Geo.h:228-237`). The endpoints are tied by **ArcRules**, four `CurveValue` equations p − (c + r(cos θ, sin θ)) = 0 (`gcs/GCS.cpp:961-965`, `gcs/Constraints.cpp:2113-2145`), which leaves 5 DOF |
| Ellipse | none | center, focus1, `radmin` (5 params). The major radius is derived as √(\|f1−c\|² + b²) (`gcs/Geo.h:258-263`). An elliptic arc adds angle params and start/end points tied by rules (`gcs/Geo.h:280-288`) |
| B-spline | `CUBIC`: 4+ control points, non-rational (`ss/entity.cpp:174-180`) | poles (points), `weights` and `knots` as params. `start`/`end` are extra *dependent* points tied by internal-alignment constraints (`gcs/Geo.h:358-370`) |

Both store dimension values differently. SolveSpace bakes `valA` into the expression as a constant (`ss/constrainteq.cpp:277`). PlaneGCS dimension values are `double*`, so a dimension can itself be an unknown (driven or reference dimensions) or a constant outside `plist`.

Fixing geometry also differs. SolveSpace adds `WHERE_DRAGGED` equations, param − current value = 0 (`ss/constrainteq.cpp:1038-1053`). PlaneGCS simply leaves fixed params out of the unknowns (`sk/Sketch.cpp:986`, `FixParameters`).

## 2. Constraint residuals

The notation is d = p2 − p1 and cross(a,b) = aₓb_y − a_yb_ₓ. "Equal" in PlaneGCS means `ConstraintEqual`, p1 − ratio·p2 (`gcs/Constraints.cpp:122`), which the reduction step later removes (§4).

| Constraint | SolveSpace (`ss/constrainteq.cpp`) | PlaneGCS |
|---|---|---|
| Horizontal / Vertical | v_a − v_b or u_a − u_b (`:839-862`) | Equal(p1.y, p2.y) (`gcs/GCS.cpp:931-949`) |
| Coincident pt-pt | u_a − u_b, v_a − v_b (`:623-641`) | two Equals (`gcs/GCS.cpp:925-929`) |
| Point on line | introduces an extra unknown t: a + t(b−a) − p = 0, 2 rows (`:659-674`; param created at `:259-265`) | signed area / length: (−x₀dy + y₀dx + x₁y₂ − x₂y₁)/\|d\| (`gcs/Constraints.cpp:1032-1041`) |
| Point on circle/arc | √(du²+dv²) − r (`:676-692`) | P2PDistance(p, c, rad) (`gcs/GCS.cpp:967`, `:1039`) |
| Tangent line–circle | none. Tangency exists only at shared endpoints | signed P2LDistance(center, line) − (±r), with the side `ccw` fixed at creation (`gcs/GCS.cpp:1123-1139`, `gcs/Constraints.cpp:896-900`) |
| Tangent arc–line (endpoint) | line_dir · (c − endpoint) = 0, where `other` picks the endpoint (`:938-951`). Coincidence is a separate constraint | coincident + `AngleViaPoint` (below) |
| Tangent circle–circle | none (arc–arc only at endpoints) | `TangentCircumf`: \|c1−c2\|² − (r1 ± r2)². Internal vs external is chosen from the geometry at creation (`gcs/GCS.cpp:1141-1187`). Near-concentric circles switch to r1 − r2 (`gcs/Constraints.cpp:1561-1581`) |
| Curve–curve G1 at endpoints | `CURVE_CURVE_TANGENT`: radius vectors parallel (cross = 0) or radius ⟂ cubic tangent (dot = 0) (`:977-1013`). Cubic–line uses the cubic's end tangent × line (`:953-975`) | coincident + `AngleViaPoint`: atan2 of the angle between the two curve normals at the point, rotated by the target angle (`gcs/Constraints.cpp:2389-2405`). The target is 0 or π for tangency, auto-detected from current geometry and stored (`sk/Sketch.cpp:3436-3484`) |
| Equal length | \|ab\| − \|cd\| (`:315-321`) | `EqualLineLength`: len2 − len1 (`gcs/Constraints.cpp:2831-2850`) |
| Equal radius | r₁ − r₂ (`:572-578`) | Equal(rad1, rad2), which gets substituted away (`gcs/GCS.cpp:1217`) |
| Parallel | cross(a,b), unnormalized (`:1015-1036`) | cross(d1,d2) · scale, with scale = 1/(\|d1\|\|d2\|) frozen at creation (`gcs/Constraints.cpp:1155-1171`) |
| Perpendicular | direction cosine = 0 (`:890-916`) | dot(d1,d2) · scale (`gcs/Constraints.cpp:1255-1271`) |
| Midpoint | p − (a+b)/2, 2 rows (`:694-731`) | Sketcher "symmetric about a point": `PointOnPerpBisector` + `PointOnLine` (`gcs/GCS.cpp:1266-1270`) |
| Concentric | coincident centers | coincident centers (substituted) |
| Collinear | none (use two point-on-line constraints) | Sketcher "tangent line–line" = two `PointOnLine` (`sk/Sketch.cpp:3113-3121`) |
| Symmetric about line | (b−a)·d_line = 0, plus signed dist(a) + signed dist(b) = 0 (`:806-837`) | Perpendicular(p1p2, l) + MidpointOnLine (`gcs/GCS.cpp:1260-1264`, `gcs/Constraints.cpp:1478-1489`) |
| Curvature (G2) | none | none for general curves. B-splines have only knot slope/tangent rules |
| Regular polygon | none | none at the solver level. FreeCAD builds it as a construction circle + points on circle + equal lengths |
| Distance pt-pt | \|a−b\| − d (`:279-281`) | \|a−b\| − d. `maxStep` stops d or the actual distance from crossing zero (`gcs/Constraints.cpp:698-760`) |
| Distance pt-line | signed (dv(u_a−u) − du(v_a−v))/\|d\| − d. The sign is kept in `valA` (`:89-124`, `:295-298`) | signed area/length − (±\|d\|) (`gcs/Constraints.cpp:896-900`, `:870-894`) |
| Horizontal/vertical distance | projected distance onto a vector (`:283-293`) | `Difference`: p2.x − p1.x − d (`sk/Sketch.cpp:2846-2860`) |
| Angle | cos(angle between) − cos(θ), multiplied by a gain 0.01/(1.00001−\|cos θ\|) near 0°/180° to keep the rank test sane (`:899-911`). The `other` flag flips one vector (`:896`) | atan2 of l2 rotated by −(atan2(l1) + θ), a wrap-free error in (−π, π] (`gcs/Constraints.cpp:1343-1355`) |
| Radius / diameter | radius expr·2 − D (`:565-570`) | Equal(rad, value) or Proportional 0.5 (`gcs/GCS.cpp:1189-1207`) |

**Takeaways.**
- PlaneGCS's atan2 angle formulation is strictly better than SolveSpace's cosine form, which needs a gain hack near 0°/180°.
- Both solvers handle orientation and side ambiguities (tangent side, internal/external, angle supplement) by **deciding from the current geometry when the constraint is created and storing the choice**.
- In SolveSpace, many constraints reduce to `param − param` and are eliminated by substitution (§4).

## 3. Solve algorithm

**SolveSpace.**
- Every equation is an `Expr` tree (`ss/expr.h:25-49`).
- `WriteJacobian` takes the symbolic partial `PartialWrt` (`ss/expr.cpp:374`) of each equation with respect to each param it references, then constant-folds and stores the results in a sparse matrix of `Expr*` (`ss/system.cpp:22-77`).
- Each iteration evaluates the trees (`:79-93`).
- `NewtonSolve` (`:354-432`) takes a **minimum-norm Gauss–Newton step**, x = Aᵀ(AAᵀ)⁻¹F, computed by a sparse QR of Aᵀ so the condition number is not squared (`:299-334`). It backtracks over 8 halvings until ‖F‖² decreases (`:406-419`).
- Convergence requires every |Fᵢ| < 1e-8 (`LENGTH_EPS/100`, `:18`, `ss/defs.h:7`) within 50 iterations (`:429`). Any value beyond ±1e11 or NaN aborts (`ss/util.h:64`).
- **Dragging**: the dragged params are set to the cursor position and their Jacobian columns are scaled by 1/20 (`:278-285`). The minimum-norm step is therefore 20× more expensive in those params, so they stay near the cursor and everything else absorbs the change.
- **Underconstrained** systems get the minimum-norm step from the current values. The result is minimal motion with no extra machinery.

**PlaneGCS.**
- Every constraint class implements `error()` and `grad(param)` (or `errorgrad`) analytically. A forward-mode dual vector, `DeriVector2`, carries one directional derivative at a time (`gcs/Geo.h:58-160`).
- `calcJacobi` calls `grad` once **per (constraint, param) pair** (`gcs/SubSystem.cpp:254-265`), which costs O(m·n) evaluations. Avoid this.
- Three algorithms are available:
  - **DogLeg** is the default (`gcs/GCS.cpp:2268-2483`): a trust region with Δ₀ = 0.1, a Gauss–Newton step by full-pivot LU, and a steepest-descent Cauchy point.
  - **LM** (`:2089-2266`): μ₀ = τ·max diag(JᵀJ), Nielsen's μ update, dense LU on JᵀJ + μI.
  - **BFGS**, as a fallback.
- Steps are clipped by `maxStep` so that, for example, distances do not go negative (`gcs/SubSystem.cpp:295-318`).
- Success means ‖F‖∞ ≤ 1e-10 (DL) or ‖F‖² ≤ 1e-20 (LM), in at most 100 iterations (`gcs/GCS.cpp:485-508`).
- **Dragging** adds temporary `P2PCoincident` constraints between the dragged point and movable "mouse" params, tagged −1 (`sk/Sketch.cpp:5148-5385`; tag semantics at `gcs/GCS.h:90-105`). For example, an arc center's mouse constraint is rescaled to 0.01 so the rim moves first (`sk/Sketch.cpp:5374-5376`).
- When a component has both real (tag ≥ 0) and temporary constraints, it is solved with **SQP**: the real constraints are equalities and the temporary ones form a least-squares objective. The Hessian comes from a BFGS update, the merit function is ℓ₁, and the subproblem is an equality QP (`gcs/GCS.cpp:1923-1924`, `:4530-4700`). The result is a least-squares-closest drag subject to an exact sketch.
- **Underconstrained** systems simply go wherever DL or LM converges from the current values. There is no explicit minimal-motion objective.

## 4. Decomposition

**SolveSpace** has no graph partition inside a group. Groups are user-level: earlier groups become constants. Within a group it does two things:
1. `SolveBySubstitution` (`ss/system.cpp:99-235`) finds every equation of the form `PARAM − PARAM` (coincident, horizontal, vertical, equal radius when both radii are params) and merges the two params into one. It keeps a **dragged** param as the representative (`:147-152`). It then rewrites the remaining equations and copies the values back after solving (`:597-603`).
2. Any equation that references exactly **one** unknown is solved alone by 1-D Newton before the main system (`:544-571`).

**PlaneGCS** `initSolution` (`gcs/GCS.cpp:1741-1879`) works in three steps:
1. It drops constraints already diagnosed as redundant (`:1771-1781`).
2. It builds a bipartite param–constraint graph and takes **connected components** (`:1786-1805`).
3. Every tag ≥ 0 `Equal` whose params are both unknowns becomes a **reduction map** entry, replacing one param pointer with the other (`:1808-1832`). `SubSystem` then builds its packed parameter vector through that map (`gcs/SubSystem.cpp:55-110`).

Each component becomes a main `SubSystem` (tag ≥ 0) and/or an auxiliary one (tag < 0). Solving iterates over the components (`gcs/GCS.cpp:1908-1947`), and `applySolution` writes the reduced params back (`:4697-4712`).

## 5. DOF, conflicts, redundancy, per-entity status

**SolveSpace.**
- After solving, it takes the sparse QR rank of J: DOF = n − rank (`ss/system.cpp:240-259`).
- If rank < m, `FindWhichToRemoveToFixJacobian` (`:469-513`) **removes each constraint in turn**, rebuilds the system and re-ranks it. Every constraint whose removal restores full row rank goes on the "bad" list. Point-coincident constraints are tested last, and the search is capped by a timeout. This costs O(c) factorizations.
- If the solve fails to converge, the bad list is every constraint with |Fᵢ| > tolerance (`:608-628`).
- **Per-param freedom** (`MarkParamsFree`, `:668-688`) tags one param at a time to drop its column and re-ranks. If the rank stays at m, the param is free. Drawing marks a point free if any of its coordinates is free (`ss/drawentity.cpp:591-596`). This is again O(n) factorizations, which is why it is opt-in.

**PlaneGCS** `diagnose` (`gcs/GCS.cpp:4773-5075`):
- It builds a reduced Jacobian over driving constraints only (`makeReducedJacobian`, `:4726`). It uses dense **full-pivot Householder QR** when there are fewer than 1000 params, and sparse QR otherwise (`:4852`). The pivot threshold is 1e-13 (`:495`).
- **QR of Jᵀ** (constraints as columns) gives the rank, with DOF = n − rank (`:4960`). For each non-pivot column j ≥ rank, `eliminateNonZerosOverPivotInUpperTriangularMatrix` (`:5442-5456`) reduces R so that column j's nonzeros in rows < rank name the pivot constraints it depends on. That set is a **conflict group** (`:5470-5481`).
- **Blame** is a popularity heuristic (`:5507-5590`). The constraint appearing in the most groups is chosen first, then the one with fewer solver rows, then the newest tag. Tag 0 and internal-alignment rows (for example ArcRules) are never blamed.
- **Redundant vs conflicting**: it re-solves without the chosen constraints (`:5596`). Any chosen constraint whose residual is then below 1e-10 is *redundant*. The rest are *conflicting* (`:5608-5650`). Tags are then split into redundant, partially redundant and conflicting (`:5660-5700`).
- **Per-param dependency** comes from a second **QR of J** (params as columns), run in parallel (`:5253-5350`). Non-pivot columns and the pivot columns coupled to them through the reduced R are "dependent", meaning not fixed. Sketcher maps those params to start, end, mid or edge flags per geometry for coloring (`sk/Sketch.cpp:559-640`).
- Caveat: the dependent set depends on pivot order, so it is not canonical.

## 6. Robustness

- **Units.** SolveSpace works in mm with angles in degrees inside `valA`, converted with ·π/180 in the expression. PlaneGCS uses radians, with lengths in sketch units.
- **Scaling.** PlaneGCS normalizes parallel and perpendicular by the line lengths at creation. Neither solver normalizes lengths against the sketch size globally. SolveSpace's comment at `ss/system.cpp:300-315` explains why squaring the condition number (forming AAᵀ) broke large models.
- **Singularities** each solver handles:
  - concentric tangency (PlaneGCS switches the formulation);
  - angle near 0/π (SolveSpace's gain);
  - distance → 0 (PlaneGCS `maxStep`);
  - arc endpoints coincident (SolveSpace skips the radius equation, `ss/entity.cpp:955-966`).
- **On failure**:
  - SolveSpace never writes params back unless the solve converged (`ss/system.cpp:595-606`), so the geometry stays at the last good state and the failing constraints are listed.
  - PlaneGCS saves a `reference` copy (`gcs/GCS.cpp:1881-1899`) and restores it before each solve. It applies the result only on `Success`, and calls `undoSolution` if the resulting geometry is invalid (`sk/Sketch.cpp:5023-5035`). It then falls back DogLeg → LM → BFGS → SQP-augmented, where every param is softly tied to its current value (`sk/Sketch.cpp:5040-5110`).
  - Dragging is refused while conflicts exist (`sk/Sketch.cpp:5150-5154`).

---

## 7. Recommended design for our solver

Targets: about 1 ms per drag frame at 100–200 entities, sketches under 500 entities, per-entity DOF status, and named conflicts.

### 7.1 Parameterization (one flat `f64` array; entities hold indices)
- **Point** (x, y).
- **Line**: two point indices, no own params.
- **Circle**: a center point plus **an explicit `r` param**. This makes radius dimensions and equal-radius trivially substitutable.
- **Arc**: center point, `r`, and **real start and end points**. Two internal residuals, \|s−c\| − r and \|e−c\| − r, give 7 params − 2 = 5 DOF. There are no angle params, so nothing wraps and endpoint coincidence substitutes directly. Sweep is CCW from s to e, and angles are derived with atan2 only where needed (arc length).
- **Ellipse**: center c, major vector a = (aₓ, a_y) (unnormalized, so |a| is the major radius), and minor radius b: 5 params, no angle. Elliptic arcs get real endpoints, each with one point-on-ellipse residual.
- **B-spline** (any degree; 5 is the default per our CAD convention): poles are real points; weights are params but fixed by default; knots are not unknowns. For a clamped spline the curve endpoints *are* the first and last poles, which avoids PlaneGCS's dependent start/end points.
- **Point on spline or ellipse** gets its own curve-parameter unknown t, like PlaneGCS `pointparam`. Lines and circles use implicit residuals instead.

### 7.2 Residual set
All length residuals are in length units and all angle residuals in radians. Divide every row by a characteristic length L for lengths, or multiply angle rows by L, so the rows are commensurate (§7.6).

- **Horizontal / Vertical / Coincident / Concentric / Equal radius / Fix / radius / diameter / coordinate dims.** These are pure param = param or param = const relations. They are **substituted** (§7.4) and never become rows unless substitution is disabled for diagnosis.
- **Point on line**: cross(b−a, p−a)/\|b−a\|. **Point on circle**: \|p−c\| − r.
- **Distance pt-pt**: \|p−q\| − d, with d > 0 enforced; zero means coincident.
  **pt-line**: signed distance − s·d, with s ∈ {±1} stored at creation.
  **Horizontal/vertical distance**: (q.x − p.x) − s·d, which is linear.
- **Parallel**: cross(d1, d2)/(\|d1\|\|d2\|), recomputed every evaluation rather than frozen.
  **Perpendicular**: dot/(\|d1\|\|d2\|).
  **Angle**: atan2(cross, dot) − θ, wrapped to (−π, π]. This is the PlaneGCS form; drop the cosine form.
- **Equal length**: \|d1\| − \|d2\|.
- **Midpoint**: p − (a+b)/2, 2 rows, both linear.
- **Collinear**: two point-on-line rows (b₁ and b₂ against line a).
- **Symmetric about a line**: dot(q−p, dₗ)/\|dₗ\| plus signed distance(p) + signed distance(q), which is SolveSpace's form.
- **Tangent line–circle**: signed distance(c, line) − s·r.
  **Tangent circle–circle**: \|c1−c2\| − (r1 + s·r2). Use the non-squared form, with s = +1 external and s = −1 internal, stored at creation. Guard \|c1−c2\| → 0.
- **G1 at a shared endpoint**: coincidence (substituted) plus atan2(cross(t1,t2), dot(t1,t2)) − φ, where t is each curve's tangent at the point and φ ∈ {0, π} is stored at creation.
- **G2**: G1 plus κ1(P) − s·κ2(P). Signed curvature is ±1/r for arcs and (x′y″ − y′x″)/\|r′\|³ for splines.
- **Regular polygon**: a composite that emits rows only. A construction circle (c, r) and N vertices give N rows \|vᵢ − c\| − r and N−1 rows \|vᵢ₊₁ − vᵢ\| − \|v₁ − v₀\|. That leaves 4 DOF (center, radius, rotation), and the 2N−1 rows are blamed as one user constraint.

### 7.3 Analytic Jacobians
Each constraint kind has one `eval(params, out_residuals, out_jac_rows)` function. It computes the residual and **all** of its partials in a single pass and writes them to a preallocated per-row slot: at most 8–12 (param index, value) pairs, packed as CSR. Shared subexpressions (\|d\|, the unit vectors) are computed once.

There is no expression tree (SolveSpace) and no per-param `grad` call (PlaneGCS). Tests check every kind against central finite differences at random configurations. This is the main correctness net.

### 7.4 Decomposition (rebuilt only when topology changes)
1. **Union-find substitution** over all param = param relations, with an optional ±/offset for horizontal/vertical distance. Each class gets one representative: a dragged param wins, otherwise the oldest. A class holding a constant becomes fixed. Two different constants in one class are an **immediate, named conflict**.
2. **Connected components** of the remaining residual-row ↔ free-param graph, again by union-find.
3. A **fill-reducing ordering** (AMD) per component for the sparse factorization, cached until topology changes. During a drag, topology is frozen.

### 7.5 Algorithm
- **The solve is a weighted minimal-motion Gauss–Newton step with Levenberg damping.** In each iteration, solve (J W⁻¹ Jᵀ + μI) z = F and set δ = −W⁻¹ Jᵀ z. W is diagonal: 1 for ordinary params and about 10³ for dragged params. Use 10³ rather than SolveSpace's 20², so the dragged point tracks the cursor closely.
- **Factor** the m×m system with a sparse LDLᵀ/Cholesky using the cached ordering. It is SPD whenever μ > 0, which also survives redundant rows.
- **Damping.** Start with μ = 0 (pure minimum-norm Newton). Accept a step if ‖F‖ falls; otherwise raise μ ×10 and retry (Nielsen's update). Clip steps with a fraction-to-boundary rule so that r, d > 0, as PlaneGCS `maxStep` does.
- **Converged** means every scaled |Fᵢ| < 1e-10·L and ‖δ‖ < 1e-12·L.
- **Iteration budget**: 20 iterations for a drag frame (warm-started from the previous frame, normally 2–4) and 50 for a commit.
- **Why not SQP?** The weighted minimum-norm step gives the same closest drag with one linear solve per iteration and no BFGS Hessian, and underconstrained geometry moves minimally by construction.
- **Drag loop**:
  1. Set the dragged params to the cursor.
  2. Solve only the component containing them; other components are untouched.
  3. If a frame fails, keep the last converged geometry and report nothing, because the next frame may succeed.
- **Fallback.** Use dense column-pivoted QR on the component if the LDLᵀ shows a pivot below tolerance with μ = 0, then retry with μ > 0.

### 7.6 Scaling and robustness
- Choose **L** as the sketch's bounding-box diagonal (or the median entity size) and solve in units of L, so params are O(1). This fixes the conditioning problem SolveSpace documents without squaring anything badly.
- Guard every normalization (\|d\| < 1e-12·L means a degenerate entity, reported as such).
- Never write a result back unless the solve converged. Keep a copy of the last good params, as both donors do.
- Reference (driven) dimensions are evaluated after the solve and never enter the system.

### 7.7 DOF, per-entity status, conflicts (run after a commit, never per drag frame)
For each component, take the scaled J (m×n, typically under 300×300 after substitution) and do **one dense Householder QR with column pivoting of Jᵀ**: Jᵀ P = Q R.
- **Rank** r comes from \|R_kk\| > 1e-10·\|R₀₀\|. DOF = n − r, plus each free substitution class that touches no rows.
- **Per-param fixedness**, done canonically (better than both donors): the null space of J is Q[:, r:n]. Param i is **fixed** iff row i of that block has norm below 1e-9. An entity is fully constrained iff all its param representatives are fixed or constant. A single factorization does this, compared with SolveSpace's O(n) and PlaneGCS's pivot-dependent sets.
- **Dependent rows** are the columns of Jᵀ that pivoting pushed past r. For each one, R₁₁⁻¹R₁₂ gives the combination of independent rows it depends on, which is its conflict group, as in PlaneGCS. Map rows to user constraints (one constraint can own many rows; substitution conflicts already carry names).
- **Redundant vs conflicting**: after the solve, a dependent row whose residual is below tolerance is *redundant* (report it, still solvable). Otherwise the group is *conflicting*.
- **Blame**: prefer the newest user constraint in the group, then the one appearing in the most groups. Never blame internal rows (arc radius, ellipse and polygon internals).
- **Cost**: dense QR of 300² is about 3.6e7 flops, around 10 ms. That is fine on commit but must stay out of the drag path. If a sketch has a component over 600 unknowns, run this asynchronously after the commit.

### 7.8 Budget check
For 200 entities: about 500 raw params, about 300 after substitution, and about 300 rows. A 2D sketch graph is near-planar, so sparse LDLᵀ fill stays small: tens of microseconds per factorization. Residual and Jacobian evaluation is about 300 rows × 10 flops·k. Four iterations should fit well under 1 ms with zero allocation per frame: one arena per topology, reused across frames.
