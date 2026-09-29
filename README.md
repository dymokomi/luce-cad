# luce-cad

Format-independent CAD foundation, entirely **Luce Base**. No STEP syntax, UI,
editor commands or donor C/C++ dependencies.

The intended ownership is:

`luce-step` (file entities) → `luce-cad` (analytic faces/topology, surface
evaluation and trimmed-surface meshing) → `luce-geocore` (display mesh).

Exports: `cad` (the analytic model, `luce_cad.model`) and `tessellation`
(rational B-spline evaluation and trim/grid/recombine meshing,
`luce_cad.tessellation.surface`). The tessellation modules were the separate
`luce-tesselator` package until luce-cad 0.2.0.

`cad_geometry` (`luce_cad.cad_geometry`) makes CAD models a family of
luce-geocore's `GeometrySet`: the `cad` component holds models (shared, never
copied), places, joins and filters them by face path, and
`CadGeometry.tessellated(set, segments, edge_size, progress)` turns them into
the set's mesh in one pass, each face's B-rep path in the `path` text
attribute. Luce uses `CadGeometry.of_model`, `model_count`, `model` and
`face_count`.

## Analytic model API

`cad.CadModel()` owns independent faces and copies input arrays:

- `add_planar(boundary)` adds a simple coplanar polygon boundary, preserving winding.
- `add_nurbs(points, weights, nu, nv, degree_u, degree_v, knots_u, knots_v)`
  retains the exact rational control net and expanded knot vectors.
- `trim_rectangle(face, u0, u1, v0, v1)` changes the NURBS face's rectangular
  parameter domain. Invalid edits leave the previous domain unchanged.
- `tessellate(segments=16, edge_size=0, progress, curvature=false,
  directions=false)` creates a display mesh without replacing
  analytic data. Zero disables size refinement; positive values are target
  spacing in source units, not a guaranteed minimum/maximum edge length.
- `face_count()` counts CAD faces, not tessellated polygons.
- `preview_face(face, divisions=8)` produces one disposable display patch,
  without requiring the entire CAD model to fit a single polygon mesh.
- `preview(progress)` previews every face on the worker pool (8 divisions for
  faces with more than 32 edges, else 16; then 4, then 16 on failure) and
  returns a `CadPreviewSet`: display batches of at most 4,096 points and
  16,000 corners merged in face order, deduplicated analytic boundary
  segments (64 chords per curved edge), one normal guide per meshed face, and
  the failed-face count with the first failure as `Patch N: message`.
- `planned_face(face, segments=16, edge_size=0)` is a B-rep diagnostic: build
  the complete shared-edge/layout plan, then mesh only the selected face.
  Unlike isolated preview, this retains the full cook's neighboring station
  decisions, source-face attributes, display triangulation and normals. Unused
  points are compacted. Success does **not** validate the other face jobs or
  prove that the full model tessellates; independent non-B-rep faces are not
  supported by this diagnostic.
- `face_edge_count/id/linear/point` expose shared boundary identities and exact
  samples for cached patch outlines, independent of display triangle edges.
- `transformed(translation, rotation, scale)` returns a separate analytic model,
  retaining trim domains and preserving orientation under reflection.

Planar boundaries currently have one outer loop with 3–256 corners and absolute
coplanarity tolerance 1e-6 in source units. Up to 1024 independent CAD faces are
stored; generated meshes are also bounded by luce-geocore's modeling budgets. A valid
analytic model can exceed a particular display mesh budget and fail tessellation.

## Shared boundary topology

`BrepModel` owns vertices, line/circle/rational spline edges, analytic support
surfaces, and faces with oriented edge loops. `CadModel.attach_brep` retains that
model. Adjacent faces reuse the same sampled edge point IDs, including periodic
seams. No positional weld is needed for a correctly connected input B-rep.

Supported face meshing:

- Planes with concave outer boundaries and multiple non-touching holes.
- Full or partial cylindrical/conical bands bounded by two circular arcs.
- Four-sided trimmed NURBS and analytic patches use a boundary-conforming
  Coons grid in parameter space, lifted to the exact support. Logical sides may
  contain multiple STEP edges. Opposite side counts agree before shared edge
  sampling; interior curvature can raise the counts even with straight borders.
  Folded, singular and degenerate grids fall back to constrained trim meshing.
- NURBS trim loops projected to UV, meshed and lifted to the surface. Repeated
  seams on verified closed supports retain separate periodic chart images but
  stitch through the original shared 3D identities. Projection must agree within
  model tolerance; singular/ambiguous charts and arbitrary p-curves can still fail.
- B-spline edges can retain a trimmed knot subinterval, including a seam-crossing
  interval on a geometrically closed curve. Endpoints must agree with the curve
  within model tolerance; no silent tolerance inflation is performed.
  A few ulps of endpoint reconstruction error are clamped before periodic wrap;
  an open spline cannot jump to its opposite endpoint from arithmetic roundoff.
- NURBS interiors use bounded geometric-deflection refinement with shared edge
  split IDs. Existing shared boundaries remain fixed. Eight passes and a per-patch
  face budget bound work; this is not a certified global chord-error guarantee.
- Trimmed cylinders/cones/spheres/tori in unwrapped analytic UV coordinates;
  latitude-bounded spherical caps receive a pole-aware mesh.
  If rotating a spherical chart turns an authored seam into a retraced interior
  slit, matching canonical identities allow it as a fixed constraint. Every
  authored seam segment survives triangulation, grid insertion, diagonal flips,
  quad pairing and surface refinement. Unidentified overlaps remain errors.
  Cylinder/cone clipping charts carry the angular branch between successive
  coedges even without a repeated seam edge. Hole loops choose the outer loop's
  chart branch; new shared stations retain that branch on refresh. This avoids
  fictitious full-revolution trim chords without moving any boundary point.
- Four-sided planar faces share the mapped path. The constrained fallback for
  complex planes uses an interior lattice before conservative triangle pairing.
  Planar four-side candidates are preflighted with the actual fitted mesher
  while shared cuts are still mutable. A folded candidate receives a support
  grid plan instead of skipping directly to fallback triangles. Successful
  maps retain their strategy; four logical corners alone do not prove that
  the fitted parameter grid is valid.
- Irregular planar, cylindrical, conical and NURBS trims can use a grid-first cell-clipping
  path. Curvature and edge-size tests seed directional rows; compatible
  isoparametric coedges transfer their station positions across patch boundaries,
  including swapped/reversed UV axes. Exact CAD-curve/grid intersections become
  shared B-rep samples before parallel meshing. Uncut cells remain quads; an
  oblique trim terminates them as small cut polygons instead of bending every row.
- Four coedges no longer automatically select a boundary-fitted grid. Hard
  oblique trims on nonperiodic NURBS domains prefer the independent support
  grid. Opposite-edge parameter speed alone does not turn a smooth seam into
  a flow stop. Verified periodic/repeated-edge boundaries can use a cut chart;
  unsupported charts retain their existing periodic strategies. The clipped path is
  attempted before a fallback that transports all shared boundary samples.
- A supported patch without four logical sides owns its cut chart from the
  start. Its physical-pitch and stopped-end rules apply during planning, not
  only after an impossible mapped attempt. Nearly incident trim coordinates
  can straighten into ordered collinear UV corners under existing numerical
  and model-tolerance limits; canonical 3D positions/IDs remain unchanged.
  Backtracking, collapsed segments, crossing and orientation reversal remain
  forbidden. This prevents numerical boundary ribbons from forcing a valid
  clipped extrusion into a triangle fallback.
- Support-grid planning checks sampled display-triangle error on both diagonals,
  in addition to directional isocurve tests. Mixed curvature can add complete
  rows before shared cuts are frozen. Mapped and clipped neighbors reconcile
  together. Exact shared boundary stations remain immutable; interior row
  ownership may stop an incompatible phase without deleting its trim corners.
- Mapped NURBS interiors can redistribute physical row spacing while keeping
  every trim vertex fixed. Boundary-to-first-row and last-row-to-boundary gaps
  participate. Three bounded passes require reduced normalized gap variance,
  no increased worst gap ratio, consistent UV orientation and a sampled
  display-deviation check against the tessellation quality budget. Improving
  one axis is not blocked by an unchanged worst row in the other axis.
  Rejected moves do nothing; exact trims and CAD boundary tolerance stay fixed.
  This is not a certified global surface-error bound.
- Severely crowded clipped NURBS interiors can also redistribute complete
  grid-line chains. A short terminal boolean-cut interval alone does not trigger
  this pass. Every trim/hole vertex and polygon connection stays fixed; only
  interior UV points move and are re-evaluated on the exact support. Three
  bounded sweeps backtrack under the unchanged display-error budget. Acceptance
  requires improved physical gap variance, no worse worst gap ratio, reduced
  aggregate quad distortion, and no worse original stretched/skewed counts or
  maximum edge ratio. Valid UV triangles and quad corner turns are preserved;
  originally well-angled charts cannot acquire corners below a sine of 0.5.
  Actual display triangles and support normals are checked transactionally.
  This improves spacing without claiming a globally optimal quad layout or
  eliminating competing station phases at the frozen boundary.
- Cut polygons retain their own display triangles independently from their real
  wire edges. Bounded diagonal flips remain inside convex UV quadrilaterals and
  strictly reduce support-normal violations. UV and projected candidates are checked against exact support
  normals; consistently oriented candidates survive worker assembly, affine
  placement, copies and subsets. If local flips stall, a bounded dynamic program
  searches visible UV diagonals for a complete normal-consistent triangulation
  (up to 256 corners, only on failed cells). Tiny grazing cut cells can merge
  across one internal manifold side into a larger neighbor: at most one eighth of its
  UV area, no extra touching vertices, no moved/deleted trim points, and at most
  32 merges per patch. The final lifted mesh still passes the same normal and
  sampled surface-deviation checks. A clipped patch with an unresolved folded cell
  is rejected transactionally so the next strategy can run. This does not claim
  that all legacy fallback meshes are free of folded triangles.
- Trim ribbons whose physical area/longest-edge ratio is below five percent
  of model tolerance can merge into a larger adjacent cell. Only
  boundary-adjacent cells qualify; the existing one-eighth-area, single-shared-
  side and simple-union guards apply. At most 128 optional dissolves run per
  patch. This removes redundant interior grid edges, not CAD boundary vertices
  or geometry; final lifted display validation remains mandatory. Curved charts
  use lifted physical positions for the area/edge measure, not arbitrary UV
  distances. Unchanged display triangles are retained when dissolving the shared
  side. This local cleanup does not promise well-shaped cells for every trim.
- Compatible circular logical sides transfer angular stations as well as counts,
  including compound sides made of several coedges. This prevents equal-count
  but differently phased fillet strips. Fully mapped coaxial families first
  agree on evenly distributed angular intervals between mandatory CAD endpoints,
  with a circular-sagitta density floor. Optional quadrant anchors cannot crowd
  nearby required endpoints. A clipped smooth neighbor seeds every coedge on
  its dominant parameter rail before curvature refinement, so compound rails
  do not acquire a competing phase on their shorter segments. Nearly constant
  parameter rails must also pass a physical-displacement check at every station:
  UV epsilon alone is unsafe on stretched charts. This classifies row transport;
  it never snaps the exact trim or increases the model tolerance. Mixed
  mapped/clipped circular families can therefore share the initial distribution
  and add support rows together during reconciliation. Hard or incompatible neighbors still
  terminate interior rows rather than inheriting the full canonical sample set.
  Mapped, clipped and constrained strategies validate their output before
  committing. Unsupported layouts fall back without dropping shared seam IDs.
- Simple support grids can clip up to 1,024 authored coedges, subject to the
  existing 16,384 sampled-boundary and bounded station/cell budgets. A grooved
  cylinder/cone is not forced into triangle fans solely because it has more
  than 128 topological edges. The four-sided mapped recognizer stays separate;
  admitting a clipped chart does not pretend its jagged boundary is rectangular.
- Support grids allow up to 1,025 stations per axis under a joint 131,072-grid-point
  budget. A long, narrow patch is not forced into fallback triangles by a
  square 257-by-257 axis limit. The joint limit is checked during inherited-row
  planning and before diagonal-refinement allocation, not only at final clipping.
  This supports anisotropic layouts; it does not solve inherited phase crowding
  or promise a minimal polygon count.
- Endpoint-only smooth rails do not impose a planar interior phase. Independent
  narrow planes start with proportional physical axis counts. A late seam cut
  is not extended across such a plane if it would create a rectangle over 20:1
  and more than double that same rectangle's previous aspect. An unrelated
  pre-existing skinny cell cannot excuse damage to another row. The exact cut stays on the
  shared boundary; useful splits, existing coupled phases, size/curvature rows
  and nonplanar grids are unchanged by this admission policy.
- Proven affine NURBS extrusion directions can subdivide long support-grid
  intervals at an even physical pitch, targeting sixteen times the mean
  transverse interval. Curved-axis stations and existing straight-axis rows
  stay fixed. Optional refinement is skipped atomically if its full proposal
  exceeds either axis capacity or the joint grid budget. This is a support-grid
  shape target, not a bound on every Boolean cut polygon. Once refined, the
  straight direction rejects a later neighbor's row phase outside the middle
  40–60% of an existing interval; the exact shared cut remains a boundary corner.
  It does not spread hard-edge samples through the interior or resample curved
  directions merely to improve aspect ratios.
- Four-edge mapped affine strips can resolve competing straight-rail phases
  into one evenly distributed row family, with the same mean-pitch target.
  The complete control net proves affine translation using Greville positions,
  including degree-elevated and knot-inserted representations; warped nets and
  changing rail weights are excluded. Exact parallel line rails must cover the
  same projected interval. Curved rows are unchanged, every old shared cut is
  retained as a boundary corner, and over-budget proposals are skipped. The
  broader recognition does not opt higher-degree charts into the separate
  clipped endpoint/pitch policies before their neighboring phases are handled.
- Planar circle/ellipse trim envelopes include analytic projected extrema within
  the trimmed sweep. A sampled arc plus a percentage guard is insufficient for
  thin segments: later grid intersections can otherwise escape that envelope.
  Exact envelope bounds do not relocate any canonical boundary vertex. Other
  support/curve combinations still use their guarded sampled envelopes.
- Close edge/grid intersections around a sampled boundary turn are distinct
  incidences, not duplicate estimates of one crossing. Their roots and the
  intervening extremum survive physical sample deduplication. Monotone runs
  retain the existing close-point representative and align its chart coordinate
  without moving canonical geometry. Regressions cover both sides of a shallow
  tangent under loose/tight model tolerances, idempotent reconciliation, swapped
  charts, reversed coedges and scaled monotone counterexamples.
- Boundary stitching and interior row transport are separate. Sampled tangent
  planes and U/V directors classify seams before meshing: G0-only creases and
  incompatible smooth frames stop flow; aligned/reversed/90-degree-swapped
  smooth frames may continue it. Stopped seams retain all shared point IDs as
  boundary polygon corners without imposing the same interior grid on a neighbor.
  The former hard-curved-edge support-row override has been removed. If a
  conformed mapped boundary polygon has invalid projected display ears, it can
  retry support-chart triangulation, bounded diagonal repair and sampled support
  error validation. The accepted indices survive worker assembly. A cell that
  still fails rejects the strategy; boundary IDs are never dropped to hide it.

Transforms retain the B-rep and compose an analytic placement matrix; they do
not tessellate the source or approximate nonuniformly scaled circles as circles.
Reflection reverses generated face winding. Source coordinates are retained.

Meshes carry `cad_face` primitive provenance, optional primitive `Cd` from CAD
face colors, and normalized corner `N`. B-rep shading normals use exact rational
NURBS first derivatives or the analytic surface normal, independently per CAD
face. Singular NURBS endpoints use a nearby limiting interior evaluation. Face
workers prepare the normals alongside geometry, before assembly placement. Positive
edge size refines shared boundaries and inserts planar/UV interior lattices.
Quads remain quality-filtered; triangles are allowed near trims.
Clipped patches reuse their known UVs for normals instead of projecting all
generated vertices again. The integer primitive attribute `cad_trim_grid`
identifies that path for diagnostics; `cad_face`, paths and colors are retained.
`cad_mesher` records the accepted per-patch strategy: 1 = local-row mapping,
2 = clipped support grid, 3 = full canonical-row mapping, 4 = constrained
fallback, 5 = baseline fallback conformed to shared seams. It reports the
actual accepted path, including parallel face jobs, not a quality guarantee.
`cad_flow_island` identifies connected compatible-flow faces (the minimum local
CAD face index in the component). It is layout provenance, not object hierarchy
or a mathematically certified continuity classification. The current classifier
samples seven seam locations with 0.5-degree tangent-plane and 5-degree director
tolerances. Conformed mapped polygons also check actual render triangles against
support normals; invalid warped n-gons retry a denser/constrained strategy.

Planning is bounded to 1,025 U/V stations per face under a joint 131,072-point
grid budget, 1,025 stations per shared
edge, eight initial trim reconciliation passes, 32 mapped-side balancing passes
per call, and at most twelve joint mapped/clipped reconciliation rounds.
It is not a global cross-field parameterization or a guarantee of all quads.
Incompatible row phases may terminate at cut cells; strongly mismatched or
singular UV domains still use the constrained fallback. These extra continuity
constraints can increase mesh density and tessellation time.

This is **not yet a general B-rep kernel**. Arbitrary p-curves, periodic trimmed
NURBS, general singular trims, sewing disconnected shells, intersections,
booleans and tolerance-controlled refinement remain unimplemented. Current
stitching follows existing topology; it is not tolerance-based healing.
Boundary/support agreement uses the model's tolerance (default 1e-6 source
units). Full circles start at `4 * segments` intervals; partial circles scale by
sweep. Surface-error, shared-row and circular-family constraints can raise these
counts before canonical points are frozen.
Spline counts use sampled turn/deflection tests within that quality budget;
straight/simple splines need fewer intervals. Opposite repeated seams confined
to a single four-coedge band can refine the first passing power-of-two count
within its bracket. Each returned count passes the same sampled criteria in its
own phase. Externally shared edges and irregular trims retain their original
dyadic family: changing those independently can crowd neighboring charts even
when the curve's own error improves. This is neither a certified tolerance
bound nor a globally minimal count. Opposing logical sides agree on total
counts, including split edges.
Lines start with one interval, then size/structured constraints may refine them.
Unsupported trims and mesh-budget overflow fail explicitly.

STEP exposes `Step.decode_model` / `Step.load_model`; its mesh convenience APIs
delegate to this package. Regression tests currently run with sibling
`luced-3d/tests/run.py`, covering model preservation, re-tessellation, rectangular
trim bounds, invalid-trim atomicity, holes, periodic seams, shared-edge
manifoldness, reflection and nonplanar boundary rejection. Mapped-patch tests
cover internal valence four, exact curved-interior samples, compound sides,
rational derivative normals, endpoint knots and reversed face orientation.
Trim-grid tests additionally cover swapped/reversed UV flow, seam valence,
oblique cuts, enclosed/crossing holes, and circular fillets with reversed frames
and compound opposite sides, including an arbitrary split almost coincident
with an optional quadrant anchor. Interior circular rows must remain evenly
distributed rather than merely sharing the union of two crowded phases.
Seam regressions cover hard-edge n-gons, smooth 45-degree UV mismatches, 90-degree
row transport, circular fillet/Cartesian-plane stops and curved strip support.
Internal Base regressions cover endpoint roundoff versus real periodic wrap,
diagonal curvature with straight isocurves, immutable trims, and first/last
interior gaps with distortion in one or both row families.
Near-grid corner alignment is completed after crossing alignment, so loop
origin and winding cannot strand an endpoint in a neighboring cut cell.
The bounded second pass preserves canonical 3D positions and rejects UV
orientation changes, new crossings, and independently moved seam aliases.
Clipped spacing also recognizes oversized boundary-to-first-row gaps, not just
uneven interior gaps; an isolated tiny Boolean-cut interval is still left alone.
Spacing proposals retain the existing topology, normal, sampled-deviation and
shape-quality gates even when no original quad exceeds the 20:1 diagnostic.

On affine NURBS extrusions, an isoparametric rail ending at a stopped oblique
trim contributes a boundary corner, not necessarily an interior station. The
planner recognizes translated control-net pairs with matching weights and
keeps curved axes, smooth joins, rail subdivisions and uncertain chart aliases
conservative. Guard cells cover the trim envelope; actual rectangular sides
remain mandatory rows. This removes thin endpoint wedges without moving the
exact shared trim. Warped and doubly curved supports retain the previous policy.
Contracts cover scaled/transposed rational nets, reversed/rotated loops,
smooth versus stopped neighbors, actual row inheritance, clipping area and
canonical boundary preservation.

Spherical pole-chart fitting retries a density-biased boundary average with a
bounded convex-hull closest-point iteration. Acceptance still requires every
sample to lie inside the same open-hemisphere margin; antipodal domains remain
unsupported. Scale/frame/density/reversal contracts preserve the exact sphere
and boundary points and verify rejection of genuinely incompatible domains.

## Curvature analysis

`tessellate(..., curvature=true)` (and `BrepModel.mesh`) adds corner
attributes `curvature.k1` and `curvature.k2`: the principal curvatures of the
exact support at each corner, from its first and second fundamental forms
(rational NURBS second derivatives, or the analytic chart's). k1 >= k2;
positive where the surface bends away from the face's outward normal, so a
sphere of radius r has H = 1/r and K = 1/r^2, a convex cylinder k1 = 1/r and
k2 = 0, and a bore k2 = -1/r. `directions=true` adds unit principal
directions `curvature.d1`/`d2`. Values are per corner, like `N`, so faces
meeting at a seam keep their own. Off by default; about 0.1 s on a 2.7M-corner
model. A CadModel placement must be a uniform scale (curvature divides by it).
The columns are float32.

## Parallel work

One persistent worker pool (luce-geocore's `geocore_parallel`, processors minus one, shared with the geometry kernels) runs
the per-face work: layout seeding and row setup, speculative crossing
computation, face meshing and normals, curvature and previews. Each item runs
in its own runtime context and hands back raw arrays only. Layout
reconciliation stays serial (each face sees its neighbours' newest cuts) but
reuses an exact per-face memo of NURBS inversions and edge/grid crossings
that a parallel pass fills first, so output is identical to the serial order
on any number of cores. Errors are raised in face order.

## Surface evaluation and trim meshing (`tessellation`)

Rational tensor-product B-spline evaluation and mesh tessellation in
`src/luce_cad/tessellation/`. `tessellation.NurbsSurface.tessellate(points,
weights, nu, nv, degree_u, degree_v, knots_u, knots_v, segments=16)` samples a
whole patch.

Points/positive weights use u-major, v-minor order. Knot vectors are expanded,
finite and nondecreasing. The evaluator uses Cox–de Boor basis functions and a
rational weighted sum over only the active controls, with binary span lookup.
`sample_checked` is for immutable nets already validated by `check`; callers
must not change the net afterward. Returns an immutable indexed polygon mesh; its quads are
triangulated by luce-geocore for rendering. No UI, File node or STEP syntax lives here.

`surface_normal_checked` evaluates exact rational first derivatives on an
already validated immutable net and returns their normalized cross product.
It returns zero at a singular parameterization; CAD owns limiting-pole policy.

`tessellate_region` also accepts `u0, u1, v0, v1` before `segments` for a
rectangular parameter subdomain; `check` and `check_region` validate input without
allocating a mesh. The CAD model owns the face's chosen domain.

`NurbsCurve.at` and `NurbsSurface.at` expose rational evaluation at actual
knot-domain parameters. `TrimPolygon.triangulate` accepts a planar outer loop
and holes, checks intersections/nesting, builds visible bridges, and clips ears
without dropping boundary vertices. `quadrangulate` first improves interior
diagonals using bounded Delaunay-style edge flips, then pairs adjacent
triangles only when convex, with all angles between 20 and 160 degrees and edge
length ratio at most 10. It does not move vertices or remove trim boundaries.
`quadrangulate(..., edge_size=0)` optionally inserts an interior regular lattice
before improving diagonals and recombining. Zero disables it. Shared boundary
sampling remains the caller's responsibility; this routine never splits a
boundary edge independently. Refinement is bounded to 8192 candidate cells.

Trim intersections, ear membership and hole containment use dominant-plane
orientation signs, with an ordinary floating-point filter and a bounded
floating-expansion fallback for ambiguous binary64 determinants. Close points
are not automatically touching or welded. These topology predicates are
separate from a CAD reader's geometric fitting tolerance. Ear selection still
requires locally conditioned display triangles; genuinely degenerate domains
remain errors. The fallback assumes finite, normal-range products in the
bounded mesh coordinate range, not arbitrary unbounded/subnormal inputs.

Ear clipping avoids leaving a remainder with no conditioned convex corner.
If it still stalls on a single loop, it may add one area-centroid interior seed,
but only after every original boundary segment forms a positive, conditioned
triangle to that point and the area is preserved. This kernel check rejects
invisible spokes; holes never take this fallback. Boundary positions and edges
remain exact. The seed is subsequently eligible for the ordinary lattice,
diagonal-improvement and pairing steps. Diagonal guards are relative to local
squared edge length, so ordinary model-unit changes do not disable flips.

`triangulate_seams(points, sizes, normal, boundary_ids)` and
`quadrangulate_seams(points, sizes, normal, boundary_ids, edge_size=0,
second_spacing=0)` accept explicit nonnegative canonical boundary identities.
A reversed coincident segment within one loop is legal only when both endpoints
have matching identities and exact chart positions. Distinct periodic chart
images remain separate. Mere proximity or equal positions with different IDs
never authorize overlapping constraints. A bounded visible-diagonal partition
can separate a retraced slit tip before ear clipping; it keeps every authored
segment and does not insert arbitrary spokes through the domain.

`TrimConstraints.build(points, ids, sizes)` records coincident seam constraints
by stable input point index. Its owner calls `close()`; meshing stages borrow the
table. `contains(a,b)` protects those segments from grid insertion, diagonal
flips, quad pairing and caller refinement. The table is empty for ordinary
boundaries, which are protected by one-face incidence. Callers must retain input
point indices. This is not a general intersecting-constraint arrangement solver.

Surface-grid sampling is uniform, not a guaranteed chord-error tolerance.

`TrimGrid.clip(points, sizes, us, vs)` intersects a supplied UV grid with one
outer polygon and holes. Whole cells stay quads; boundary cells keep their
polygon outlines. Holes within a cell and cells over the 256-corner polygon
limit are triangulated locally. **Input trim segments must already be split at
every grid crossing.** The first output points retain every input point and ID;
the clipper neither invents CAD seam samples nor welds distinct vertices. Output
validation requires every input trim segment exactly once and every interior
edge twice. Touching/ambiguous arrangements are rejected, not silently patched.
The implementation uses per-cell boundary buckets and per-row scanline parity.

Curved trim evaluation, cross-patch station planning, adaptive support checks,
periodic bands and shared CAD-edge ownership live in the CAD model above. General
field-aligned all-quad remeshing is not implemented.

Limits: degree 1–8, 2–1,024 surface control points per direction (262,144 total), up to 4,096 curve
control points, divisions 1–64. Invalid
weights, knot counts and collapsed surface polygons are checked errors.
Planar trims are bounded to 64 loops and 16,384 vertices. Exact shared-vertex
junctions within one loop are supported; touching distinct loops and unidentified
overlapping segments remain invalid. Explicit retraced seams use the identity
API above. Regression tests include
a bilinear plane, rational quarter-cylinder/curve, multiple holes, invalid
boundaries, conservative quad recombination, grid-crossing holes, oblique trims,
oversized cut cells and rejection of unsplit trim crossings.

## Tests

`./test.sh` builds and runs, native and through the C backend (`--opt N`,
`--backend native|c` narrow it):

- the Base contracts in `src/luce_cad/tests/` (`layout_contract` and
  `trim_predicates_contract` run every contract module they import), which reach
  unexported internals;
- the Luce regressions in `tests/` (`main.luc` and one `*_tests.luc` module per
  topic), which build CAD models in code, mesh them and check topology, trims,
  normals and spacing through the public `cad` and `tessellation` exports.

STEP-file regressions live in luce-step; editor behavior stays in luced-3d. CI
pins the compilers and sibling packages in `bootstrap/PACKAGES`.
