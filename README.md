# luce-cad

Format-independent CAD foundation, entirely **Luce Base**. No STEP syntax, UI,
editor commands or donor C/C++ dependencies.

The intended ownership is:

`luce-step` (file entities) → `luce-cad` (analytic faces/topology) →
`luce-tesselator` (surface sampling/meshing) → `luce-3d` (display mesh).

## Analytic model API

`cad.CadModel()` owns independent faces and copies input arrays:

- `add_planar(boundary)` adds a simple coplanar polygon boundary, preserving winding.
- `add_nurbs(points, weights, nu, nv, degree_u, degree_v, knots_u, knots_v)`
  retains the exact rational control net and expanded knot vectors.
- `trim_rectangle(face, u0, u1, v0, v1)` changes the NURBS face's rectangular
  parameter domain. Invalid edits leave the previous domain unchanged.
- `tessellate(segments=16, edge_size=0)` creates a display mesh without replacing
  analytic data. Zero disables size refinement; positive values are target
  spacing in source units, not a guaranteed minimum/maximum edge length.
- `face_count()` counts CAD faces, not tessellated polygons.
- `preview_face(face, divisions=8)` produces one disposable display patch,
  without requiring the entire CAD model to fit a single polygon mesh.
- `face_edge_count/id/linear/point` expose shared boundary identities and exact
  samples for cached patch outlines, independent of display triangle edges.
- `transformed(translation, rotation, scale)` returns a separate analytic model,
  retaining trim domains and preserving orientation under reflection.

Planar boundaries currently have one outer loop with 3–256 corners and absolute
coplanarity tolerance 1e-6 in source units. Up to 1024 independent CAD faces are
stored; generated meshes are also bounded by luce-3d's modeling budgets. A valid
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
- Nonperiodic NURBS trim loops projected to UV, meshed and lifted to the surface.
  Projection must agree within the model tolerance; singular/ambiguous periodic
  parameterizations and arbitrary p-curves are not supported.
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
- Four-sided planar faces share the mapped path. The constrained fallback for
  complex planes uses an interior lattice before conservative triangle pairing.
- Irregular planar/nonperiodic NURBS trims can use a grid-first cell-clipping
  path. Curvature and edge-size tests seed directional rows; compatible
  isoparametric coedges transfer their station positions across patch boundaries,
  including swapped/reversed UV axes. Exact CAD-curve/grid intersections become
  shared B-rep samples before parallel meshing. Uncut cells remain quads; an
  oblique trim terminates them as small cut polygons instead of bending every row.
- Four coedges no longer automatically select a boundary-fitted grid. Hard
  oblique trims on nonperiodic NURBS domains prefer the independent support
  grid. Opposite-edge parameter speed alone does not turn a smooth seam into
  a flow stop. Periodic/repeated-edge
  boundaries retain their existing periodic strategies. The clipped path is
  attempted before a fallback that transports all shared boundary samples.
- Support-grid planning checks sampled display-triangle error on both diagonals,
  in addition to directional isocurve tests. Mixed curvature can add complete
  rows before shared cuts are frozen. Mapped and clipped neighbors reconcile
  together; late straight-rail redistribution cannot change a smooth seam's
  agreed stations independently.
- Mapped NURBS interiors can redistribute physical row spacing while keeping
  every trim vertex fixed. Boundary-to-first-row and last-row-to-boundary gaps
  participate. Three bounded passes require reduced normalized gap variance,
  no increased worst gap ratio, consistent UV orientation and a sampled
  display-deviation check against the tessellation quality budget. Improving
  one axis is not blocked by an unchanged worst row in the other axis.
  Rejected moves do nothing; exact trims and CAD boundary tolerance stay fixed.
  This is not a certified global surface-error bound.
- A warped cut n-gon that cannot be displayed safely is decomposed using its UV
  triangles locally, without replacing the whole patch by a constrained fan.
  Unaffected quads/n-gons and shared boundary identities remain unchanged.
- Compatible single-circle sides transfer angular stations as well as counts.
  Compound sides retain count matching with interior geometric refinement;
  universal split-ring phase transfer still needs tolerance-aware trim healing.
  This prevents equal-count but differently phased ordinary fillet strips.
  Mapped, clipped and constrained strategies validate their output before
  committing. Unsupported layouts fall back without dropping shared seam IDs.
- Boundary stitching and interior row transport are separate. Sampled tangent
  planes and U/V directors classify seams before meshing: G0-only creases and
  incompatible smooth frames stop flow; aligned/reversed/90-degree-swapped
  smooth frames may continue it. Stopped seams retain all shared point IDs as
  boundary polygon corners without imposing a grid on a planar neighbor.
  Analytic bands and single-span transverse NURBS strips can retain additional
  curved-boundary support rows locally. This guard is deliberately not applied
  to doubly-curved NURBS patches, where it can reintroduce distorted row phases.

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
`cad_flow_island` identifies connected compatible-flow faces (the minimum local
CAD face index in the component). It is layout provenance, not object hierarchy
or a mathematically certified continuity classification. The current classifier
samples seven seam locations with 0.5-degree tangent-plane and 5-degree director
tolerances. Conformed mapped polygons also check actual render triangles against
support normals; invalid warped n-gons retry a denser/constrained strategy.

Planning is bounded to 257 U/V stations per face, 1,025 stations per shared
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
units). Full circles use `4 * segments` intervals; partial circles scale by sweep.
Spline counts use sampled turn/deflection tests within that quality budget;
straight/simple splines need fewer intervals. This is not a certified tolerance
bound. Opposing logical sides agree on total counts, including split edges.
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
and compound opposite sides.
Seam regressions cover hard-edge n-gons, smooth 45-degree UV mismatches, 90-degree
row transport, circular fillet/Cartesian-plane stops and curved strip support.
Internal Base regressions cover endpoint roundoff versus real periodic wrap,
diagonal curvature with straight isocurves, immutable trims, and first/last
interior gaps with distortion in one or both row families.
