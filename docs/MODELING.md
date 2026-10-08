# Modeling

`luce_cad.modeling` builds and changes closed B-rep solids. The operations are features in the
sense of Fusion 360's timeline: sketch profiles → Extrude → Join, Cut and Intersect with other
bodies → Chamfer and Fillet. luced-3d's CAD node wires them as nodes.

The research behind the design is in
[research/CAD-MODELING-STUDY.md](research/CAD-MODELING-STUDY.md). It covers Fusion 360,
Plasticity, OpenCASCADE, SolveSpace, truck and the papers. This document describes what the code
does today.

## Operations

| `CadModeling.` | What it does |
|---|---|
| `extruded(set, distance, start, tag)` | The sketch's closed curves in `set`, as profile regions, each swept `distance` along the plane's normal from `start` along it. |
| `revolved(set, axis, tag, angle)` | Each region turned about the plane's u (`0`) or v (`1`) axis: a full turn, or `angle` radians closed by the profile at each end. |
| `profile_count(set)` | How many regions the sketch has. |
| `combined(a, b, operation)` | `0` union, `1` `a` without `b`, `2` intersection. |
| `chamfered(model, references, distance, tag)` | The referenced edges chamfered `distance` along each face. |
| `filleted(model, references, radius, tag, continuity)` | The referenced edges filleted to `radius`: G1 (circular, `1`), G2 (`2`) or G3 (`3`). |
| `edge_references(model, edges)` | References to edges, given by their B-rep numbers (what a viewport pick gives). |
| `resolved_count(model, references)` | How many edges the references find, or an error when one is gone. |
| `tagged(model, tag)` | Unnamed faces named `tag/f<i>`, for bodies made without names, such as primitives. |
| `prefixed(model, prefix)` | Every name with `prefix` in front, so a pattern's or mirror's copy is distinct. |

`tag` names the feature (luced-3d passes the node). Edges may be outside (convex) or inside
(concave) corners. Empty `references` mean every straight convex edge between two flat faces. Rims
are blended only when referenced. A rim is a circle, or an arc of one, between a flat face and a
cylinder or cone about it: a hole's rim, a boss's edge, a countersink's rim.

## Names

Edge numbers change whenever anything upstream changes, so a fillet can't remember its edges by
number. This is the topological naming problem; the study's §4 compares how OpenCASCADE, FreeCAD and
Onshape handle it. Here, every face carries a name saying what made it (`naming.lucb`), and edges are
referred to by the faces they lie between (`references.lucb`).

**Face names** are `tag/role`:

| Made by | Role | Which face |
|---|---|---|
| Extrude | `start<c>`, `end<c>` | The caps of the region whose outer loop has sketch curve `c` (its lowest). |
| | `side<c>.<k>` | The side swept by piece `k` of sketch curve `c` (a polyline's segment, say). |
| Revolve | `side<c>.<k>` | As Extrude's; a sphere's two halves get `a` and `b` after it. |
| Fillet, Chamfer | `blend[<edge>]` | The blend of an edge, `<edge>` its two faces' names. |
| | `corner[<faces>]` | A rounded corner, by the names of the faces that met there. |
| A primitive | `f<i>` | By the face's number, which a primitive keeps. |

Sketch curves are numbered in the order the sketch holds them. Editing a dimension keeps every name.
Deleting a curve renumbers the curves after it.

**Through booleans,** each kept piece keeps its face's name. When a face is split into several pieces,
they all share the name. Flush faces merged into one carry all their names, apart by `|`. Pattern and
mirror copies get a prefix, so they don't take the original's names.

**An edge reference** is its two faces' names, sorted, and where its ends were when it was picked:

    #3/side0.1&#3/side0.2@1,0,3,1,1,3

References are apart by `;`. Separators count only outside brackets, so names can nest.

**Resolving** a reference finds the edges between faces of those names (any alias), then:

- **One edge:** that is the edge.
- **Several** (faces split by later features): the nearest to the recorded ends, together with
  others on the same curve whose middles lie between those ends. An edge cut into pieces upstream is
  all of its pieces.
- **None:** the faces no longer meet, and the feature fails with "an edge this feature refers to is
  gone". It never quietly picks another edge.

Edges move with dimension edits, so where an edge was only breaks ties between edges with the same
names.

## How each works

### Profiles (`profile.lucb`)

Sketch curves are read exactly:

- **Lines.** A polyline's segments.
- **Arcs.** A NURBS whose every span is a rational quadratic circular arc becomes arcs. Edit
  Sketch's arcs and circles are built this way. Consecutive arcs of one circle are merged.
- **Other NURBS.** A B-spline edge.

Curves are chained end to end into closed loops. Loops nest by containment: a loop at an even depth
bounds a region, and the loops directly inside it are its holes. Fusion's profiles work the same
way.

Not yet: curves crossing each other are not split into an arrangement. Each closed loop counts as
it is. A closed spline is also not a profile yet.

### Extrude (`extrude.lucb`)

Each region becomes a closed solid:

- two planar caps;
- a side face per profile edge: a plane, a cylinder band, or a B-spline surface swept straight;
- straight edges joining them at the loop vertices.

Every loop runs counterclockwise about its face's outward normal, so every edge is used twice, in
opposite directions.

### Revolve (`revolve.lucb`)

Each profile edge turned about the axis sweeps a face:

| Edge | Face |
|---|---|
| A line along the axis | A cylinder |
| A line at an angle | A cone |
| A line across the axis | A plane |
| An arc centered on the axis | A sphere |
| An arc off it | A torus |

Each vertex off the axis sweeps a circle. A full turn needs no caps: each face's seam is the profile
edge itself, used both ways. A part turn has an arc for each vertex, the profile edge at the start
and its turned copy at the end, and the profile region as a planar cap at each end. A line on the
axis stays put, so both caps share it.

A spline turns as a rational surface of revolution, at most a half turn per face. A full turn is
then two halves, each with a turned copy of the profile at the half-way angle.

A sphere's pole is kept off its faces: along the axis on the face's side, leaning away from the turn's
middle. A pole on a face makes the tessellator flip triangles.

### Booleans (`shape`, `intersect`, `clip`, `split`, `boolean`, `classify`, `merging.lucb`)

This is OpenCASCADE's General Fuse structure, in a lean form:

1. **One working shape.** Both solids go into one shape in world space (`shape.lucb`), with
   vertices merged within the tolerance.
2. **Surface intersection.** Face pairs whose boxes meet are intersected exactly, in closed form
   (`intersect.lucb`). The table:
   - plane/plane: a line;
   - plane/cylinder: a circle, one or two lines, or an ellipse;
   - plane/cone with the axis across the plane: a circle; through the axis: two lines;
   - plane/sphere: a circle;
   - parallel cylinders: lines;
   - cylinders of one radius whose axes cross: two ellipses, one in each plane bisecting the axes;
   - cylinder/sphere with the center on the axis: circles, or one where they touch;
   - sphere/sphere: a circle;
   - plane/torus with the axis across the plane: circles; through the axis: the tube's two
     sections;
   - surfaces of revolution about one axis (cylinder, cone, sphere, torus): circles, where their
     radii agree along the axis.

   Coincident surfaces are reported as such, facing the same way or opposite: planes, cylinders,
   spheres, cones and tori, each found twice.

   Planes within 1e-7 radians of parallel are treated as parallel: noise, not a crossing.

   Analytic pairs the table doesn't cover are traced (`march.lucb`). This covers cylinders at an
   angle, cones at an angle and tori. Each surface is an implicit function, and a curve on both runs
   along the cross product of their gradients. Start points are where one face's boundary, or a grid
   over the face, crosses the other surface. Each start is traced both ways: a step along the curve,
   then Newton back onto both surfaces. A trace ends where the surfaces touch (their gradients
   parallel). Each traced curve becomes an interpolating cubic B-spline
   through points exactly on both surfaces.
3. **Clipping to faces.** Each curve is clipped to both faces (`clip.lucb`). Its crossings with
   either face's boundary are found on samples, then refined by Newton steps on the exact curves.
   Those crossings cut the curve and the boundary edges. The stretches inside both faces become new
   edges.

   A curve lying along an existing edge is that edge: it is cut and imprinted, never doubled.
   Coincident faces imprint each other's boundaries.
4. **Shared edges.** Edges are cut at every crossing. Pieces with the same ends and middle are one
   edge, so the faces on both sides share it.
5. **Face splitting.** Each face is split by tracing (`split.lucb`). From each half-edge, the trace
   takes the first edge clockwise in the tangent plane, about the face's outward normal. Cycles that
   turn positively in the surface's chart bound pieces. Negative ones are holes, put in the smallest
   piece around them.

   Charts that wrap (cylinders, cones, spheres) are tested per loop, with the point moved by whole
   turns into the loop's range. A sphere's loops close over one pole per face.
6. **Classification.** Each piece is classified at a point well inside it: on a coincident face,
   facing the same way or opposite; otherwise inside or outside the other solid. Inside/outside is
   decided by three rays, majority wins (`rays.lucb`).

   Rays meet the other solid's exact faces: closed form for planes, cylinders, cones and spheres,
   sampled and bisected on a torus. A hit counts when it lies inside the face, by the winding of
   the face's chart. Nothing is tessellated, except a solid with B-spline faces, which is meshed
   for its rays.

   The point inside a piece is the best of a coarse grid over its chart, refined about the best
   three times.

   Two shortcuts keep rays to a minimum:
   - **Faces far away:** a face whose box misses the other solid's box is outside it, with no ray.
   - **Connected blocks** (OpenCASCADE's connexity blocks): pieces of one solid sharing an edge
     that keeps clear of the other solid's surfaces are on the same side, so one ray serves them all.
7. **Selection:**
   - union keeps the outside pieces;
   - subtract keeps the first solid's outside pieces and the second solid's inside pieces, turned;
   - intersect keeps the inside pieces;
   - of coincident pieces, one or none is kept.
8. **Merging and output.** Kept pieces of one face, or of flush faces, that share an edge nobody
   else uses and end up facing the same way are merged. That includes a subtracted solid's turned
   face lying flush on the other's. A sphere zone gets a seam meridian, because the tessellator meshes a band
   on a sphere only with a seam. The result is written as a new B-rep. If any edge isn't used
   exactly twice, the operation fails rather than producing a broken solid.

The tolerance comes from the model's size (1e-7 of it). Results are inputs again, so it must not
grow from one operation to the next.

### Chamfer and fillet (`blend.lucb`)

A rim's tool is its corner profile turned about the rim's axis: a cone for a chamfer, a torus for a
fillet.

These are the first rows of OpenCASCADE's ChFiKPart table, made with booleans. Each edge (or chain
of collinear pieces) between two flat faces gets a tool solid and has it subtracted:

- **Chamfer tool:** the triangle between the edge and the points `distance` along each face.
- **Fillet tool:** the region between the edge and the circle touching both faces.

Each tool is swept along the edge. Past an end where the corner is empty (an outside corner), it
reaches a margin further. Where the solid continues past the end (a wall the edge runs into), it
stops flush.

Corners where chamfers meet come out of the booleans as three chamfer planes meeting at a point.
Fusion's setback triangle is not made.

**A plane meeting a cylinder along its length** (a D-shaft's flat, a slot's side) is blended by a
cylinder parallel to the axis. A fillet's center is where the plane moved by the radius meets the
cylinder moved by it: a radius of R − r or R + r, by which sides the material and the corner are on.
Its section closes back along the cylinder's own circle, not a chord, so it takes no material the
fillet keeps. A chamfer runs `size` along the plane and `size` around the circle.

**Inside corners** (concave edges, where faces meet at more than 180° through the solid) take
material rather than losing it. The corner profile is the same shape, lying in the air between the
faces, and its tool is added with a union:
- a straight edge's tool ends exactly at the face at each end when that face is square to the edge;
  otherwise it is swept past and cut by the face's plane;
- a rim's tool (where a boss meets a plate, or a blind hole's floor) is revolved, as for outside
  rims.

Outside tools are subtracted first, then inside ones added. "Every edge" (no references) takes
outside edges only.

**G2 and G3 fillets** are smoother than circular ones: curvature flows into the faces instead of
jumping at the contact lines. Their cross-section is a degree 5 B-spline between the circular
fillet's contact points. Degree 5 is the element every freeform curve and patch here uses.
- **G2:** one span, six control points. The first three on each side lie along that face, at 0, 0.3
  and 0.6 of the way toward the corner.
- **G3:** three spans (knots at the thirds), eight control points. The first four on each side lie
  along that face, at 0, 0.1, 0.2 and 0.6.

The curvature is then zero where the curve meets each face, and for G3 so is its rate of change.
For a square corner with contacts a unit from it, both curves' curvature peaks at 1.17 times the
circle's. Their middles are 0.376 (G2) and 0.360 (G3) from the corner, against the circle's 0.414.
A single degree 5 curve can't make a good G3: it would need its two middle control points on the
corner itself, and its curvature would peak at 3 to 6.5 times the circle's.

The blend face is that curve swept along the edge, a B-spline surface. `splines.lucb` lets booleans
meet a plane with such a swept profile in closed form:
- **A plane crossing the sweep** cuts it in an affine copy of the profile.
- **A plane along the sweep** cuts it in lines through the profile's points on the plane.

At a G2 contact, curves stay within tolerance of the face they touch over a stretch, so two rules
keep that stretch from producing false crossings:
- **Crossings snap:** where a curve runs along an edge up to either one's end, the crossing is that
  end.
- **Splitting looks further:** where two edges leave a vertex along one tangent, the split compares
  them further along to see which turns first.

On a rim, the G2 or G3 profile is turned about the rim's axis. A spline turns as a rational surface
of revolution: two rational quadratic arcs around, each at most a quarter turn. A full turn is two
halves, so each face's chart opens without a seam. Booleans meet such a surface with a plane across
its axis, and with a cylinder or cone about it, in circles: where the profile's height, or radius,
is theirs. Revolve turns sketch splines the same way.

Two G2 or G3 fillets meeting where a third edge stays sharp meet in a miter. Their swept profiles
meet in the plane bisecting their sweeps, through where their contact lines cross, as that plane's
affine copy of either profile. It is checked to lie on the other.

**Setback corners.** Three G2 or G3 fillets meeting at a corner of three flat faces can't be rounded
by a ball: no sphere is tangent to them, since their sections aren't circles. So each fillet stops
short of the corner, at its setback (setback.lucb), as Fusion and SolidWorks do. That is a plane
square to its edge, past where the next fillets' contacts cross on the faces, by half the largest
contact distance (1.5 r at a square corner).

The hole left is six-sided. Its sides alternate between the fillets' end sections (A curves) and
curves on the faces (B curves). A B curve runs between two fillets' contacts, its legs along their
contact lines, built like the profile. Six quads fill the hole, one at each of its corners, meeting
at a center (patch_fit.lucb). Each is a degree 5 B-spline net: one span for G2; for G3, two (a
curve's half, cut at its middle by knot insertion, keeps two of its three spans).
- **Outer sides:** exact. Next to an A curve, a quad's first rows are the curve's control points
  moved along the sweep. Next to a B curve, they lie in its plane. So the corner meets the fillets
  and faces with their own continuity: two rows beyond the side for G2, three for G3.
- **Seams:** G1 exactly, with the fixed coefficients of a vertex where six meet (one quad's cross
  derivative is the next one's turned back, plus t times the seam's). G2 holds by least squares, and
  thin-plate energy picks among what's left. It is one linear solve. Across the seams the surface
  normals agree within 0.00025 degrees (G3; G2's within 0.000003).

The study's single trimmed surface was tried first. Its sides only came within 1e-5 of the corner's
size, which the booleans can't use. The corner tool is the region between the sharp corner and the
patch, inside the setback planes. It is taken away after the edges' tools, which end exactly at the
setbacks, and added at an inside corner. Its patch's control points stay between the faces and the
setback planes, which is checked. So it meets the body only along curves both carry: a B-spline
whose control points a plane keeps apart from the other surface (kept_apart) adds no curves.

**Too large.** Before any tool, each edge's blend must fit:
- **Its faces:** samples along the edge step into each face by the contact distance, and must stay
  on it.
- **Its rounded corners:** must leave the edge some length.
- **The whole-body rounding:** must leave each shrunk edge some length.

Otherwise the error gives the size the blend must stay under, for example "a fillet radius of 1.5
is too large for this edge's faces: it must be less than 1.0". Rolling on onto the next face, as
Parasolid's overflow does, is not done.

**Tangent chains.** Like Fusion's Tangent Chain, a referenced edge brings the edges that continue
it tangentially with the same convexity, such as a rounded slot's rim or the edges around an
earlier fillet. Pass `chain = false` to blend only the edges named.

Arcs of rims (an arc between a flat face and a cylinder about it) are blended by a corner profile
turned part way along them, a part torus or cone. Where chained edges meet, each tool ends square
at the joint, so neighbouring tools share their end planes. A free end of an arc reaches past, as a
line's does, when the corner beyond is empty.

### Fillets meeting at corners (`corners.lucb`)

- **Two fillets** of one radius meeting where the third edge stays sharp meet in a miter. Both
  tools reach past the corner, and their cylinders cross in two ellipses, one in each plane bisecting
  the axes. `intersect.lucb` gives those in closed form.
- **Three fillets** meeting at a corner of three flat faces are rounded by a ball, as OpenCASCADE's
  `Rotule` does. The ball's center is where the faces' planes, moved in by the radius, meet. Each
  edge's tool stops at the plane through that center square to the edge, where its cylinder touches
  the ball in a great circle. A corner tool then takes what is left: the region beyond those three
  planes, reaching past the faces, minus the ball. That leaves the sphere patch.
- **Inside corners** work the same way, mirrored: the ball rolls in the air, on the faces' planes
  moved out by the radius, and the corner tool adds material around it. A pocket's floor edges meet
  in miters; with its walls' edges too, its corners are balls.
- **Inside and outside fillets meeting** where three edges meet are made in two passes, ordered by
  OpenCASCADE's pivot rule:
  - One outside edge with two inside ones (a boss's upright on a plate): the outside edge first.
    The inside edges then run around its fillet as a tangent chain, a part torus at the corner.
  - Two outside edges with one inside one (an L's inside upright under its top edges): the inside
    edge first. The outside edges then run around it.

  The second pass finds its edges by reference on the first pass's result.
- **Anything else** is refused: more than three edges at a corner, fillets meeting where the faces
  aren't flat, inside and outside fillets meeting where only two edges are picked, or orders that
  conflict along one edge.

Tangent faces meet along these circles, so booleans also cut an edge wherever a vertex lies inside
it, or wherever an edge lying along it is cut (OpenCASCADE's vertex/edge interference). Without
that, two copies of one circle could be split at different points.

A sphere patch bounded by three great-circle arcs, as at these corners, is meshed on a barycentric
grid (`spherical_triangle.lucb`, mesher 7). Its border is the arcs' own samples. The interior points
are the corner directions weighted and normalized. This gives near-equilateral triangles with no pole.
It is used when the arcs are sampled alike and finely enough for the meshers' deviation; otherwise the
general meshers take the patch.

### Every edge rounded whole (`rounded.lucb`)

When no edges are picked, filleting every edge of a convex polyhedron is built whole, as a
Minkowski sum: the solid shrunk
by the radius, then grown back by a ball. Each face keeps its plane with its corners moved in, each
edge becomes a cylinder band about the shrunk edge, and each corner becomes a sphere patch bounded
by its edges' arcs. A box comes out with 26 faces. Each sphere patch's pole sits in its middle, which keeps its
tessellation even.

## Limits, and what comes next

Roughly in order of usefulness:

1. **Fillet corners beyond three edges,** and G1 setback corners (the same patch with one row:
   unequal radii, Fusion's other corner type).
2. **Blends on other curved edges:** ellipses, and edges between two curved faces.
3. **B-spline surfaces in booleans** beyond a plane meeting a swept profile: marching on parametric
   surfaces, and rays through them (classifying a body with B-spline faces meshes it now, most of a
   G2 corner's time).
4. **Sweep and loft;** tapered extrudes; Extrude "to object".
5. **Sketch arrangement.** Crossing curves split into regions, then a constraint solver.
6. **Shell,** offset, draft and press/pull.
