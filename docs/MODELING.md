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
| `revolved(set, axis, tag)` | Each region turned a full turn about the plane's u (`0`) or v (`1`) axis. |
| `profile_count(set)` | How many regions the sketch has. |
| `combined(a, b, operation)` | `0` union, `1` `a` without `b`, `2` intersection. |
| `chamfered(model, references, distance, tag)` | The referenced edges chamfered `distance` along each face. |
| `filleted(model, references, radius, tag, continuity)` | The referenced edges filleted to `radius`: G1 (circular, `1`), G2 (`2`) or G3 (`3`). |
| `edge_references(model, edges)` | References to edges, given by their B-rep numbers (what a viewport pick gives). |
| `resolved_count(model, references)` | How many edges the references find, or an error when one is gone. |
| `tagged(model, tag)` | Unnamed faces named `tag/f<i>`, for bodies made without names, such as primitives. |
| `prefixed(model, prefix)` | Every name with `prefix` in front, so a pattern's or mirror's copy is distinct. |

`tag` names the feature (luced-3d passes the node). Edges may be outside (convex) or inside (concave)
corners. Empty `references` mean every straight convex
edge between two flat faces. Rims are blended only when referenced: a rim is a full circle between a
flat face and a cylinder about it, such as a hole's rim or a boss's edge.

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

### Booleans (`shape`, `intersect`, `clip`, `split`, `boolean`, `classify`, `merging.lucb`)

This is OpenCASCADE's General Fuse structure, in a lean form:

1. **One working shape.** Both solids go into one shape in world space (`shape.lucb`), with
   vertices merged within the tolerance.
2. **Surface intersection.** Face pairs whose boxes meet are intersected exactly, in closed form
   (`intersect.lucb`). The table:
   - plane/plane: a line;
   - plane/cylinder: a circle, one or two lines, or an ellipse;
   - plane/cone with the axis across the plane: a circle;
   - plane/sphere: a circle;
   - parallel cylinders: lines;
   - cylinders of one radius whose axes cross: two ellipses, one in each plane bisecting the axes;
   - cylinder/sphere with the center on the axis: circles, or one where they touch;
   - sphere/sphere: a circle;
   - plane/torus with the axis across the plane: circles;
   - surfaces of revolution about one axis (cylinder, cone, sphere, torus): circles, where their
     radii agree along the axis.

   Coincident surfaces are reported as such, facing the same way or opposite.

   Analytic pairs the table doesn't cover are traced (`march.lucb`). This covers cylinders at an
   angle, cones at an angle and tori. Each surface is an implicit function, and a curve on both runs
   along the cross product of their gradients. Start points are where one face's boundary, or a grid
   over the face, crosses the other surface. Each start is traced both ways: a step along the curve,
   then Newton back onto both surfaces. Each traced curve becomes an interpolating cubic B-spline
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
7. **Selection:**
   - union keeps the outside pieces;
   - subtract keeps the first solid's outside pieces and the second solid's inside pieces, turned;
   - intersect keeps the inside pieces;
   - of coincident pieces, one or none is kept.
8. **Merging and output.** Kept pieces of one face, or of flush faces, that share an edge nobody
   else uses are merged. A sphere zone gets a seam meridian, because the tessellator meshes a band
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
jumping at the contact lines. Their cross-section is a Bezier curve between the circular fillet's
contact points. For G2, it is degree 5, and the first three control points on each side lie along
that face. For G3, it is degree 7, with four on each side. The curvature is then zero where the
curve meets each face, and for G3 so is its rate of change.

The points sit at 0, 0.3 and 0.6 of the way toward the corner (G3: 0, 0.2, 0.4 and 0.6). This keeps
the curve's middle near the circle's: 0.27 of the way in for a square corner, against the circle's
0.29.

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

G2 and G3 fillets meeting at a corner, and those of rims, are refused for now.

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
- **Anything else** is refused: more than three edges at a corner, fillets meeting where the faces
  aren't flat, or inside and outside fillets meeting at one corner.

Tangent faces meet along these circles, so booleans also cut an edge wherever a vertex lies inside
it, or wherever an edge lying along it is cut (OpenCASCADE's vertex/edge interference). Without
that, two copies of one circle could be split at different points.

### Every edge rounded whole (`rounded.lucb`)

When no edges are picked, filleting every edge of a convex polyhedron is built whole, as a
Minkowski sum: the solid shrunk
by the radius, then grown back by a ball. Each face keeps its plane with its corners moved in, each
edge becomes a cylinder band about the shrunk edge, and each corner becomes a sphere patch bounded
by its edges' arcs. A box comes out with 26 faces. Each sphere patch's pole sits in its middle, which keeps its
tessellation even.

## Limits, and what comes next

Roughly in order of usefulness:

1. **Fillet corners beyond three edges,** and setback corners (an n-sided patch, Fusion's other
   corner type).
2. **Blends on other curved edges:** a cone's rims, ellipses, and edges between two curved faces.
3. **B-spline surfaces in booleans** beyond a plane meeting a swept profile: marching on parametric
   surfaces. That would also bring G2 and G3 corners and rims.
4. **Partial revolves,** sweep and loft; tapered extrudes; Extrude "to object".
5. **Sketch arrangement.** Crossing curves split into regions, then a constraint solver.
6. **Inside and outside fillets meeting at one corner;** shell, offset, draft and press/pull.
