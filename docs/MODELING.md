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
| `extruded(set, distance)` | The sketch's closed curves in `set`, as profile regions, each swept `distance` along the plane's normal. |
| `profile_count(set)` | How many regions the sketch has. |
| `combined(a, b, operation)` | `0` union, `1` `a` without `b`, `2` intersection. |
| `chamfered(model, edges, distance)` | Edges chamfered `distance` along each face. |
| `filleted(model, edges, radius)` | Edges filleted to `radius`. |

`edges` are the body's B-rep edge numbers. An empty list means every straight convex edge between
two flat faces.

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

### Booleans (`shape`, `intersect`, `clip`, `split`, `boolean.lucb`)

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
   - sphere/sphere: a circle.

   Coincident surfaces are reported as such, facing the same way or opposite.
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
   decided by three rays through the other solid's tessellation, majority wins.
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

These are the first rows of OpenCASCADE's ChFiKPart table, made with booleans. Each edge (or chain
of collinear pieces) between two flat faces gets a tool solid and has it subtracted:

- **Chamfer tool:** the triangle between the edge and the points `distance` along each face.
- **Fillet tool:** the region between the edge and the circle touching both faces.

Each tool is swept along the edge. Past an end where the corner is empty (an outside corner), it
reaches a margin further. Where the solid continues past the end (a wall the edge runs into), it
stops flush.

Corners where chamfers meet come out of the booleans as three chamfer planes meeting at a point.
Fusion's setback triangle is not made.

## Limits, and what comes next

Roughly in order of usefulness:

1. **Fillets meeting at a corner.** Two fillet cylinders meeting at an angle need cylinder/cylinder
   intersection, which is a marching case. A sphere patch at a three-edge corner would follow the
   KPart table.
2. **Blends on curved edges.**
   - A circle between a plane and a cylinder (a hole's rim) chamfers to a cone and fillets to a
     torus. Booleans would then need cylinder/cone, plane/torus and cylinder/torus.
3. **General surface intersection.** Marching with start points from boundary crossings or a mesh
   pre-pass, as truck does. That gives cylinders at any angle, cones at an angle, tori and
   B-spline sides.
4. **Revolve, sweep, loft;** two-sided and tapered extrudes; Extrude "to object".
5. **Sketch arrangement.** Crossing curves split into regions, then a constraint solver.
6. **Persistent naming.** Each operation would record which faces and edges it made from which.
   Downstream edge references would then survive upstream edits; today they are B-rep numbers. See
   the study's §4.
7. **Concave edge blends** (adding material), shell, offset, draft and press/pull.
