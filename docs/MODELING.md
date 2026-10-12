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
| `extruded_sides(set, distance, other, tag, taper, other_taper, start)` | Fusion's Extrude directions: `distance` along the normal and `other` against it (one side, two sides, or symmetric with both alike), each side leaning in by its taper angle (Fusion's sign is the other way: luced-3d turns it over). |
| `extruded_through(set, model, tag, direction, taper, profiles, start)` | Extent All: past the whole body along the normal (`1`), against it (`-1`) or both ways (`0`), from the plane moved `start` along the normal. |
| `extruded_to(set, model, tag, face, offset, taper, profiles, start, extend)` | Extent To Object, from the plane moved `start`, as Fusion's Extend says: `0` until the extrusion meets the named face's surface (extended past the face), `offset` further; `1` until it meets that face and the faces next to it; `2` until it meets the body holding the face; `3` through that body. With no face, `2` and `3` take the whole model. |
| `extruded_from(set, model, tag, face, offset, extent, distance, target, target_offset, taper, profiles)` | Start: Object. From the named face's surface (moved `offset` along the way), a `distance` (the end follows the face's shape moved on), to face `target`, or through all. A face parallel to the sketch is a start offset. |
| `revolved_to(set, origin, direction, tag, model, face, way, profiles)` | Revolve's To extent: turned about the axis, toward the plane's normal (`way` 1) or against it (-1), until the turn meets the named face's surface. |
| `revolved(set, axis, tag, angle)` | Each region turned about the plane's u (`0`) or v (`1`) axis: a full turn, or `angle` radians closed by the profile at each end. |
| `revolved_about(set, origin, direction, tag, angle)` | The same about any line in the sketch's plane: a sketch line, a body edge, a construction axis. |
| `edge_axis(model, reference)` | A line edge's line, or a circular edge's axis, for Revolve. |
| `edge_frame(model, reference)`, `face_point(model, name)` | Where a feature's viewport handle stands (`handles.lucb`): an edge's middle, its direction and the way out of the corner between its faces (a blend's radius arrow); a point near a face's middle and its outward normal (Press Pull's arrow): on a B-spline face the nearest point to the middle of its edges, searched for (a lattice of starts, then Newton) when that middle lies off the surface, as on a G2 or G3 strip. |
| `profile_count(set)` | How many regions the sketch has. |
| `profile_keys(set)`, `profile_at(set, point)` | The keys of the sketch's faces; the key of the face under a point (a click). `extruded` and `revolved` take keys to choose faces. |
| `CadProfileMap.of(set)`, `key_at(point)` | The sketch's faces laid out once (`profile_map.lucb`), so the face under a moving pointer is a containment test, not an arrangement made again. |
| `profile_fills(set, keys)` | The regions of the profiles `keys` names (empty: every profile), triangulated flat on the plane by the trim mesher (`polygons/`), to shade them. A region whose loops touch is left out. |
| `profile_anchor(set, keys)` | Where a feature's arrow stands on those profiles (empty: the closed shapes Extrude takes): a point inside them (the middle of their area when it lies on them, else of their largest triangle), the plane's normal, their area. |
| `inside(model, point)` | Whether a point lies inside a closed solid, by rays through its exact faces (`rays.lucb`): what an automatic Join or Cut asks. |
| `shelled(model, faces, thickness, tag, side, chain, outside_thickness)` | Shell: hollowed to a wall `thickness` thick, the faces named opened (none: a closed void), the wall inside (`0`), outside (`1`) or both ways (`2`): half each way, or with `outside_thickness` 0 or more Fusion's Inside (`thickness`) and Outside thicknesses. With `chain` (Fusion's Tangent chain) the faces meeting opened ones tangentially open too. |
| `combined(a, b, operation)` | Combine: `0` join, `1` cut (`a` without `b`), `2` intersect. Fusion's Keep Tools is the caller keeping `b`; several tools are combined one after another. |
| `split_by_plane(model, origin, normal, tag)`, `split_by_face(model, tool, face, tag, extend)` | Split Body: the piece on the plane's normal side (inside a curved face's surface) and the piece beyond, as two lumps of one model. With `extend` off (Fusion's Extend splitting tool), the face cuts only where it reaches. |
| `sliced(model, origin, normal, tag)` | Fusion's sketch Slice: what lies behind the plane (against its normal), the half-space in front taken away and the cut capped by faces named `<tag>/to/f<i>`; the model itself when nothing of it is in front; refused when all of it is. |
| `lump_count(model)`, `lump(model, at)` | A model's separate pieces, each as a body of its own. |
| `moved_body(model, by, origin, axis, angle, prefix)` | Move/Copy: turned about a line, then moved; a copy's faces take `prefix` so features tell it from the original. |
| `chamfered(model, references, distance, tag, chain, corner)` | The referenced edges chamfered `distance` along each face; where three meet at a corner, Fusion's Corner Type: `0` Chamfer, `1` Miter, `2` Blend (each checked against Fusion itself: Chamfer corners below). |
| `chord_radius(model, references, chord, chain)` | Fusion's Chord Length: the radius whose fillets leave the faces `chord` apart. |
| `full_round(model, center, tag, continuity, side1, side2)` | Fusion's Full Round Fillet: the flat center face rounded away whole, tangent to it and its sides. |
| `variable_filleted(model, reference, positions, radii, tag, smooth, continuity, chain, tangency)` | Fusion's Variable radius: one straight edge between flat faces, a tangent chain round a flat face or a loop between curved faces, radii at positions 0 to 1 along it, smooth or straight between, G1 to G3, under a Tangency weight. |
| `filleted(model, references, radius, tag, continuity, chain, setback, tangency)` | The referenced edges filleted to `radius`: G1 (circular, `1`), G2 (`2`) or G3 (`3`); with `setback`, Fusion's Setback corner type for G1 corners too; `tangency`, Fusion's Tangency weight (0.1 to 2, 1 the default: Tangency weight below). |
| `rule_edges(model, rounds, fillets, faces, others)` | Fusion's Rule Fillet (`rules.lucb`): references to the blendable edges a rule takes, convex ones (rounds), concave ones (fillets) or both; with `faces` named, only edges on those faces, and with `others` too, only edges between the two sets. Made again on every cook, the edges follow the body. |
| `chamfered_sides(model, references, distance, second, angle, tag, chain, flip)` | Fusion's Two distances (`second` along the other face) and Distance and angle (`second` 0: leaving the first face at `angle`) chamfers (`chamfers.lucb`). The first face of each edge is the one more of the chosen edges lie on (a loop's own face), a tie by name; `flip` swaps. Every edge Equal distance takes (loops round a flat face, rims, curved faces, inside edges); each distance runs straight from the edge to the chamfer's contact in the plane square to the edge (`chamfers.lucb`). |
| `edge_references(model, edges)` | References to edges, given by their B-rep numbers (what a viewport pick gives). |
| `resolved_count(model, references)` | How many edges the references find, or an error when one is gone. |
| `tagged(model, tag)` | Unnamed faces named `tag/f<i>`, for bodies made without names, such as primitives. |
| `prefixed(model, prefix)` | Every name with `prefix` in front, so a pattern's or mirror's copy is distinct. |

Direct modeling's operations are on `CadDirect` (`direct_api.lucb`), on faces named as `face_frame`
takes them, several apart by `;`:

| `CadDirect.` | What it does |
|---|---|
| `offset_faces(model, faces, distance, chain)` | Offset Face, Press Pull on a face: each face moved along its outward normal (a plane moved, a cylinder's, cone's, sphere's or torus's radius changed). With `chain` (the default) the faces meeting them tangentially move too. |
| `moved_faces(model, faces, by)`, `rotated_faces(model, faces, origin, axis, angle)` | Move Face: translated, or turned about a line. |
| `replaced_faces(model, faces, target, target_faces, chain)` | Replace Face: the faces (with `chain`, their tangent chain too) on the target faces instead (names apart by `;`; empty: the whole target body). Targets on one surface move the faces onto it, as do targets matching the faces one for one; other targets on several surfaces, faces of a solid or of an open surface body, trim or extend the body to them (`face_replace.lucb`, `face_quilt.lucb`). |
| `deleted_faces(model, faces)` | Delete with healing: holes and bosses whole, one fillet, chamfer or corner face, between flat faces any number of them at once (a rounded chain and its corners, strips apart), or a band between one face and a ring of faces, curved ones too (a top chain beside rounded uprights). |
| `split_faces(model, faces, tool, tool_face, tag, extend)`, `split_faces_by_plane(model, faces, origin, normal, tag)` | Fusion's Split Face: the faces split where a face's surface (extended; with `extend` off, the face itself, right across each face) or a plane crosses them; one closed solid with the new edges. The pieces on the surface's outer side (a plane's normal side, the side a tool face faces; outside a curved surface) are named `<tag>/split[<name>]`, the others keep the face's name, so each can be picked apart. |
| `tangent_chain(model, faces)` | Fusion's Tangent chain: the faces and every face meeting them tangentially, and so on, by name. |
| `copied_faces(model, faces, prefix)` | Move/Copy of faces with Create copy: an open surface body of copies of the faces, their names with `prefix` before them. |
| `face_names(model, faces)` | The names of B-rep face numbers (a pick's), apart by `;`. |

Edit CAD (luced-3d's direct modeling) reaches these through the viewport's pick mesh, with
`CadEdits` (`cad_edits.lucb`):

| `CadEdits.` | What it does |
|---|---|
| `model_faces(pick, group, model)` | The names of the B-rep faces of one model a face group names, apart by `;`: what the face operations take. |
| `model_share(pick, group, model)` | How much of a model a group names: nothing, some faces, or all of it (a whole body). |
| `transformed_faces(set, pick, group, numbers)` | Transform Components' numbers on what a group names: whole models placed, faces of the others turned about the pivot and moved (`rotated_faces`, `moved_faces`); faces never scale. |
| `named_models(set, prefix)` | Models with unnamed faces (a STEP import's) named `<prefix><i>/f<n>`, so they can be referred to; the same set when all have names. |
| `carried(before, selection, now)` | A face or edge selection on one pick mesh carried onto another of the same B-rep faces and edges (meshed again after a face moved), as a group. |

A set's sketches, Fusion's sketches in direct modeling, are objects too: `CadSketches`
(`cad_sketches.lucb`), a component holding each sketch's name, CadSketch text, plane and
visibility (`with_sketch`, `with_visible`, `without_sketch`, `count`, `find`, `name`, `text`,
`origin`, `u`, `v`, `visible`). It places with the set (the plane moves and turns; the sketch keeps
its sizes), joins, filters by name as an object path and is saved in geometry files.

`tag` names the feature (luced-3d passes the node). Edges may be outside (convex) or inside
(concave) corners. Empty `references` mean every straight convex edge between two flat faces. Rims
are blended only when referenced. A rim is a circle, or an arc of one, between a flat face and a
cylinder or cone about it: a hole's rim, a boss's edge, a countersink's rim.

## Seams

A periodic face here is bounded by a seam: an edge the face uses both ways (a cylinder's or a
torus's closing line, a turned strip's section at its loop's vertex), or an edge between two faces
on one surface (a sphere made of two hemispheres: its equator). Fusion's periodic faces have none
(probe 7 of 2026-10-12: a cylinder has two edges, its circles; a sphere and a torus none). So seams
stay in the B-rep (the tessellator, the booleans and the charts use them) but are **not drawn**
(`overlay.lucb` `seam_edges`: the edges' lines and the preview's boundaries) **and not picked** (Edit
CAD's pick mesh keeps no edge between a face and itself or two faces of one surface).
`fusion_probe_tests` checks a dome's rim strip draws only its contact circles, a cylinder its rims,
a sphere and a torus nothing.

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

### Profiles (`profile.lucb`, `arrangement.lucb`)

Sketch curves are read exactly:

- **Lines.** A polyline's segments.
- **Arcs.** A NURBS whose every span is a rational quadratic circular arc becomes arcs. A Sketch's
  arcs and circles are built this way. Consecutive arcs of one circle are merged.
- **Other NURBS.** A B-spline edge.

Then, as Fusion's profiles: every piece is split where it meets another (crossing, touching, or a
curve's end on it; lines and arcs in closed form, B-splines by their samples refined by Newton and
split by knot insertion). The pieces join at shared points into a planar graph, dangling pieces are
dropped (a loose line inside a region splits nothing), and the graph's faces are traced: each
bounded face is a profile. A face's **key** names it by the sketch curves round it and the way each
runs (`0+,1-`), so it stays the same face as dimensions change; two crescents of overlapping
circles differ by which circle each runs along.

A feature takes the faces its keys name, or by default islands alternating from the outside in (a
face's depth: how many faces of other connected pieces hold it): a plate's hole stays empty,
overlapping shapes join. The taken faces are outlined together, so faces next to each other become
one region, and parts of one curve going on the same way join back into one edge (one side face).
Not yet: a closed spline as a profile.

### Extrude (`extrude.lucb`)

Each region becomes a closed solid:

- two planar caps;
- a side face per profile edge: a plane, a cylinder band, or a B-spline surface swept straight;
- straight edges joining them at the loop vertices.

Every loop runs counterclockwise about its face's outward normal, so every edge is used twice, in
opposite directions.

### Extents (`extents.lucb`, `extent_targets.lucb`)

Fusion's Extrude dialog: two sides, each with its own distance and taper. The solid is built
directly, without booleans, as **levels**: the profile at heights along its normal, each edge moved
outward in the plane by the level's growth, `-height × tan(taper)`.
- A line moves parallel to itself; an arc about its center, its radius changing by the growth.
- Neighbors meet again where the moved edges cross (closed form: line/line, line/circle,
  circle/circle, the crossing nearest the vertex); where they run on tangentially, the vertex moves
  along its normal.
- Between two levels each edge sweeps a face: a plane for a line (slanted by the taper), a cylinder
  or a cone for an arc, a B-spline swept straight for a spline. A spline takes no taper (refused
  with a message).
- The edge where two sides meet between levels (`upright`) is a line where both are planes, where
  they run on tangentially, or where the levels lean alike. A tapered plane meeting a cone at an
  angle meets it in a conic (two cones: a quartic): that edge is traced, the moved edges' crossing
  at 33 heights, as a cubic through them. A line between its ends lay on neither side (0.009 off on
  a 12 high drafted ring), and booleans on the body failed or misjudged it.
- One side is two levels; two sides are three (the profile in the middle), or two when neither
  side tapers. Where two sides of different taper meet at the profile, each side face breaks
  there: the lower part is named `back<c>.<k>`.
- A taper that shrinks an edge to nothing, turns an arc over, or keeps neighbors from meeting is
  refused: "the taper is too steep for this profile".

Extents that read a body:
- **All**: the body's reach along the normal (its faces' boxes) and a margin. Refused when the body
  is not on that side.
- **To a face**: a plane parallel to the sketch is a distance. Any other plane, or a cylinder, cone,
  sphere or torus, stops the extrusion where it meets the face's surface, extended past the face,
  as Fusion's To Object does. The extrusion is made long; the half-space past a plane (a large box
  on it), or the solid of a curved surface, is taken away, or kept when the sketch is inside it.
  The lump holding the profile's start cap is the result (`lumps.lucb`), so whatever lies past
  the surface is dropped. Offsets move the surface along the way the extrusion runs. B-spline faces
  are refused.
- **To a body**: the long extrusion less the body, the lump at the profile.
- **Through a body**: the long extrusion less the pieces of it past the body's last face (those
  holding its far cap); refused when the extrusion passes beside the body.
- **To adjacent faces**: as to a body, then checked: the piece kept may stop only on the face and
  the faces sharing an edge with it, and must not run on past them (its far cap left). Otherwise it
  is refused, naming To body and To selected face.
- **A start offset** moves the levels' start along the normal for every extent; a target behind
  the start is refused.

**Taper's sign, verified against Fusion 2705** (probe of 2026-10-12): a positive taper leans the
sides out, a 20 square extruded 10 at +10° spanning -1.7633 to 21.7633 (20 + 2 × 10 tan 10° = 23.53
across at the top). The kernel's positive taper leans in; luced-3d's Extrude turns the sign over, and
`cad_extent_tests` checks Fusion's numbers.

**Start: Object** (`extent_starts.lucb`). A face parallel to the sketch is a start offset. Any
other plane, cylinder, cone, sphere or torus bounds the extrusion: it is made long both ways, the
part past the start surface (moved by the offset) is kept, and the far end is the start surface
moved the distance along the normal, the target face's surface, or a plane past the body (All).
Which side of a curved surface is past it is judged where the profiles' line meets it. A taper
needs a parallel start. From a slanted or curved face, the volume is the profile's area times the
distance, which `extent_start_tests` checks.

**Revolve To** (`revolve_to.lucb`). The profiles are turned almost a whole turn the chosen way,
the side of the face's surface away from them is taken away, and the piece holding the profile
is kept (the turn's start cap, or its end cap when it was turned back to run the other way);
refused when the turn comes round without meeting the surface.

**Thin walls** (`sketch/thicken.lucb`, `CadSketch.thicken`). Thin Extrude's band round a chain:
each side's boundary is the chain's offset copy (the chain itself at 0), a closed chain thickened
inside and outside (by which way it turns), an open one to its left and right with its ends
closed by lines; the chain becomes construction when both sides have a thickness. Fusion's wall
locations, **verified against Fusion 2705** (probe of 2026-10-12), are luced-3d's mapping onto it:
Side 1 is inside a closed profile and left of an open curve as it was drawn, Side 2 outside and
right, Center half each way. A 20 × 10 rectangle, wall 2, 5 high: Side 1 spans 20 × 10 (520), Side 2
24 × 14 (680), Center 22 × 12 (600); a line drawn (0, 0) to (20, 0) goes to y 0..2, -2..0 and
-1..1. `cad_parity_tests` checks those numbers.

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
   spheres, cones and tori, each found twice. A B-spline surface lies on another surface when a
   9 × 9 lattice of its points, projected onto the other, lands on it (within ten times the
   tolerance, normals parallel) wherever it lands inside the other's domain, and enough points do
   (`spline_coincidence.lucb`): a rounded box's strips on its copy's, moved along them, or a strip
   on a cylinder it was rolled from. The first point off the other surface ends the test, so
   crossing surfaces cost a projection or two.

   Planes within 1e-7 radians of parallel are treated as parallel: noise, not a crossing.

   Pairs the table doesn't cover are traced (`march.lucb`). This covers cylinders at an angle,
   cones at an angle, tori, and B-spline surfaces against anything (a fillet's strip cut by a tilted
   plane or pin, two rounded bodies' strips crossing). Each surface is a field: an analytic
   surface's implicit function, a B-spline's signed distance to its nearest point (along its normal
   there, Newton from the nearest point found last, a lattice of starts for each new trace). A curve
   on both runs along the cross product of their gradients. Start points are where one face's
   boundary, or a grid over the face, crosses the other surface. Each start is traced both ways: a
   step along the curve, then Newton back onto both surfaces. A trace ends where the surfaces touch
   (their gradients parallel). Each traced curve becomes an interpolating cubic B-spline through
   points exactly on both surfaces; it is traced again at half the step until the cubic stays within
   half the tolerance of both surfaces between its points.

   B-spline pairs with a closed form keep it: a plane against a profile swept straight, two sweeps
   meeting in a miter, a turned profile against a plane or a cylinder or cone about its axis.
3. **Clipping to faces.** Each curve is clipped to both faces (`clip.lucb`). Its crossings with
   either face's boundary are found on samples, then refined by Newton steps on the exact curves.
   Those crossings cut the curve, and they cut a boundary edge only where the other face is: a
   crossing outside it ends no new edge, and would leave a stray vertex on the edge (a notch at
   one end of a box would cut the far end's edges too). The stretches inside both faces become
   new edges.

   A curve lying along an existing edge is that edge: it is cut and imprinted, never doubled.
   Curves of one face pair crossing each other are cut where they cross, at one vertex: two equal
   pipes crossing meet in two ellipses that cross where the pipes touch.

   A section along which the two faces are tangent is a touching contact: at a quarter, half and
   three quarters of it their normals are parallel (to a thousandth of a radian) and stepping off it
   across one face, both ways, stays on one side of the other (the faces part, they do not cross).
   Joined where they touch from outside, or subtracted where one lies inside the other touching its
   face, the bodies would leave an edge four faces meet at: the operation refuses, naming the result
   non-manifold. So does a result whose edges are each used twice but where a kept face passes
   through one of them (no thickness there), one whose kept faces both reach such a contact (their
   pieces either side merged, it closes but is pinched to nothing there), and one whose only open
   edges are such four-face edges. Vertices merge within the sum of their zones: five tolerances
   each, or a tolerant edge's or vertex's precision; a written vertex's precision holds its edges'.
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
   facing the same way or opposite; otherwise inside or outside the other solid. The point is the
   candidate farthest from the piece's boundary on a grid over its chart, refined; a piece thinner
   than the grid's cells (a thin shell's ring round a hole) takes candidates stepped in beside the
   middle of each boundary segment instead. Inside/outside is
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
   face lying flush on the other's. Flush pieces on B-spline faces stay separate faces sharing an
   edge: neither surface need reach over the other's part. A sphere zone gets a seam meridian where
   both its circles have a vertex on the chart's seam (else the tessellator meshes it about an axis
   of its own), because the tessellator meshes a band on a sphere only with a seam. The result is
   written as a new B-rep. If any edge isn't used exactly twice, the operation fails rather than
   producing a broken solid.

The tolerance comes from the model's size (1e-7 of it). Results are inputs again, so it must not
grow from one operation to the next.

**Split Body** (`split_body.lucb`) uses the same booleans: the body with the cutting surface's
solid and without it. A plane's solid is a large box on its normal side; a cylinder's, cone's,
sphere's or torus's is the primitive of that surface, made long enough. The two results go side by
side into one model, two lumps (`lumps.lucb` finds a model's pieces by the edges its faces share). A
surface that misses the body is refused. With Fusion's **Extend splitting tool off**, the tool is
the face swept along its normal past the body (`face_quilt.lucb`): a piece cut off by the face alone is
a piece; one the sweep's walls or far end cut too lay beside the face, not behind it, and is joined to
the body again. A face that cuts nothing off alone (part way across) is refused, naming the Extend
option. **Slice** (`behind_surface`) is the second boolean alone:
the body without the plane's solid, capped where it was cut; the box round the body says first
whether the plane cuts it at all.

### Chamfer and fillet (`blend.lucb`)

A rim's tool is its corner profile turned about the rim's axis: a cone for a chamfer, a torus for a
fillet. A rim between a flat face and a sphere about its axis (a dome's edge), along its whole loop,
is blended directly, with no booleans (`curved_blend.lucb`, its rows turned exactly:
`curved_turned.lucb`): a fillet is a ball rolling between the plane and the sphere (G1 a torus, as
Fusion's), a chamfer the line between its contacts turned, an exact cone from the plane's circle to
the sphere's (the distance on the sphere a chord to its meridian circle), the sphere's sliver beyond
the chord gone with the face trimmed at the contact. **Verified against Fusion 2705** (probe 5 of
2026-10-12, `fusion_chamfer_tests`): domes cut at their middle, 4 above it and 4 below it, chamfered
Equal distance, Two distances and Distance and angle each way, Fusion's circles to 1e-5 and its
volumes. (The cone tool these took before, through the booleans, could not cut the dome cut above its
middle.) An arc of such a rim alone is chamfered by that tool still (`sphere_rims.lucb`: the section
closes back over the sphere's tangents at the edge and at the contact); filleting it is refused.

These are the first rows of OpenCASCADE's ChFiKPart table, made with booleans. Each edge (or chain
of collinear pieces) between two flat faces gets a tool solid and has it subtracted:

- **Chamfer tool:** the triangle between the edge and the points `distance` along each face.
- **Fillet tool:** the region between the edge and the circle touching both faces.

Each tool is swept along the edge. Past an end where the corner is empty (an outside corner), it
reaches a margin further. Where the solid continues past the end (a wall the edge runs into), it
stops flush.

**Chamfer corners** (`chamfer_corners.lucb`), Fusion's Corner Type where three chamfers meet at
a corner of three flat faces:
- **Miter**: the tools reach past the corner, and the chamfers meet in three planes and a point.
- **Chamfer** (Fusion's default): on each face the two chamfers' contact lines meet at a point,
  P = V + a·(l_b / sin θ) + b·(l_a / sin θ) for the edges' ways a, b from the vertex V, θ apart,
  and legs l_a, l_b on that face; the plane through the three points is the corner's face, and the
  tip beyond it is taken away by a box on that plane. At a square corner with equal chamfers d the
  tip is d³/12, which `chamfer_corner_tests` checks.
- **Blend**: the chamfers set back from the corner and one flat face, a hexagon, closing it. Its
  plane is the Chamfer face's turned the same way, moved in until every contact line runs on past
  its crossing by its chamfer's distance (the longer leg): at a square corner the chamfers end 2 d
  from the vertex, the plane meeting the edges 3 d from it (x + y + z = 48 at the corner (20, 20,
  20), d = 4). Its sides alternate: a
  chamfer's end (where the plane cuts it), then a line across a face. A plane crossing an edge
  beyond its far end is refused. Two distances take it too.

**Verified against Fusion 2705** (probe of 2026-10-12, a 20 mm cube's corner, three chamfers of 4;
`chamfer_corner_tests`): Miter 9 faces, 7568.0; Chamfer the triangle (16, 16, 20), (16, 20, 16),
(20, 16, 16), 10 faces, 7562.6667; Blend Fusion's hexagon (12, 16, 20), (16, 12, 20), (20, 12, 16),
(20, 16, 12), (16, 20, 12), (12, 20, 16), 10 faces all flat, 7456.0. Fusion's hexagon lies 0.0006
further in (its corners at 12.0006, its volume 7456.029); that hair is its blend engine's setback
tolerance, not a rule, so the corners here are the exact 2 d.

**Blend corners with curved faces: the rule.** Fusion's Blend is a vertex blend: the chamfers set
back and a surface fills the hole between their ends and the faces. Its own result on flat faces is
that surface's plane, because every side of the hole (a chamfer's end is a line, a ruled chamfer's
section; a flat face's side is a line) lies in one plane. The rule here follows: **the corner is one
flat face wherever a plane closes it**, a plane cutting the corner's tip off in one loop that crosses
each chamfer and each face once. On a curved face the plane's side is that face's section (an
ellipse's arc on a cylinder, a conic on a cone), still one flat corner face, as long as the face
bulges away from the plane (a convex corner: a pin's side, a boss's). Only where a face curves back
across the plane (a hole's wall within the corner, a saddle) can no plane close it, and only there
would a curved patch (the setback patch of `patch_fit.lucb`) be needed. OpenCASCADE's chamfer corners
follow the same rule (`ChFi3d_ChBuilder_C3`: a plane where one fits, else a constrained filling;
FILLET-STUDY.md, row 6). The corner types act today
where three straight chamfered edges meet (three lines at a vertex: their faces are flat, so a
plane always closes it, and no patch is made); corners where a rim's arc meets straight edges take
the edges' tools as they meet (a miter) for every corner type.

**Chord Length** (`chord_length.lucb`): faces whose ways from the edge are θ apart take contacts
r cot(θ/2) from it, 2r cos(θ/2) apart, so the radius is chord / (2 cos(θ/2)); G2 and G3 fillets
touch where the circle does. Edges whose faces meet at different angles would need different
radii: refused.

**Full round** (`full_round.lucb`): for a flat center face between sides along it, the round is
the circle tangent to the center face and both sides in the section across them. The center face's
edges with the sides must run alike, so every section is the same:
- **Parallel lines** (a rib's top): the sides flat, tapered, or curved along the rib (cylinders
  whose axes run with the edges, a bulging side). Both edges are filleted to the circle's radius:
  the two strips touch the center face at the same line, so it goes and the strips are one cylinder
  (G2 and G3: two strips meeting with zero curvature, joined into one face, below).
- **Concentric arcs** (a curved rib): the sides cylinders or cones about the center face's normal,
  the ends flat. Two fillets' part-torus tools would touch each other along the round's top, which
  the booleans cannot part, so the round is made in two steps: the slab over the chord between the
  side contacts is taken away, then the cap between that chord and the round is added back, both
  turned about the axis along the arcs. The slab's chord runs on past the contacts out of the body
  and its walls stand off the sides, along the axis (cylinders): a slab face on a side's surface, or
  a slab edge along one, left the booleans coincident pieces to part. An end through the axis (a
  revolved rib's) ends both tools flush. Past a leaning end (an extrusion's drafted ends; any flat
  end) the tools turn on until every slab corner has crossed its plane, plus a margin, and a box on
  the plane's outer side, no larger than the tools' end, trims each back to the plane
  (`rib_ends`, `ends_trimmed`); an end that is not flat is refused. The round is one torus at G1,
  two strips of revolution meeting where the center face was at G2 and G3, joined.
- **One face** (`strip_join.lucb`), as Fusion's full round is: G2 and G3's two halves meet where
  the center face was, each surface ending on the same row of its net there. The nets are set end to
  end across that row (the knot between them repeated to the degree, the second net's weights
  scaled to the first's on the row, its knot span in proportion to its length across), the seam
  edge dropped and the two loops joined: the surface is exactly the two it was, with no edge drawn
  along the round's top. Named for both edges' blends, as G1's one face is.

The circle comes from the faces' tangent planes where the section meets them (Cramer's rule on
n·X + r = h for each), refined by Newton on each face's signed distance (exact for flat sides and
for a curved rib's, whose sides are straight in the section); it is converged to the last digits,
since strips a hair apart leave a sliver of the center face the booleans cannot take.
`full_round_tests` checks a curved rib by Pappus, a revolved rib with unlike tapers (cones), a
curved rib extruded with draft (leaning ends), a rib with bulging sides and a straight rib (its
volume), at G1, G2 and G3 (bulging: G1), each closed with the round one face.

**Variable radius** (`variable_fillet.lucb`, `radius_law.lucb`): Fusion's radii at points along a
tangent chain, positions 0 to 1 as fractions of the chain's length (Fusion's API: "the percentage
along the curve"). On an open chain round a flat face that length is **the balls' path's** (the
sections' centers), not the edge's: Fusion's strips run on it (probe 8: a plate's line, arc and line
of 10, 15.708 and 10 ran 10.0416, 11.024 and 10.0416 at 2 to 4, the law straight in it), so round a
convex arc, where the path is shorter, the radius changes more slowly than along the edge; the law
is tabled so (`cap_blend.lucb`'s `spine_law`, a few passes, the centers moving with the radius), and
Fusion's volumes follow to 0.001 (`fusion_plate_tests`). A straight edge's path is straight, so its
law is the edge's. The chain starts at an open end, the first named edge's chain followed back, or
for a closed loop at the first named edge's start; a closed loop's start and end radii must agree.
G1, G2 and G3, under a Tangency weight (each section the weight's at its radius: Tangency weight
below). **The law between points** (`radius_law.lucb`) is Fusion's by default (luced-3d's Smooth
transition off), read back from its faces in probes 5 to 7 of 2026-10-12 (15 laws on a box's edge,
`luce_work/cad/fusion/FINDINGS.md`): a cubic on each piece whose slopes are
- straight between two radii, and where the points line up;
- else a **natural cubic spline** through the radii (no curvature at the open ends), which is
  Fusion's to 0.003 in radius wherever that spline keeps the radii's shape (`fusion_law_tests`:
  Fusion's volumes within 0.12);
- held flat at an interior extreme and beside an equal radius (equal radii hold between them), the
  runs between such points natural splines held flat at those ends (Fusion's to 0.03 on alternating
  extremes);
- limited as Fritsch and Carlson's monotone cubic where a run's spline would still turn back.

Where the spline would not keep the shape Fusion's own pieces are **not cubics and not decoded**: a
piece from an open end to a flat run is one universal shape in Fusion (the same on four laws, its
slope 1.72 times the chord at the open end, flatter than a cubic into the run), and a steep piece
beside a shallow one runs straight at its far end. There ours differs by up to 0.4 in radius
(volumes 0.3 to 5.5 on the 20 box's edge: L5, L7, L10, L12, L14 of probe 7). With **smooth** on,
each radius is held a moment at its point (t²(3 − 2t) between: Inventor's Smooth radius transition;
Fusion has no such option). **A closed loop's law** is Fusion's: through its mid radii only, round
the loop (Fusion leaves its start and end radii unused: one mid radius holds all round, Fusion's
cylinders and tori; `fusion_law_tests`).

**Where Fusion rolls a ball** (probe 5): at G1 under a Tangency weight of exactly 1 (0.999 and
1.001 scale the section), along any law; every other weight and G2 scale the section. Along one
straight edge between flat faces the ball rolls here too: a straight law makes the cone Fusion makes
(below), any other law the balls' envelope (`rolled_ball.lucb`, below), within 0.12 of Fusion's
volumes where the law is Fusion's. Round closed loops it rolls too (`cap_rolled.lucb` round a flat
face, `curved_rolled.lucb` between curved faces or a flat face and a sphere): each section the balls'
envelope there, the balls searched round the whole loop (between curved faces sampled once and read
by Catmull-Rom cubics), fitted as a straight edge's are. Checked against Fusion with Fusion's own
law read back from its G2 faces (a rounded box's top loop, a dome's rim), so only the rolling is
compared: within 0.03 (`fusion_roll_tests`). Open chains' run-outs still scale the section (Fusion
fails at those ends). **Where the envelope folds** (steep radii: probe 7's L11; radii 2, x, 2 along a
20 edge from x = 12) Fusion scales the section along the whole edge instead (its face ends flush,
its knots inside the edge), and so does ours (`rolled_folds`: across the fillet each characteristic
circle's arc must move on along the edge), switching where Fusion does (x = 10 rolls, 12 scales).
- **Chains** are built as a constant fillet's are, the radius asked of the law at each section:
  loops round a flat face and their run-outs (`cap_blend.lucb`: a box's top past rounded uprights,
  an edge ending where an earlier fillet's edge goes on), loops between two curved faces
  (`curved_blend.lucb`: a drilled ball's rim). Their rows are fitted along the chain within the
  tolerance; the exact line and arc rows of a constant radius are not used.
- **One straight edge between flat faces** is exact: in each section every control point is the
  edge's point plus r(s) times a fixed vector (G1: the rational quadratic arc whose middle control
  point is the edge's own point, weight sin(θ/2), contacts r cot(θ/2) along each face; G2, G3: the
  fillet's B-spline on those contacts), so with r(s) a piecewise cubic the face is a B-spline, cubic
  along with a knot thrice at each point.
- **The rolled ball** (G1 at weight 1, a straight law: `rolling_weight`): the balls' envelope is a
  cone, its apex on the edge where the radius would be 0, its axis the centers' line, its half angle
  sin α = m / |c'| for the radius' slope m along the edge (c' = (1, m / sin(θ/2)) along and across).
  Its section square to the edge is the conic tangent to both faces at the same contacts (each ball
  touches a face at the foot of its center), its middle control point the edge's own point, and one
  weight w all along (each section is the first scaled about the apex): at the bisector the cone lies
  d / r = (1 / sin(θ/2) − √(1 + m² / sin²(θ/2) − m²)) / (1 − m²) from the edge, so
  w = r cos²(θ/2) / (sin(θ/2) d) − 1 (sin(θ/2), the circle's, when m = 0). Fusion 2705's face there
  is that cone (its apex, axis and half angle 0.19366 read back on a 20 box's edge, 2 to 6), and its
  volumes on edges of 90°, 120° and 60° are ours (`fusion_probe_tests`). Each row, a ruling of the
  cone, is rational, weighted inversely to the radius at its ends, so rows stopped at a slanted end at
  different radii still meet in a section of the cone.
- **The rolled ball along any other law** (`rolled_ball.lucb`): the envelope of the balls in planes
  square to the edge, so its ends stop in square end faces as Fusion's trimmed face does. In each
  section the envelope's points along rays from the edge between the faces (where a ray first
  enters the balls about the section, the least of its distances to them over their places along
  the edge: the ball touching the envelope there), the contacts exact (each ball touches a face at
  the foot of its center); the section is a degree 5 Bezier through the contacts, its second and
  fifth points along the faces (G1), the inner points fitted by least squares with each sample's
  parameter moved to its foot: within 1e-5 of the envelope at radii of a few millimeters. The
  sections' points are fitted along the edge (`row_fit.lucb`, knots four times at the law's points,
  where its cubics meet C1). The law is carried on past the edge's ends. At a slanted end the
  sections lie in planes turning from the end face's to the other end's (`rolled_section`): the
  first is the envelope's trace in the end face, as Fusion's face trimmed by it. Its tool is the region between the edge and that face,
  taken away (added for an inside edge); its net keeps to one side of each face's plane. Ends:
  - square to the edge: flush;
  - any other flat end face, leaning either way (a slanted end, a wall a convex edge runs into, an
    inside edge's end): Fusion 2705 sweeps the section on past the end (the law carried on straight
    from its end piece) and the end face trims it (probes 8 and 10). Where the radius holds (a
    cylinder) or runs straight (the rolled ball's cone) each row of the net stops where it meets the
    end face's plane, exact (rows of a cylinder or a cone stopped anywhere stay on it). Any other law
    (`tilted_sweep.lucb`): the sections lie in planes turning from the end face's to the other end's,
    each the sweep's own curve there (for each point of the unit section, the place along the edge
    where the scaled point lies in the plane), fitted with the section's degree and knots and its
    points next to the contacts held on the faces' traces (G1 to G3 kept), and fitted along: the
    trimmed sweep within the tolerance. (Stopping each row in the plane would mix rows from different
    places along the edge in one section, off the sweep.) Fusion's faces sampled from its NURBS lie
    within 2e-3 of ours (`fusion_roll_tests`); the mesh volumes of such sheared strips converge too
    slowly to compare. Reaching past and letting the end face trim the strip with the booleans was
    tried first: a G2 strip's marched crossing with a slanted face could not be split.

`fillet_option_tests` checks the volume taken along a box's edge against the cone's corner area × ∫ r² ds
(straight) and (1 − π/4) × ∫ r² ds (smooth);
`variable_chain_tests` checks G2 and G3 against their sections' areas, a slanted end exactly in its
face's plane (by the classifier; the sheared strip's mesh volume is within 0.3% of the constant
fillet's), an edge running into a leaning wall against the flush tool less the wedge
behind the wall's plane, an inside edge with slanted ends, a rounded box's top loop (G1, G2) and a
line of it running out, a drilled ball's rim, and two chains refused. Corners where a variable
fillet meets other fillets in one feature (three edges at a vertex) are not built: Fusion's
variable sets are one tangent chain each too.

**Chamfer types and distances** (`chamfers.lucb`). Fusion's Chamfer has Equal distance, Two
distances and Distance and angle. Each distance is measured from the original edge, in the plane
square to the edge, straight to where the chamfer meets the face: along a flat face that is the
distance along it, on a curved face a chord (Parasolid calls this an apex range). A dome of radius 10
chamfered 3 at its rim meets the sphere on the circle of 9.55, 2.96606 from the rim's plane, 3.000
straight from the rim (along the arc it would be 3.011), and the plane on the circle of 7.
- **Which face takes the first distance**: the one more of the chosen edges lie on (a loop's own
  face); on a tie a curved face over a flat one (Fusion's on a dome's sphere and a D-shaft's
  cylinder), then by name; Flip swaps.
- **Distance and angle** leaves the first face at the angle from that face's own tangent at its
  contact, in the plane square to the edge, and runs on until it meets the other face (`angle_leg`;
  `angled_point` between curved faces). Between flat faces φ apart the second leg is
  d·sin θ / sin(φ + θ) (d·tan θ when square). On a curved first face the tangent is the contact's,
  not the edge's: a dome's rim chamfered 3 on the sphere at 30° meets the plane on the circle of
  8.879, the cone's half angle 12.75°, as Fusion's (the edge's tangent would give 1930.76, not
  Fusion's 1981.44).
- **Verified against Fusion 2705** (probes 5 and 6 of 2026-10-12, `fusion_chamfer_tests`): the
  domes cut at 0, 4 and −4 and a D-shaft's flat edge, each type and way: Fusion's contacts and
  volumes (its volume is the dome less the triangle and the sphere's segment beyond the chord
  turned about the axis, to 1e-4).

All three take the same paths as Equal distance: loops round a flat face (`cap_blend.lucb`, its legs
per corner, so a drafted wall's second leg follows the draft), loops between curved faces or a flat
face and a sphere (`curved_blend.lucb`), and the booleans' tools (straight edges, rims, a plane on a
cylinder, inside edges).

**A plane meeting a cylinder along its length** (a D-shaft's flat, a slot's side) is blended by a
cylinder parallel to the axis. A fillet's center is where the plane moved by the radius meets the
cylinder moved by it: a radius of R − r or R + r, by which sides the material and the corner are on.
Its section closes back along the cylinder's own circle, not a chord, so it takes no material the
fillet keeps. Under a Tangency weight G1 is Fusion's conic, and G2 Fusion's section (below), each on
the corner where the plane meets the cylinder's tangent at the contact, G2's inner point there moved
so its curvature is the cylinder's; G3 matches the cylinder's curvature and its change (below).
**Verified against Fusion 2705** (`fusion_probe_tests`): G1 and G2 at weights 0.5, 1 and 2 take
Fusion's volumes; G3 (not in Fusion) is measured G3 at both contacts. Each of these sections touches
the cylinder only tangentially, so the booleans meet the swept section with the cylinder along it in
closed form (`splines.lucb`, `along_sweep`: lines along the sweep through the profile's points on
the cylinder, its touching ends found exactly), as they meet a plane along it. A G3 section weighted
heavily (2 here) would cross the cylinder between its contacts (its inner points lie along the
cylinder's tangent): refused, naming the weight. A chamfer runs `size` along the plane and reaches
the circle `size` straight from the edge (a chord, as every chamfer's distance on a curved face:
`chamfers.lucb`; Fusion's contacts and volumes for each type, `fusion_chamfer_tests`).

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
- **G2 is Fusion's own** (**verified**: Fusion 2705's blend faces read back as B-splines by the probe
  of 2026-10-12, at corner angles 30° to 150° and nine weights, match to 1e-14). Fusion's is degree
  4, two spans (knots 0, 1/2, 1), six control points: each contact, then two along its leg, a and b
  from the contact, a = r φ w / 8 and b = r sin φ / 4 + a (5 − 2 cos φ) / 3, for the turn φ between
  the legs (π less the faces' angle), the radius r (each leg r tan(φ/2)) and the Tangency weight w.
  So w scales the speed at the contacts, which is the circle's arc length r φ at 1. Ours is that
  curve raised to degree 5 (eight points, the middle knot twice, still C3): the same curve, so the
  degree 5 rule and Fusion parity agree. On a curved face (a dome's sphere, a cylinder along a
  D-shaft) the inner point moves off the leg by 8 a² κ / 3 toward the face's center of curvature in
  the section, so the curvature matches the face's at the contact, as Fusion's does (its dome and
  D-shaft sections show it).
- **G3** (not in Fusion): three spans (knots at the thirds), eight control points. The first four on
  each side lie along that face, at 0, 0.1, 0.2 and 0.6. On a curved face the third and fourth move
  off the leg toward the face's center of curvature, so the section's curvature and its change at the
  contact are the face's (`g3_leg`): with the knots at the thirds the start's first three
  derivatives are 15 D0, 90 D1 − 180 D0 and 270 D2 − 1215 D1 + 1620 D0 (D_i the legs between the
  points), so for the first leg a1 the curvature 0.4 h2 / a1² gives the third point's offset
  h2 = 2.5 κ a1², and the change of curvature the fourth's (the face's change along the section,
  `curvature_turn`; nil on a sphere or a cylinder's circle). A dome's rim and a D-shaft's flat edge at
  G3 measure G3 at both contacts (`fusion_probe_tests`, `CadMeasure.check`).

The curvature is then zero where the curve meets a flat face, and for G3 so is its rate of change.
For a square corner with contacts a unit from it, the middles are 0.366 (G2) and 0.360 (G3) from the
corner, against the circle's 0.414; the corner regions 0.16824 and 0.16411 (the circle's 1 − π/4).
Our G2 before Fusion's was one span with its points at 0.3 and 0.6 of each leg (middle 0.376).
A single degree 5 curve can't make a good G3: it would need its two middle control points on the
corner itself, and its curvature would peak at 3 to 6.5 times the circle's.

The blend face is that curve swept along the edge, a B-spline surface. `splines.lucb` lets booleans
meet a plane with such a swept profile in closed form:
- **A plane crossing the sweep** cuts it in an affine copy of the profile.
- **A plane along the sweep** cuts it in lines through the profile's points on the plane, and so
  does **a cylinder along it** (its axis the sweep's way: a D-shaft's cylinder, which a weighted G1,
  G2 or G3 section touches only tangentially at its end).

At a G2 contact, curves stay within tolerance of the face they touch over a stretch, so two rules
keep that stretch from producing false crossings:
- **Crossings snap:** where a curve runs along an edge up to either one's end, the crossing is that
  end.
- **Splitting looks further:** where two edges leave a vertex along one tangent, the split compares
  them further along to see which turns first, at equal distances from the vertex (not equal
  fractions of edges whose lengths may differ a thousandfold: a G2 section against the long arc of a
  cylinder's end it touches with the same curvature).

On a rim, the G2 or G3 profile is turned about the rim's axis. A spline turns as a rational surface
of revolution: two rational quadratic arcs around, each at most a quarter turn. A full turn is two
halves, so each face's chart opens without a seam. Booleans meet such a surface with a plane across
its axis, and with a cylinder or cone about it, in circles: where the profile's height, or radius,
is theirs. Revolve turns sketch splines the same way.

Two G2 or G3 fillets meeting where a third edge stays sharp meet in a miter. Their swept profiles
meet in the plane bisecting their sweeps, through where their contact lines cross, as that plane's
affine copy of either profile. It is checked to lie on the other.

**Tangency weight** (Fusion's, 0.1 to 2, 1 the default; its API: "the influence of the
continuity, G1 or G2, on the fillet"). It changes a section's inside and keeps its contacts and the
lines its end control points lie on, so the continuity at the faces stays. **Verified against Fusion
2705** (probe of 2026-10-12: its faces' B-spline data, sections and volumes):
- **G1:** the circle's rational arc has its middle weight sin(θ/2) times w (`arc_section`), θ the
  faces' angle: a conic whose end tangents meet at the corner, a hyperbola above 1, an ellipse below.
  Fusion's is exactly this (its middle weight cos(φ/2) w for the turn φ = π − θ; 0.3536 and 1.4142 at
  a square corner): `fusion_section_contract` matches its sections at 0.5 and 2, `fusion_probe_tests`
  its volumes at 0.5, 1, 2 on edges of 90°, 120° and 60°. A conic has no ball tangent to three of
  them, so a weighted G1 fillet takes the booleans' tools (straight edges and rims) with rational
  spline sections and setback corners; loops round a flat face carry the conic directly (their strips
  B-splines, not cylinders or tori), as do loops between curved faces (a dome's rim).
- **G2:** Fusion's curve above, w scaling its speed at the contacts: sections at 0.1, 0.5, 1, 2 and
  volumes at 0.5, 1, 2 on 90°, 120°, 60° edges match Fusion's.
- **G3** (not in Fusion): each leg's fraction f toward the corner becomes 1 − (1 − f)^w, its first
  points' speed so nearly w times, as Fusion's G2's, its inner ones never past the corner.
Every path takes it: the direct strips, setback corners, loops round a flat face, loops between
curved faces (a dome's rim: Fusion's volumes at G1 and G2, 0.5 to 2), a plane meeting a cylinder
along it (Fusion's volumes at G1 and G2, 0.5 to 2), and variable radius (`variable_filleted`):
along a box's edge, 2 to 6 straight, G2 at 0.5, 1, 2 and G1 at 0.5, 0.9, 0.99, 0.999, 1.001,
1.01, 1.1 and 2 scale the section with the radius, and G1 at exactly 1 rolls a ball (Variable
radius below), all Fusion's volumes. Full round takes no weight (Fusion's Full Round has none). `tangency_tests` checks that a box edge keeps more of its corner as the weight
rises at G1, G2 and G3, that 1 is the plain fillet, corners and a cap loop.

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

**G1 setback corners** (Fusion's Corner type Setback for tangent fillets): the same corner, its
A curves the fillets' circular arcs raised to degree 5 (rational: `arc_section`, the quadratic's
weights elevated in homogeneous space), and the edges' tools then take that same rational arc as
their section (`g1_spline`) so their ends are the curves the patch meets. The patch is fitted in
homogeneous space (`patch_fit.lucb`): the weights first, as a field of their own under the same
seam and fairness rows, fixed on the sides and along the sweeps; then the weighted points, a
plane's rule holding n·X = h·w at each point's weight. The seams' G1 rows hold in every
homogeneous coordinate, so they hold for the rational quads. One row beyond each side (G1). The
direct build rounds G1 corners with balls, so G1 setbacks go to the booleans.

**Too large.** Before any tool, each edge's blend must fit (the way into each face is found a
quarter of the reach in, clear of where a curved boundary's chart, its chords, strays off the
edge: a hair in, the chords of a large arc said the face was on the other side):
- **Its faces:** samples along the edge step into each face by the contact distance, and must stay
  on it.
- **Its rounded corners:** must leave the edge some length.
- **The whole-body rounding:** must leave each shrunk edge some length.

Otherwise the error gives the size the blend must stay under, for example "a fillet radius of 1.5
is too large for this edge's faces: it must be less than 1.0". One case runs over instead: a fillet
on a straight edge between flat faces that runs off one face over a straight convex edge parallel to
it, into a flat face that turns the same way (a plate thinner than the radius). That face trims the
fillet (the study's rail classification, its same-convexity case): the edge's tool reaches past the
face, outside the body, and the booleans cut it there, so the face it runs off goes and the round
meets the next face at an angle. Rolling on onto a tangent next face (Parasolid's overflow) and
covering a cliff (a concave edge beyond) are not done: refused as above.

**Tangent chains.** Like Fusion's Tangent Chain, a referenced edge brings the edges that continue
it tangentially with the same convexity, such as a rounded slot's rim or the edges around an
earlier fillet. Pass `chain = false` to blend only the edges named.

Arcs of rims (an arc between a flat face and a cylinder about it) are blended by a corner profile
turned part way along them, a part torus or cone. Where chained edges meet, each tool ends square
at the joint, so neighboring tools share their end planes. A free end of an arc reaches past, as a
line's does, when the corner beyond is empty.

### Direct fillets (`local_blend.lucb`, `local_corners.lucb`)

Fillets of straight edges between flat faces are built directly, without booleans (FILLET-STUDY.md
§3.2). The booleans above are what's left for anything else. The result's B-rep is made from the
body's:
- **Each chosen edge** becomes a strip between its contact lines on its two faces: a cylinder for
  G1, its degree 5 section swept straight for G2 and G3.
- **Each face touched** keeps its plane. Its loops are rebuilt: a chosen edge becomes its contact
  line, a sharp edge at a rounded corner is shortened, and at a corner the face gains the curves the
  corner leaves on it.
- **Each corner** (a vertex where a chosen edge ends, with three edges and three flat faces) ends
  its strips there and adds its own faces:
  - one chosen edge: its strip runs to the third face's plane and is cut by it, and that face gains
    the cut;
  - two: a miter, both strips cut by the plane bisecting their edges;
  - three, G1: a ball, the strips stopping square to their edges through its center;
  - three, G2 or G3: a setback corner, its six quads.

Every point and curve is made once and shared by the faces that meet there, so the result is
closed by construction. It is still checked: each edge used twice, once each way, with the new
faces turned to agree with the body's.

What it declines falls to the booleans:
- curved faces at a corner, more than three edges at a vertex, chamfers and rims;
- an inside fillet meeting an outside one;
- a miter that isn't symmetric about its plane. The two strips' sections are one curve only when
  the sharp edge lies in the plane.

A contract (`src/tests/direct_fillet_contract.lucb`) requires nine cases to be built directly at
G1, G2 and G3:
- an edge alone, a ring of miters, a corner of three, every edge;
- an L's inside edge, a pocket's floor alone and with its walls;
- a slanted box's oblique ends and corner.

It also builds them by booleans and requires the same faces and volume. That runs at G1 for every
case, at G2 for all but the two heaviest, and at G3 for four of them. `fillet_tests` checks the
heaviest ones' volumes against exact values instead.

The cost is the touched faces', with no intersection, splitting or classification. Every edge of a
box at G2 takes 44 ms, against 1 s by booleans. A corner patch's thin-plate energy is one quadratic
form, the same for every quad, so it is made once per fit.

**Kept between radii** (`fillet_plan.lucb`). `CadModeling.fillet_plan` works out once what does
not depend on the radius:
- the body's modeling shape;
- the edges its references name, with their chains;
- each setback corner's patch, fitted at radius 1.

A corner scales with the radius about its vertex: its setbacks, contacts and curves all do, and the
fit is made at unit size. So `FilletPlan.filleted(radius)` scales the patches instead of fitting
them. `made_for` says when the body (the same object), references, continuity, tag or chaining
changed. luced-3d's Fillet node keeps its plan on the compute worker, so dragging its radius only
rebuilds. On every edge of a box, a radius costs 1 ms at G2 and 1.5 ms at G3, against 33 ms and 55
ms afresh.

### Loops round a flat face (`cap_blend.lucb`, `cap_rows.lucb`)

A flat face's edges where it meets walls ruled away from it are blended directly too, as a whole
tangent loop or part of one. A wall may be:
- a plane at any angle to the face (a box's side, a tapered extrusion's);
- a cylinder or cone about the face's normal (a hole, a tapered extrusion's rounded corner);
- a surface swept straight along one way out of the face's plane: an extrusion's B-spline side, an
  earlier fillet's strip, a cylinder along a slanted upright (a tapered box's filleted corner).

The top of a box whose uprights were filleted is such a loop: four lines and the uprights' G1 arcs
or G2/G3 sections, tangent where they meet; so is a tapered box's, and a hole's rim. Every vertex of
a picked edge is a joint (two picked edges meeting tangentially, and one seam between their walls
along their ruling) or an open end (below). Anything else is left to the booleans.

The blend is a 2D section (a circle's arc, the G2/G3 B-spline, or a chamfer's line) carried along the
edge in the frame of the way into the face, `u`, and the wall's ruling `S`, scaled so that across the
edge it is one unit long. OpenCASCADE and Parasolid build a ball rolling along a curve the same way.
- **The corner** has cosine λ = D · u (D the wall's way square to the edge): 0 for a square wall,
  -sin(taper) at a tapered extrusion's top. Contacts are r cot(θ/2) = r (1 + λ)/μ from the corner (μ
  = √(1 − λ²)), a chamfer's its size along each face. A G1 section is the rational arc with middle
  weight sin(θ/2); the chain's corner sets it (a slanted sweep's corner changes along it, its
  sections then near-circles meeting the faces tangentially).
- **The ruling** is a plane's from one seam's way to the other's (the plane between two slanted
  fillet strips), a cylinder's or cone's line through the point, or a sweep's way.
- **Lines and arcs whose corner stays the same** are exact: rows are lines or concentric arcs;
  square ones and drafted ones without skew make a cylinder or plane along a line, a torus or cone
  about an arc.
- **Everything else is fitted.** Each row of the section's control points follows the edge: its ends
  and end tangents exact, its interior a least-squares fit on the edge's parameter (a B-spline's own
  degree and knots, else degree 5 over two spans). Spans that stray beyond the shape's tolerance are
  halved, and the rows are fitted again. The rows on the face and on the wall are those faces' new
  edges, so they are shared exactly; the wall's row lies on the wall because it runs along its
  ruling.

The face's loop moves onto the contacts. Each wall's edge moves along the ruling to its contact. Each
seam is shortened, and each joint gets one section edge, which both of its strips share. The result
is closed by construction, and checked.

**Open ends: run-outs.** Without Tangent chain, a picked edge may stop where the loop goes on
tangentially into an edge that is not picked (a fillet ending where a tangent loop goes on). Fusion 360
does not cap the blend there: the rolling ball runs on past the end and the blend runs out into the
faces it meets, trimmed by them (`cap_runout.lucb`). Here the section at the end is carried straight
on along the edge's way. Its contact with the face runs on as a straight line until it crosses the
edge going on, at the run-out point P; every other point of the section is carried until it leaves
the material through the wall going on (a rounded upright, which turns away), and those points make
the trim curve on that wall, from P down to the wall contact (a cubic through graded samples, refined
until it stays within half the tolerance of the wall). The edge going on is cut at P, and the seam
is shortened to the wall contact as at a joint. A chamfer's or G1 fillet's strip along a line (a
plane or a cylinder) simply runs on in its own surface; any other strip gets a run-out piece
(`<tag>/run[...]`) on the section swept straight on. On a box whose uprights are rounded, one top
line filleted alone takes the section along the line plus, past each end, the slices of the corner
farther than δ(s) = R − √(R² − s²) from the wall, which `cap_loop_tests` checks.

Where the wall going on does not turn away from the carried section (a flat side after a corner's
arc, a hole's wall curving into the material), nothing trims the blend, so it stops flat: the strip
ends in the section's plane, and a flat end face (`<tag>/end[...]`) fills the corner between the
section and the sharp edge, bounded by the section, the seam's upper part and a line on the face
back to the corner.

**Corner ends.** Where the loop turns a sharp outside corner instead (a plate's top edges ending at
its sides), the chain ends there against an end wall: a plane square to the blended edge, the
cap's edge going on straight along it. Fusion runs the strip flush into that wall: the section at the
end lies in the wall's plane, the wall loses its corner beyond the section (its loop now the cap's
edge cut at the contact, the section, and the seam shortened to the wall's contact), and no face is
added. **Checked against Fusion 2705** (probe 8 of 2026-10-12, `fusion_plate_tests`): a plate's
line, arc and line filleted with Tangent chain, ten faces, constant 3 at G1 (weights 1 and 0.5) and
G2 within 0.001 of Fusion's volumes, 2 to 4 along the balls' path at G1 0.5 and G2 too; at G1 weight
1 Fusion rolls a ball along the changing chain, ours scales the section (0.09 more). An end wall at
another angle, or curved, is not a corner end yet (the booleans then try). The seam is split at the wall's contact. The end face touches the face and the
wall in cusps, which the tessellator samples finely. Concave chains' open ends stop flat too. **Checked against Fusion 2705** (probe of
2026-10-12, `fusion_probe_tests`): a stadium plate's arc top edge filleted alone (r 3, Tangent chain
off, G1 and G2) fails in Fusion ("ASM_BL_END_TOO_CMPLX ... at the ends of the selected edges"). The
flat end here is a deliberate superset: it succeeds where Fusion fails, and the G1 half turn takes
exactly the section's Pappus volume.

**Reflections at a G3 corner.** Where a box's top loop is blended around an earlier G3 upright,
zebra stripes on the corner's strip converge to one point on its contact with the top. That is the
G2/G3 contact itself, not a defect: along the contact the strip's normal is the face's, so the
stripes' function is constant there and every stripe on the strip reaches the contact where the
face's own single reflection boundary crosses it (a G1 strip would show a jump instead). The mesh
there is a mapped grid whose sections fan toward the corner's center of curvature: its cells narrow
on the contact's side, where the contact runs at about half the edge's radius of curvature (0.26
against 0.51 for uprights of 0.6 and a loop of 0.25). A few boundary cells take an extra sample the
top face puts on their shared contact (pentagons, nine in 4086 cells). No pole: no point where the
patch's parameter lines meet.

**Too large** is said with the size that fits. A contact on the face must stay short of where the
edge's offset folds (the edge's least radius of curvature toward the face: 0.341 for G3 uprights of
0.4). Each seam must keep some length (half its length when both its ends are blended). A drafted
corner's contacts are farther than the radius; the sizes said allow for it.

### Loops between two curved faces (`curved_blend.lucb`, `row_fit.lucb`)

An edge between two curved faces (a hole's rim through a ball, a boss's foot on a pipe, two
fillets' strips meeting) is blended by a ball rolling between the two surfaces, in the plane square
to the edge at each point. Neither face may be flat (those go to the paths above), but for a
fillet's whole rim between a flat face and a sphere about it (a dome's edge).
- **The spine.** The ball's center is where both surfaces, offset by the radius, meet in that plane:
  Newton on their signed distances (a B-spline's along its normal at its nearest point), the plane's
  equation the third. Inside the material on a convex edge, in the air on a concave one. When it no
  longer touches both, the fillet is too large ("the ball no longer touches both").
- **The section.** The contacts are the center's nearest points on each face; their tangents
  across the edge meet at a corner. G1 is the circle's arc, rational, its middle weight sin(φ/2) for
  the corner's angle φ (times the Tangency weight); G2 and G3 are the fillet's B-spline on the same
  contacts and corner, G2's inner points moved to match each face's curvature across the edge (from
  how its normal turns along the section, a finite difference). A chamfer joins the points its size
  along each face (a chord), square to the edge.
- **The rows** are fitted along the edge (`row_fit.lucb`, shared with `cap_rows.lucb`): a
  B-spline edge's degree and knots, else degree 5 over four spans, spans halved until every row
  stays within the tolerance. A G1 strip's corner row is fitted homogeneous with its weight as a row
  of its own, then divided back: its weights vary along the edge.
- **Exact rows round a turned rim** (`curved_turned.lucb`): when the edge is a circle (or an arc of
  one) both faces are turned about (a plane square to the axis, a sphere, cylinder, cone or torus about
  it) and the radius holds, every section is the first turned. Its rows are then exact rational arcs
  (spans of at most a quarter turn), the strip a surface of revolution and its contact edges exact
  circles: a dome's rim filleted G1 is the torus, as Fusion's.

Whole loops only. Every vertex of a picked edge is a joint of two picked edges meeting tangentially
(traced edges' ends agree to a hair) or a closed edge's own. The faces' other edges there (a seam,
an edge to a third face) must run on square to the loop. They are shortened to the contact on their
face, and the section at the vertex lies along them, so that a periodic face's seam stays exactly
on its chart's. A straight one a hair off is redrawn from the contact. Each loop lies between one
pair of faces.

Checked: a ball drilled along its axis, both rims filleted G1 and chamfered, against Pappus in the
meridian plane; G2 and G3 take less; a boss's foot on a pipe (two cylinders meeting in a traced
loop of two edges, concave) gains less at G2 and G3 than at G1. **A dome's rim** (radius 10, cut at
its middle) filleted 2, **verified against Fusion 2705**: the torus between the plane's circle of
√60 = 7.74597 and the sphere's of 9.68246 at 2.5, three faces, volume 2020.56251 (ours, meshed and
extrapolated, within 0.001); G1 at weights 0.5 and 2 and G2 at 0.5, 1 and 2 to Fusion's volumes
within 0.01 (`fusion_probe_tests`); G3 closes too.

### Fillets meeting at corners (`corners.lucb`)

- **Two fillets** of one radius meeting where the third edge stays sharp meet in a miter. Both
  tools reach past the corner, and their cylinders cross in two ellipses, one in each plane bisecting
  the axes. `intersect.lucb` gives those in closed form.
- **Three fillets** meeting at a corner of three flat faces are rounded by a ball, as OpenCASCADE's
  `Rotule` does. The ball's center is where the faces' planes, moved in by the radius, meet. Each
  edge's tool stops at the plane through that center square to the edge, where its cylinder touches
  the ball in a great circle. A corner tool then takes what is left: the region beyond those three
  planes, reaching past the faces, minus the ball. That leaves the sphere patch.
- **Four or more fillets** meeting at a corner of as many flat faces, all its edges, G1: a ball too,
  where one touches every face there (its planes moved in by the radius meet in one point: a regular
  pyramid's apex). The corner tool is the convex cell beyond each edge's plane and past each face
  (`convex_cell`), minus the ball. Where no ball touches them all (a rectangular pyramid's apex) the
  fillets are refused, naming it; every edge of a convex polyhedron rounded at once (`rounded.lucb`,
  the solid shrunk and grown back by a ball) asks the same of each vertex, and so is refused there
  too rather than built with its shrunk corner guessed.
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
- **Anything else** is refused: four or more edges at a corner no ball fits, fillets meeting where the faces
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

### Local face operations (`face_moves.lucb`, `face_delete.lucb`)

Offset, Move and Replace Face are one operation, as Parasolid's local operations are: the chosen
faces take new surfaces and the body keeps its topology. Built directly, without booleans:
- **Vertices.** Each vertex on a chosen face is found again where the surfaces of its faces now
  meet: Gauss–Newton on their signed distances, each step the least move that zeroes them to first
  order, so with two surfaces (a circle's vertex) it slides along their curve. It starts from the
  old vertex projected onto the surfaces in turn, round after round: where two of them meet
  tangentially (a top replaced with its fillet) Gauss–Newton's system is nearly singular far from the
  corner and its steps can land anywhere, while projections close in on it steadily. A vertex whose
  faces all move rigidly moves with them.
- **Edges.** An edge on a chosen face gets a new carrier where its two faces' surfaces meet: the
  line through its new ends between two planes, else `intersect.lucb`'s curve through both ends. A
  seam follows its face (a line through its ends, a meridian's circle grown with it). An edge
  between two unchosen faces keeps its carrier with a new interval, so neighbors extend or shrink.
  An arc runs round its circle the way it did, judged from its old tangent. Two faces meeting
  tangentially and offset alike (a fillet's rim on a corner torus, in a shell) touch rather than
  cross, so no intersection finds their edge: it is the old edge moved along their shared normal,
  a line through its new ends or the circle through them and its moved middle.
- **Refusals.** An edge that would vanish or turn round, an arc that would turn over, a corner whose
  faces no longer meet in one point, a flat face whose loop would turn inside out: "the move changes
  the body's shape too much". A face tangent to a chosen one (a fillet's strip beside the face it
  rounds) would no longer meet it, so it must move too: Offset's tangent chain takes it along; a
  Move without it is refused.
- **B-spline faces.** A chosen B-spline face's offset is a B-spline fitted within the model's
  tolerance (`offset_surface.lucb`): the base's degree and breakpoints, each interior breakpoint once
  more (the offset is one order less smooth), interpolated at the knots' Greville points, every span
  halved until it holds at the spans' middles. A profile swept straight (a G2 or G3 blend's strip)
  stays one and is swept four offsets further both ways, so a neighbor moved out past its old end
  still meets it. A moved face and a B-spline neighbor meet along the booleans' curve: in closed form
  where they have one (a plane across a swept profile), else traced round the edge.

The result is checked closed. Not checked yet: a moved face passing through a far part of the
body (a self-intersection), which needs the booleans' face pairs.

**Delete with healing** takes the faces away and closes the gap with the faces around:
- **Holes and bosses.** Deleted faces that leave whole inner loops on the faces around (a hole's
  rim on a plate's top, a boss's footprint) take those loops with them.
- **A strip** (one face with four neighbors: a fillet's or chamfer's face between two faces, ending
  on two more). Its two long neighbors meet along a new edge where their surfaces cross, from their
  corner with one end face to their corner with the other (each corner the three surfaces' point);
  the end faces' loops shrink to those corners. Either opposite pair may be the long one; the one
  whose surfaces cross into a proper edge is taken (parallel end faces never meet).
- **A corner** (one face with three neighbors): the three surfaces' point.

The edges running into the gap keep their carriers, lengthened to the new corners.

**Several faces at once** (`face_heal.lucb`), where the local healing above refuses (a chain of
fillets and its corners, strips apart, a face with more than four neighbors), are closed by caps when
every face around them is flat. Each deleted face's cap is the box round its boundary cut by every
neighbor's half-space on the side the face lies: the space between the face and the sharp edge or
corner the extended neighbors make. A face on every neighbor's material side (a round, a chamfer) has
its cap joined to the body; one outside some (a fillet in an inside corner) has it taken away. The
caps' faces on a neighbor's plane merge into it, and their own names are dropped. A deleted face left
over, or a cap face standing on its own (a cap reaching past the body), is refused, as are curved
faces round the gap. A box's top, say, has four parallel-sided neighbors that cannot close over it.

**A band beside curved faces** (`face_band.lucb`), where the caps refuse because a curved face
borders the gap: the deleted faces leave one face with a whole loop on the gap (a top) and a ring of
faces round it, each with one run of gap edges (the sides and the rounded uprights, G1 cylinders or
G2/G3 B-spline strips). Each ring face meets the top along a new edge where their surfaces cross: a
line between planes, an analytic curve (a cylinder's circle), else traced on both surfaces and fitted
by a cubic that stays on both (`march.lucb`'s steps). Each edge between two ring faces running into the
gap keeps its carrier, lengthened by Newton along it until it reaches the top's surface. A rounded or
chamfered top chain beside rounded uprights heals back to the rounded box exactly. A ring of one face
whose run closes on itself (a pin's or a hole's wall, its run a whole circle from its seam) meets the
open face along one closed curve: the seam runs on to the face's surface and the new edge goes once
round the circle or ellipse the two share, the way the run went. An inside fillet or a chamfer at a
boss's foot, or a hole's rounded rim, deleted, heals to the sharp body.

**Groups apart** (`deleted_faces`): where none of the above takes the faces together, faces in groups
not joined by an edge (a foot fillet and a rounded upright elsewhere) are healed group by group, each
by whichever healing takes it; any group refused refuses the whole.

**Replace Face onto several targets** (`face_replace.lucb`). Targets on one surface (coplanar faces,
a split face's pieces) take the local operation. So do targets on several surfaces that match the
replaced faces one for one, as Fusion replaces a chain onto a like chain (a top, its fillet and the
side it rounds onto, onto another body's): each face takes the target of its surface kind facing most
nearly its way (within 25°, the nearer on a tie), every target taken; the target body's solid plays no
part, so it may overlap the body. Other targets on several surfaces must be faces of a solid:
the replaced faces are pushed far past them (Offset Face), the target solid is taken away where it lies
beyond them (its faces facing the body) or kept where it lies behind, and the piece holding no replaced
face and most of the body's other faces is kept. A quilt that does not reach across the faces, or a
target solid cutting into the body elsewhere, is refused.

Targets on an **open surface body** (`face_quilt.lucb`) are closed into a solid first: the quilt swept
far along the way the replaced faces face. Its faces are the quilt's (turned to face back toward the
body), the quilt moved along the sweep, and a wall along each open edge: a plane for a line or a flat
curve the sweep lies in, a cylinder for a circle about the sweep, else a ruled B-spline (a circle's or
ellipse's exact rational net first). The walls meet at lines from the quilt's corners. The solid then
goes through the booleans as a solid's faces do.

**Split Face** (`face_split.lucb`) runs the booleans' first steps: the surface, as the faces of
its solid lying on it (a plane's large square, a curved surface's long wall), meets only the chosen
faces; each is split along the new edges, whose ends cut the edges of the faces beside it, and every
piece is kept as it is, none merged back. A surface that crosses none of the faces, or lies on one of
them, is refused. With Fusion's **Extend splitting tool** off the tool face itself is the sheet (a
B-spline face too, which cannot be extended): it splits only where it reaches, and a face it meets
must be cut right across. A cut ending inside a face is refused, naming the option.

### Shell (`shell.lucb`)

Two offset bodies are made with the local face operations above. The outer one has every face moved
out by the outside thickness; the removed faces stay where they are. The inner one has every face
moved in by the inside thickness, while the removed faces move out past the outer body, so the inner
body pokes through where the hollow opens. The shell is the outer body less the inner one. Without
removed faces the inner body lies wholly inside: a closed void, a second shell.

Faces offset as Offset Face's do, so a box's rounded uprights of 0.3 give a hollow rounded by 0.2
for a wall of 0.1. A wall thicker than a rounded face's radius, or one that would change the body's
topology, is refused: "the shell's thickness is too large for this body", with the inner wall's own
reason after it. The inner wall's faces are named `<tag>/` and the face's own name. B-spline faces are offset as
Offset Face's are: a box rounded G2 shells closed or opened at its top.

### Every edge rounded whole (`rounded.lucb`)

When no edges are picked, filleting every edge of a convex polyhedron is built whole, as a
Minkowski sum: the solid shrunk
by the radius, then grown back by a ball. Each face keeps its plane with its corners moved in, each
edge becomes a cylinder band about the shrunk edge, and each corner becomes a sphere patch bounded
by its edges' arcs. A box comes out with 26 faces. Each sphere patch's pole sits in its middle, which keeps its
tessellation even.

## Measuring (`measure/`)

`luce_cad.measure` measures a model without changing it: how continuous its faces are across
their edges, curvature combs on its edges, and how far it deviates from another model. It is
Stage 0 of luced-3d's surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md`),
after Alias's Evaluate tools. luced-3d's Check node draws what it returns.

| `CadMeasure.` | What it does |
|---|---|
| `check(model, references, level, stations, distance, angle, curvature, rate, combs, samples, quills, scale, sharp)` | The edges `references` names (empty: every edge between two faces) checked for continuity `level` (0 to 3) at `stations` per edge. Returns a `CadCheck`: per edge (`edge(i)`, an `EdgeCheck`) its faces, its worst gap, angle and curvature and rate differences, and the highest order every station meets; per station (`station(i)`) the values behind them; the comb samples (`comb(i)`); `passed()`, `summary()`; how many creases were left out (`creases_left_out()`; without `sharp`, edges whose faces meet at more than 10° all along them are not checked, though they keep their combs); the lines to draw by group (`line_count`, `line_point`; Base readers `check_lines`); and a label an edge (`label_count`, `label(i)`). |
| `with_curve_combs(check, set, samples, scale)`, `curve_check(set, samples, scale)` | Curvature combs of a set's shown curves (free and on faces, `samples` a span) added to a check, or a check of the curves alone (Alias's Curve Curvature on curves). |
| `deviation(model, reference, acceptable, ramp, segments)` | `model`'s tessellation with point attributes `deviation` (its signed distance from `reference`'s surfaces) and `Cd` (a color ramp), and its largest, smallest, mean and RMS deviation, and the share within `acceptable`. |

**Jets** (`geometry/jets.lucb`). Continuity to G3 needs third derivatives. `basis_jet` extends
The NURBS Book's A2.3 to the third order, and takes the limit from below (`left`) at an inner knot,
where a B-spline's derivatives jump. `spline_jet` and `surface_jet` apply the quotient rule to
rational curves and surfaces (A4.2 and A4.4) up to the third order; lines, circles, ellipses,
planes, cylinders, cones, spheres and tori take their closed forms. `basis_derivatives` is the
second-order view of `basis_jet`.

**Continuity** (`measure/continuity.lucb`). At `stations` stations along an edge (the middles of
equal parameter steps, so no station sits on a vertex), the station's point is dropped onto both
faces' supports. Each face is then cut by the plane square to the edge, which is how Alias checks
"relative to the boundary, square to it". That plane cuts each face in its normal section, a curve
leaving the edge in the direction across it, `d = t × n₁`, where `t` is the edge's direction as
face 1's loop runs it. Across the edge:

- **G0** is the gap between the two surfaces' points.
- **G1** is the angle between the oriented normals.
- **G2** is the difference of the two sections' curvatures, relative to the larger, in percent.
- **G3** is the difference of their curvatures' rates of change by arc length, relative in the same way.

The section is `S(w(s))` with `|S'| = 1` in that plane. So `w'` solves the first fundamental
form (`J w' = d`). The tangential part of `S'' = J w'' + H(w', w')` vanishes, which gives `w''`,
and the curvature is `H(w', w') · n`. The rate is the normal part of
`S''' = J w''' + 3 H(w', w'') + T(w', w', w')`, which does not need `w'''`.

Each face's derivatives come from its own side of any knot line: face 1 lies behind `d` and face 2
ahead of it, so a B-spline is evaluated from below in a parameter where moving into the face
lowers it.

Along the edge nothing more is needed. The edge is shared, so once the normals agree along it, the
curvatures in its own direction and the twist agree too, and only the cross terms can break.
Curvatures under a thousandth of `1/size` (and rates under a thousandth of `1/size²`) are flat:
two flat sides agree, whatever their noise. Seams and open boundaries have no second face and are
not checked.

**What it draws** (`measure/check.lucb`):

- **Edges by result.** Each checked edge is drawn in group 0 when it meets the level, 1 when it is
  one order short (Alias's yellow), and 2 when it is further short (red).
- **Labels** (`label_count`, `label(i)`, a `CheckLabel`; Base readers `check_labels`). One an
  edge, at its middle station: its line group, the order it reached (-1 where even G0 fails), and
  `value`, its worst measure of `order`, the first order it misses or the order asked for when it
  passed. luced-3d draws them as Alias's on-model labels ("G2  κ 0.00%").
- **Curvature combs** (group 3, `measure/combs.lucb`). Quills at `samples` along each curved edge
  point away from the center of curvature, each `κ × scale` long, with an outline through their
  tips. A circle's comb is a ring of equal quills.
- **Cross quills** (group 4). Each side's curvature across the edge is a quill along the normal at
  every station, just behind and just ahead of the edge. Equal quills mean G2; a jump shows as two
  lengths.

At scale 1, the longest quill is a tenth of the model's size. When every curvature is under the
flat floor, nothing is drawn: a scale fitted to rounding noise would draw it as quills.

**Deviation** (`measure/deviation.lucb`). Each point of the measured model's tessellation (its
edges at most an eightieth of its size: the display tessellation leaves ruled faces without inner
points) is matched to the nearest face of the reference's tessellation (geocore's triangle BVH). The point is
then dropped onto that face's exact support. The distance is signed by the reference's outward
normal and computed in parallel. Colors:

- green within `acceptable`;
- yellow to red outside the reference, saturating at `ramp`;
- cyan to blue inside it, saturating at `ramp`.

The tessellation keeps its points in single precision, so the map resolves deviations down to
about 1e-7 of the model's size.

Tests (`tests/contracts/jets_contract.lucb`, `tests/regressions/measure_tests.luc`):

- **Bases.** Third derivatives match differences of the second.
- **One-sided limits.** A C1 quadratic's second derivative jumps at its knot, and each side's limit
  is right.
- **Rational ellipse.** A rational quadratic elliptic cylinder's section has the curvature
  `ab/D^{3/2}` and the rate `-3ab(a²-b²) sin t cos t / D³`, to 1e-8.
- **Analytic sections.** Spheres', tori's and cylinders' sections are circles, with no rate.
- **Fillets.** A G1 fillet's contacts measure G1, with curvature 0 on the plane and 1/r on the
  fillet. A G2 fillet's measure G2, with curvature 0 on both sides and a rate on the fillet's side;
  asked for G3, they fail. A G3 fillet's measure G3.
- **Split faces.** The pieces of a cylinder's side and of a G3 fillet's strip, cut by Split Face,
  are equal at every order.
- **Combs.** A cylinder's rims comb at exactly 1/r, pointing out from the axis.
- **Deviation.** A sphere of 10 lies 0.5 inside one of 10.5 everywhere; a box's G1 fillet of r
  lies r (1 - 1/√2) inside the box at its middle; a model's deviation from itself is zero.

Not yet: combs on isoparms and on curves on surfaces (Stage 2 brings 3D curves), inflection
marks and torsion quills, a radius band on combs, and checks between faces that share no edge
(Stage 3's stitching).

## The NURBS toolkit (`nurbs/`)

Stage 1 of luced-3d's surfacing plan (`docs/research/SURFACING-ALIAS-PLASTICITY.md` there): The
NURBS Book's structure algorithms (Piegl and Tiller, chapters 5 and 9), public, on curves and on
B-spline faces with their trims. Everything works on homogeneous control points
(`x w, y w, z w, w`), so a rational curve's algorithm is the polynomial one. The degree cap is 11
(Rhino's rebuild bound; Alias builds to 9): the evaluators' bases are twelve wide.

| Call | What it does |
|---|---|
| `CadSpline.make(points, weights, degree, knots)` | A rational B-spline curve; `at`, `curvature`, `point`, `weight`, `knot`, `low`, `high`. |
| `inserted(t, times)` | Knot insertion (A5.1): the same curve, a control point more a knot. |
| `knot_removed(t, times, tolerance)` | Knot removal (A5.8), as far as the curve stays within `tolerance` (eq. 5.30's bound). |
| `elevated(by)` | Degree elevation: Bezier segments raised in closed form, then each inner knot taken back to its old continuity by exact removal. |
| `lowered(tolerance)` | Degree reduction: a least-squares refit one degree lower, inner knots kept, ends held to tangents; refused beyond `tolerance`. |
| `bezier()`, `piece(a, b)` | NURBS to Bezier (every inner knot to full multiplicity); a piece over `a..b` (split, extract). |
| `rebuilt(degree, spans, keep, rational)` | A least-squares rebuild on uniform knots, its ends held to G`keep` (0–3); with `rational` false, weights 1. `deviation(other)` measures it. |
| `CadNurbs.inserted(model, surface, direction, fraction, times)` | A knot inserted across a face's surface along u (0) or v (1). |
| `knot_removed(model, surface, direction, fraction, times, tolerance)` | The inner knot nearest `fraction` removed from every row, if every row stays within `tolerance`. |
| `degree_raised(model, surface, by_u, by_v)`, `degree_lowered(model, surface, direction, tolerance)` | Degree changes across the net. |
| `rebuilt(model, surface, kind, degree_u, degree_v, spans_u, spans_v, keep_u, keep_v, tolerance, most)` | Alias's Rebuild Surface types (`rebuild_explicit` …): Explicit (degrees and spans, Explicit Control), Within tolerance (spans added where it is furthest off, Alias's Curvature), Reduce spans, Del multi knots, Non-rational, Bezier. |
| `knots_matched(model, surface, other, other_surface, keep_u, keep_v)` | Match Knots: rebuilt on another surface's degrees and knots. |
| `converted(model, surface)` | An analytic face (plane, cylinder, cone, sphere, torus) as an exact rational B-spline: a bilinear patch, or a surface of revolution (A8.1) of a line or arc profile through an arc (A7.1), over its faces' chart box. |
| `structure`, `knot`, `knot_count`, `parameters_at`, `point_at`, `surface_of_face`, `surface_of_cv`, `first_cv` | What the UI reads: degrees, counts, spans, rational; knots as domain fractions; a point's parameters. |
| `cloud_moved(cloud, ids, mode, amount, falloff, reach, exponent, curve, mirror)` | CV moves on a control vertex cloud (`cv_moves.lucb`): World, NUV, Slide, Project, Parallel and View, with Rows, Hull-distance or Radius falloff, mirrored across X, Y or Z with `mirror` (each moved CV's mirror partner moves as its reflection, a CV on the plane stays on it). `cloud_frame` (the surface's frame NUV uses), `cloud_hull_frame` (the hull's, Slide's) and `cloud_row` give the gizmo's frames and a hull row's group. |
| `surface_text(model, surface)`, `CadNetProbe.parse(text)` | A surface's net as text, and read back elsewhere: `point(u, v)` at domain fractions, `knot_count` and `knot` (inner knots as fractions). luced-3d's viewport draws Insert Knot's isoparm under the pointer from it. |

Each surface edit returns a `CadNurbsEdit`: the model, how far the surface moved (0 for the exact
edits) and how many control points or knots changed. The edited surface is marked edited, so its
faces' trims take their nearest parameters on it (exact edits change nothing there); the B-rep's
other edited marks now survive modeling operations too (`shape_of` and `brep_of` keep them).

**Rebuilds hold continuity.** A fit (`fitting.lucb`) holds its first and last `keep + 1` control
points so the result's derivatives up to `keep` at both ends are the source's: those control
points follow from the end derivatives and the bases' derivatives there, one at a time. The rest
is least squares on samples (twice degree + 1 a span), its normal equations a band factored by
Cholesky. A surface is fitted along u for each v sample and for the v ends' cross derivatives (so
the boundary's cross curvature fits too), then along v through those rows. So a G2 fillet rebuilt
to two quintic spans held to G2 still measures G2 at its contacts. Explicit Control's single span
of degree 7 holds G2 at both ends with its eight control points; G3 needs eight a direction.

**Converting.** A face whose loops go round the axis with no seam edge (a sphere's caps from
`CadSolids.sphere`) would run off the B-spline's domain at the seam, so it is split first, as
Alias does: by the plane through the axis furthest from the face's vertices (`cap_faces`,
`caps_split`), each half then having a seam of its own. A sphere's or cone's face that caps the
pole or apex runs to it.

**CV clouds.** `CadGeometry.cv_cloud` points carry `cad_cv_model`, `cad_cv_surface`, `cad_cv_u`,
`cad_cv_v`, `cad_cv_nu` and `cad_cv_nv`: a pick takes a hull row by them, and Rows falloff counts
rows with them. They also carry their surface's frame at the CV's Greville point
(`cad_cv_normal`, `cad_cv_du`, `cad_cv_dv`: brep's `placed_cv_frames`), which NUV moves along, as
Alias's and Rhino's do; on a curved net it differs from the hull's. A set's CAD curves' control
points follow the models' (`cad_cv_model` -1, `cad_cv_surface` the curve's index, a net one row
wide), a curve on a face's as its chart curve's points mapped onto the face.

Tests (`tests/regressions/nurbs_toolkit_tests.luc`), against closed forms:

- **Exact edits.** Knot insertion (once, twice) and degree elevation (to 5 and to 11) leave a
  circle of two rational quadratic arcs a circle to 1e-12; removal undoes an insertion; the
  arcs' joint is no knot to remove within 1e-6; lowering a raised curve gives it back.
- **Rebuilds.** A quarter circle rebuilt to four quintic spans held to G2 is within 1e-4, its end
  curvature 1/3 to 1e-9; kept rational it is exact; a single span of degree 7 is within 1e-2.
- **Surfaces.** On a G2 fillet's face: insertion and elevation keep the surface to 1e-12 and the
  contacts G2; removal and lowering give them back; a two-span quintic rebuild held to G2 reports
  its deviation and keeps the contacts G2, as does a single span of degree 7; Within tolerance
  meets 1e-5; Bezier and Del multi knots are exact.
- **Conversion.** A G1 fillet's cylinder becomes a rational B-spline within 1e-12 of radius r,
  its contacts still G1; a cylinder's, a cone's and a torus's whole sides convert and mesh; a
  sphere's caps are split through the axis and convert, every piece on the sphere to 1e-9; a box
  face becomes a bilinear patch.
- **CV moves.** On a 3 by 3 planar net: NUV moves the middle CV along the normal alone; Rows
  falloff with one row moves a neighbor by 0.5^e and a corner by its square; hull distance 3
  moves a neighbor two away by 1/3; Slide stops at the net's edge; Parallel moves two CVs alike;
  Symmetry moves a CV's mirror partner as its reflection and keeps a CV on the plane on it.
- **Frames.** On a G2 fillet's quintic net every CV's cloud frame is `cv_frame`'s to 1e-6, and on
  it the normal differs from the hull's; a net sent as text reads back the same surface.

Not yet as public calls: knot vectors made compatible across a set of curves (Stage 5's builders
do it inside, `make_compatible`), reparameterization by chord length. Extension is Stage 5's
(Extend, below); Untrim is Stage 3's (Projection, trims and stitching, below).

## Curves (`curves/`, `cad_curves.lucb`)

Stage 2 of the surfacing plan: CAD curves as objects of a geometry set, beside its bodies and
sketches (the owner's answer: free 3D curves and curves on surfaces are first-class), Blend Curve
to G4, and Alias's curve edits.

**Curves as objects** (`cad_curves.lucb`, `CadCurves`). A set's `cad_curves` component holds named
curves: an exact rational B-spline each, shown or hidden, its source ("edge <reference>",
"isoparm <face> u 0.5", "blend", …) and, for a curve on a face, the face's name and the curve in
the face's chart coordinates (a plane's x and y, a cylinder's angle and height, a B-spline's
normalized domain). It places with the set (control points moved, chart curves kept), joins,
filters by name as an object path and is saved in geometry files (element `cad_curves`).
`with_curve`, `with_visible`, `without_curve`, `free_name`; `curve`, `chart_curve`, `face`,
`source`. luced-3d draws them (`curve_lines`, a curve on a face lifted off it a hair so the face
does not hide it; `curve_hulls`), picks them (`pick_mesh`: each segment a zero-area triangle whose
edges carry `cad_curve`, `cad_edge` and `cad_curve_name`) and edits their control points in the
CV cloud (`curve_cloud`, `moved_curves`: a curve on a face's moved points go back onto the face,
its 3D curve fitted again).

**Making curves** (`CadCurveTools`):

| Call | What it does |
|---|---|
| `by_points(points, degree)` | Alias's CV curve: the points as control points, clamped, equal spans, degree 5 by default (lowered to what the points allow). |
| `through(points, degree)` | Alias's EP curve: global interpolation (A9.1) at chord-length parameters, knots averaged. |
| `edge(model, reference)` | Duplicate Curve of an edge, exactly: a line (degree 1), a circle or ellipse arc (rational quadratics, A7.1), a B-spline edge's piece; from the edge's start to its end. |
| `isoparm(model, face, direction, fraction)` | Duplicate Curve of an isoparm, exactly: the net's row combined at that parameter (an analytic face through its exact net). |
| `isoparm_fraction(model, face, direction, p)` | Where along the face's net the isoparm through the point nearest `p` lies (luced-3d's isoparm picked by a double-click). |
| `face_at(model, p, fraction)`, `face_uv(model, face, p)`, `face_point(model, face, uv)` | The face a clicked point lies on (within a fraction of its size, inside its edges' box), a point's chart coordinates on it, and back. |
| `on_face(model, face, uv, tolerance)` | A curve on a face: the 3D curve along chart curve `uv`, fitted at degree 5 on equal spans, doubled until within `tolerance`, its ends on the surface. |
| `blend(a, ta, sa, ga, b, tb, sb, gb, degree, shape, extra)` | Blend Curve (below). |
| `attached(a, b, blend, bias, continuity)`, `closed(s, preserve)`, `stretched(s, params, moves, floating)`, `reversed(s)`, `nearest(s, p)`, `derivative(s, t, order)` | Alias's curve edits (below), the nearest parameter, derivatives to the fifth order. |

**Blend Curve** (`blend.lucb`, §6.2 of the plan). Each end is a point of a curve and the way the
blend leaves it (`sa` 1: past the curve's high end, its part below kept; -1 the other way). The
blend near an end is that curve reparameterized, B(τ) = C(φ(τ)) with φ' = α (the tangent's
length, Start tangent times the chord), φ'' = β (curvature's shape), φ''' = γ, φ'''' = δ, so its
derivatives there follow by Faà di Bruno and are G_k exactly whatever α > 0 and the others are.
The derivatives of a clamped B-spline at its start rest on its first k + 1 control points alone
(at its end on its last), so an end at G_k fixes k + 1 of them by a triangular solve on the
derivative curves' control points (A3.3). The other control points minimize ∫|B'''|² (∫|B''|²
when both ends are G0) with the fixed ones held: a Gram matrix by Gauss–Legendre on each span,
one linear solve. Degree 5 by default on as many equal spans as the fixed control points need:
G1 and G2 both ends one span, G3 three, G4 five (the table in §6.2); degree 7 gives a single span
to G3 (Explicit Control); `extra` adds spans whose points bend as little as they can. G4 is for
blend curves only (the owner's answer). A degree below the continuity asked is refused.

**The curve edits** (`edits.lucb`):

- **Attach** joins a's end to b's start as one curve. Blend moves both ends to meet at Blend Bias
  between them (0 a's end, 1 b's start); each curve's last rows of control points are set so both
  share the average of their arc-length derivatives there to G0–G3; the joint is a knot of full
  multiplicity, the curve G_k across it. Connect keeps the ends and bridges a gap with a straight
  span raised to their degree.
- **Close**: with Preserve Shape a G2 blend span from the end back to the start, attached; without,
  the two ends moved to meet halfway, G1.
- **Stretch**: up to degree + 1 handles on the curve moved; every control point follows a
  polynomial field of the parameter through the handles' moves (at its Greville abscissa), then a
  least-norm correction lands the handles exactly. Locked keeps the handles' parameters; Floating
  moves each to the point of the new curve nearest its target (three rounds).
- **Detach** is `piece`; **Reverse** mirrors the knots.

**Checking curves** (`measure/curve_joins.lucb`, `CadMeasure.curve_join`). Two curves meet at G_k
when their derivatives by arc length agree to order k: the unit tangent, the curvature vector and
its first and second rates, from the parametric ones by the inverse of s(t) (`arc_jet`). The join
reports the gap, the tangents' angle and the relative differences of the three higher vectors in
percent, and the order reached (gap 1e-6 of the curves' size, 0.01°, 0.01%).

Tests (`tests/regressions/curve_tests.luc`), against closed forms: blends G0 to G4 between a
quarter circle and a twisted quintic measure the order asked at both ends and not one more by
chance, with 6 + spans − 1 control points as §6.2's table says; mixed G3/G1 is one span; a single
span of degree 7 holds G3; a G2 blend leaving a circle of radius 2 starts at curvature 1/2 along
its tangent to 1e-9, its tangent as long as the chord; G4 at degree 3 is refused saying why; a
cylinder's rim comes out a circle of radius 3 to 1e-12 and its side's isoparm a circle, the other
way a line; a curve on the side lies on it within its tolerance; Attach Blend at 0.5 meets
halfway, G1; Close with Preserve Shape keeps the curve's points and closes G2, without it meets
halfway G1; Stretch lands both handles exactly and moves the middle halfway; curves as objects
replace in place, hide, pick, move through the CV cloud, place with the set, and save and load.

## Projection, trims and stitching (`surfacing/`, `modeling/trim`, `stitch`, `untrim`, `face_meets`)

Stage 3 of the surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md` §4.3,
§6.3–§6.5): Alias's Project, Intersect, Trim, Untrim, Trim Convert and Shell Stitch, Rhino's
ShrinkTrimmedSrf, and Parasolid's tolerant edges. `CadSurfacing` works on a geometry set, its bodies
by index and faces by name; curves on faces are the set's `CadCurves`.

| `CadSurfacing.` | What it does |
|---|---|
| `projected(set, body, face, curves, direction, normal, both, original, tolerance, stem)` | Project: the curves named (apart by `;`) cast onto the face along `direction` (both ways, or only ahead) or along its normals (Pull), each piece a curve on the face; `original` keeps each curve's degree and spans (Alias's Match original). A `CadSurfaceEdit`: the set, how many pieces, the worst fit. |
| `intersected(set, body, face, other, other_face, both)`, `intersected_plane(set, body, face, origin, normal)` | Intersect: where two faces meet (or a face and a plane), as curves on the first face, with `both` on the second too. |
| `trim_regions(set, body, face, curves, cast, direction, cutters, extend)` | The face cut by its curves: a marker inside each region (Alias's region markers), the crossings, and the loose ends that cut nothing (Show Intersections). `cast` 1 or 2 is Alias's 3D Trimming (curves off the face cast along `direction` or the normals as the trim runs), `cutters` faces cutting it where they meet it, `extend` short ends carried on. |
| `trimmed(set, body, face, picks, mode, curves, tag, cast, direction, cutters, extend)` | Trim: the regions holding `picks` kept (0) or discarded (1), or every region its own face (2, Divide, named `<tag>/divide<k>[<face>]`); the face before it pushed as a trim stage. |
| `untrimmed(set, body, face, all)`, `stages(set, face)` | Untrim Last (or All): the face's latest (first) stage back in place of its pieces, stitched to its neighbors; with no stage, its natural surface. |
| `trim_converted(set, body, face, du, dv, su, sv, swap, keep)` | Trim Convert: a four-sided face an untrimmed B-spline face of those degrees and spans, its surface and edge deviations reported; `keep` keeps the original as a body of its own. |
| `shrunk(set, body, face)` | Shrink: a B-spline face's net cut down to its loops' box. |
| `stitched(set, bodies, gap, detach_periodic, detach_single)`, `unstitched(set, body)` | Shell Stitch within a gap (how many edges it joined, the widest gap, whether it closed into a solid; with Alias's Detach options for export, how many faces and closed edges it split), and Unstitch (every face an open body). |
| `patch(points, weights, nu, nv, du, dv, ku, kv, name)` | A body of one B-spline face over its whole domain. |
| `aligned(set, input, master, continuity, …)` | Align (below): a `CadAlignEdit` with the set, Alias's Diagnostic Feedback, the side and the aligned rows. |
| `edge_precision(model, edge)`, `vertex_precision(model, vertex)`, `is_closed(model)` | A tolerant edge's or vertex's precision; whether every edge is shared. |

**Project** (`surfacing/project.lucb`). Along a vector d each point C(t) goes where the line
C(t) + λd meets the surface: S(u, v) = C(t) + λd by Newton on (u, v, λ), seeded by the last point's
answer along the curve, else by a 9 by 9 lattice over the face's chart, the nearest hit along the line
winning (both ways, or ahead only). Where the line runs along the surface (d along a cylinder's
rulings: the system is singular) the point's foot on the surface is the hit when the line passes
through it, so a circle on a cylinder cast along its axis lands on itself. Along the normal each point
goes to its nearest point of the surface. A point whose answer lies outside the face (its loops wind
round it nowhere in the chart) misses; runs of hits are the pieces, their ends found by halving the
step 48 times and then put on the boundary edge they reached exactly. Each piece's chart curve is
fitted at degree 5, its spans doubled until the surface along it lies within the tolerance (a
ten-millionth of the body by default) of the cast points, its far end found by continuation (a closed
curve's chart runs on past a whole turn); the 3D curve is fitted along it (`curves.face_curve`).

**Intersect** (`modeling/face_meets.lucb`) is the booleans' step 2 between two faces (their surfaces'
meeting curves in closed form, or traced, each clipped to both faces); each curve is then cast onto the
face along its normals, which gives its chart curve.

**Trim** (`modeling/trim.lucb`) is the booleans' steps 1 to 4 with the curves in place of a second
solid: each curve's crossings with the face's boundary (and with the other curves) cut it and the
boundary edges, which the faces round it share; the stretches inside the face are new edges across it
alone, and the face is split along them (`split.lucb`). A stretch with a loose end (a curve stopping
inside the face, Alias's dangling curve) cuts nothing: it is left out, and its end reported. A region
is told by a point: the region whose loops wind about it in the chart. What is kept is written as a
body that may be open (`write_pieces` without `closed`): a solid trimmed loses its closure, as a shell
trimmed in Alias does. The curves cutting the face are the set's curves on it, their 3D curves checked
to lie on its surface (one left on a face whose surface changed since is refused: project it again).

**Trim stages** (`cad_trims.lucb`): a set component holding, per trim, the face's name and the face
as it was (a one-face body placed with the set; saved as element `cad_trims`). Untrim puts the latest
(or with All, the first) back: the faces of that name and Divide's pieces of it go, the stage's face
comes in, stitched to the body where their edges meet within ten tolerances. With no stage a face
untrims to its natural surface (`modeling/untrim.lucb`): a B-spline's whole domain (its boundary rows
as edges, a row closing on its opposite one seam edge, a pole left out), a plane's rectangle round its
loops, a cylinder's or cone's whole turn between the heights it spans, a sphere or torus whole.

**Trim Convert** (`surfacing/convert.lucb`): the face's four sides are found at its loop's four
sharpest turns (a loop of four edges: its vertices) and followed in its chart by arc length; a Coons
map of the four chart curves carries the unit square onto the region, and the surface sampled through
it is fitted by least squares (rows, then columns; corners and sides held) on the degrees and spans
asked. Its deviation inside (on a grid) and along its sides is reported; the new face replaces the old
one, stitched to its neighbors within the edge deviation. **Shrink** cuts a B-spline's net down to the
box its loops span in the chart, and a fiftieth more each way (the loops' samples miss a curve's bulge
between them), by knot insertion (`nurbs.extract` along each direction); the edges stay.

**Stitch** (`modeling/stitch.lucb`): the bodies' shapes in one; vertices within the gap merge
(`Work.vertex` at a tenth of it), an edge passing a merged vertex within the gap is cut there, and two
edges between the same vertices whose middles lie within ten gaps become one (`cut_edges`); each joined
edge is then sampled against the edges it joined, refused when they lie farther apart than the gap. Faces
are turned so every shared edge is used once each way (a breadth-first walk; a one-sided shell refuses),
and a body whose every edge is shared is a closed solid. The joined edges keep the first face's curve;
their **precision** is the widest the joined edges lay apart, or the faces' surfaces off the edge
(sampled), and each merged vertex's the most of its edges'; the model's own tolerance stays as it was.
Unstitch makes every face its own open body.

Tests (`tests/regressions/surfacing_tests.luc`), against closed forms:

- a circle of radius 2 in the plane z = 10 cast along −Z onto a cylinder of radius 3 about Y lies on
  x² + y² = 4 and z = √(9 − x²) to 1e-5; a circle of the cylinder's radius cast along its axis lands on
  itself to 1e-6; pulled along the normals each point keeps its height and lies on the ray to it; a
  curve missing the face is refused;
- the cylinder meets the plane y = 2 in its circle; a box's wall x = 1 meets it in four lines (two on
  each face) at z = ±√8;
- Trim on a 4-box's top cut by a projected unit circle: two region markers; Keep leaves area π (the box
  opened), Discard 16 − π, Divide both faces with the sum 16 and the box still closed; the stage saves
  and loads with the set; a curve stopping inside the face reports one loose end and no regions;
- Divide, Unstitch and Stitch of the two pieces join one edge and leave one face's area round four open
  edges; Untrim of the kept disc gives back the closed box of volume 64; after two trims Untrim takes
  the last back (one hole) and Untrim All both; a stageless B-spline face untrims to its whole square;
- a box unstitched into six open bodies stitches back into the closed cube of volume 8; with its top
  raised 0.01, a gap of 0.02 stitches all twelve edges with precision 0.01 (kept when saved and
  loaded) and Check passes G0 across them within 0.02; at 0.005 the sides join (8 edges) and the top
  stays apart, and the top and one side alone refuse; a filleted box unstitched and stitched again
  measures G1 across every fillet edge;
- Trim Convert of a G1 fillet at 4 quintic spans lies within 1e-4 of it, its face a B-spline and the
  body closed with the same volume; Shrink of a disc trimmed on a converted top keeps area π, the
  net's corner moved in to the disc's box.

**The trim's curves** (`surfacing/trim_curves.lucb`). With Alias's **3D Trimming** the curves need
not lie on the face: those off it are cast onto it as the trim runs (Project's casting, along a vector
or the normals), and faces of any body cut it where they meet it (Intersect's meeting curves, pulled
onto it); none of them is kept on the face. **Extend** carries a curve end that stops inside the face
on along it: from the end's chart point, straight on in the chart along the end's own direction, in
steps of a 512th of the chart, until the face's loops no longer hold it (then halved 48 times and put
on the boundary edge exactly) or it comes within a step of another of the curves (then onto that
curve's nearest point); the chart line, lifted onto the surface, is a curve of its own joining the
first end to end, so the region it closes off is cut. An end within ten fit tolerances of the boundary
or of another curve is not loose.

**Tolerant edges in the booleans** (`modeling/shape.lucb`, `boolean.lucb`, `clip.lucb`). A shape keeps
each edge's and vertex's precision beyond the tolerance (`edge_slack`, `point_slack`, from the B-rep's
columns; `brep_of` writes them back). The booleans read them edge by edge, as Parasolid's tolerant
modeling does, instead of the model's largest: vertices merge within their own precision, an edge's
crossings and the boundary test (`face_contains`) reach as far as that edge's, new edges ending on a
tolerant edge merge their ends within it (`Work.section_end`), and the pieces of a tolerant edge keep
its precision in the result (`write_result`). The tessellator still works to the largest.

**Untrim through an apex**: a cone's band whose height reaches the apex closes on it, as the pointed
cone primitive is made (the seam from the circle to the apex vertex, no circle there).

**Stitch's Detach options** (`modeling/stitch.lucb`'s `detached`), for systems that refuse faces
wrapping round their surface and loops of one closed edge: Detach periodic splits each face that uses a
seam edge both ways by the plane through the surface's axis and the seam (`split_faces_by`), so each
half meets the seam once; a torus is cut again by its equator plane for its other seam; a periodic
B-spline face is refused (rebuild it open). Detach single-curve boundaries splits each closed edge that
alone makes a loop at its middle, every loop using it taking both halves.

Tests (`tests/regressions/trim_followup_tests.luc`): a free circle cast down as the trim runs keeps a
disc of area π and leaves no curve on the face, and without casting it is refused; a cylinder of radius
1 through a box's top cuts the same disc by Intersect; a line stopping in the middle of the top,
extended, cuts the strip z > 0.5 of area 6, and a second line stopping short of it runs on to meet it
(the quarter of area 5); a pointed cone's side untrims to the whole side (area π r l, volume π r² h/3);
a cylinder unstitched and stitched with Detach single-curve boundaries has its two caps' circles in two
edges each, closed with its volume, and with Detach periodic its side in two halves none of which
meets an edge twice; a rod cut from a stitched box through its tolerant top edges (gap 0.01) closes
with the half rod's volume gone, the cut pieces of those edges keeping precision 0.01.

Not yet: trims with Keep Originals on bodies; Detach periodic on closed B-spline faces.

## Align (`surfacing/align*.lucb`)

Stage 4 of the surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md` §4.2, §6.1):
Alias's Align and Symmetry Plane Align, with every option of Alias's window, Align 2008's Modify Both
and Rhino's Refine. `CadSurfacing.aligned(set, input, master, continuity, …)` moves an input surface's
edge (or a curve's end) onto a master and matches it G0 to G3 by its first CV rows.

**Words.** The input is "face <name> x y z" (the side of the face's net nearest the point) or "curve
<name> x y z" (the curve's end nearest it). The master (`align_master.lucb`) is "edge <face> x y z" (the
face's boundary edge nearest the point), "isoparm <face> <0 or 1> <fraction>", "curve <name>" (on its
face when it lies on one), "face <name>" (Project and U/V types), or "plane ox oy oz nx ny nz [free]"
(Symmetry Plane Align; `free` leaves the edge off the plane: Project For Position off).

**The method** (`align.lucb`). The input's surface becomes a polynomial net (an analytic face's exact
net, a rational one refitted non-rational within a ten-millionth of the body), turned so the side to
align is its u = low rows (`align_rows.lucb`). At a clamped end the cross derivatives rest on the first
rows alone, S^(k) = Σ_{i≤k} c_k^i P_i, so row k is solved from the k-th derivative and the rows before
it. The master answers at each station along the edge with a point and its cross curve Q: on a surface
the curve whose chart is the straight line across the edge (the other parameter's isoparm when the edge
runs along a parameter line, which keeps a polynomial master's fields polynomial; square to the edge
otherwise; on the face's side of an edge, found once in the edge's middle; for an isoparm or curve on a
face, the way the input leaves), on a free curve the curve itself. Q is handed on as its arc-length jet
(T, K, K'). The input's cross curve A, run toward the master, gets
A' = vT, A'' = v'T + v²K, A''' = v''T + 3vv'K + v³K' (Faà di Bruno), which is G0 to G3 across the edge
whatever the speeds: along the edge the shared curve and the matched normal section give the rest
(G1 along the edge then fixes the mixed terms). The speeds are Alias's sliders by the hull: v puts row 1
the input's own distance behind the edge along T times the Tangent Scale (the CVs move least), v' row 2
behind row 1 times the Curvature Scale, v'' row 3 behind row 2 times the G3 Scale; Starting and Ending
scales run linearly along the edge; AutoScale G2 and G3 carry the tangent (and curvature) scale into the
later rows; Tangent Balance makes the distances follow the master's speed |Q'| along the edge at the
input's mean (Alias's "mimic the master's inner hull"); Skew turns Q's chart direction in the tangent
plane by an angle running from start to end. Each row is fitted at 2(p + 1) stations a span (least
squares on the row's own knots, its ends exact) and the next row solved from the fitted ones, so their
errors do not add up.

**Exactness and Refine.** When the master's derivative fields lie in the rows' space the match is
exact: Explicit Control off raises the input to quintic across (the multi-span quintic) and, with
Position Influence 1, to the master curve's degree along the edge with its knots. Otherwise the
stations are checked between the fitted ones (65 along the edge) against the limits (the G0 gap, the G1
angle in degrees, the G2 and G3 differences in percent; the report is Alias's Diagnostic Feedback, its
start and end angles against the master's own cross direction), and Refine inserts a knot in the
middle of each span holding a station that misses, aligning again, up to Max New Spans.

**Rows across.** The order's k + 1 rows, as many kept at the far side (its own continuity), and the
extra rows between: 2(k + 1) + Extra CV Rows. Explicit Control on refuses fewer ("Align G3 needs 8 rows
across, has 6") and refits the input to the degrees and spans asked; off inserts spans across, the
widest first.

**The options.**

- **Position Influence** blends the input's own parameterization (each station to the master's nearest
  point, the ends moved onto the master's ends) into the master's (the range in equal steps).
- **Partial** aligns only a range of the input's edge: the range's ends and a span a tenth of it outside
  each get knots of full multiplicity, the master's nearest part is matched inside, and the edge beyond
  the outer knots stays exactly where it was.
- **Extra CV Rows / Blend Factor**: rows k + 1 … k + E carry the last row's change by weights
  (1 − i/(E + 1))^(2^factor): 0 linear, positive dying faster, negative lasting.
- **Types**: Edge; Project (the input's edge cast along the vector onto the master face, the cross
  direction square to the cast curve on the face, a curve on the face left there unless NoCOS);
  U/V (a curve's end to the face's U or V direction); Vector (rows 1 and 2 move only along the vector,
  their normal component solved: S_u in the master's tangent plane, the normal curvature along S_u the
  master's).
- **Two masters** align a second side (the one nearest its point, or the side across) after the first;
  the far rows of each are the other's.
- **Modify Both** (the master an edge of a B-spline-able face): the input to the middle of the two
  edges and cross jets (T averaged, K averaged square to it, K' with its tangential part −|K|²), then
  the master all the way to the input as it now is; both faces written.
- **Symmetry Plane Align**: the edge onto the plane (Project For Position), T square to it, K the input's
  own kept in the plane, K' = −|K|² T: what a curve and its mirror image run back need to meet G3.
- **Curves**: the CVs from the end are the rows, one each; a free master curve gives Q itself (at its
  nearest point or Attach Point), a face the U/V direction or the way the curve leaves; Modify Both on
  two free curves moves both ends to the middle. Against a surface master (an edge, an isoparm, a
  curve on a face or the face; `align_both.lucb`) Modify Both bends the surface locally: its CVs move
  by a bump in its chart ((1 − d²/r²)³ of their Greville places' distance within a quarter of the
  chart, scaled so the met point lands exactly halfway to the curve's end), the curves on that face
  are lifted onto it again from their chart curves, and the curve's end then aligns to the surface as
  it now is.

The result writes a natural face as a new face of the same name, facing as before, stitched to its body
where its edges still meet; a trimmed one on the moved surface, its trims taking their nearest
parameters (`with_face_surface` with `edited`). It reports the aligned rows, the input's whole hull
before the align (Alias's proxy display: `proxy_point(row, index)` over `proxy_row_count()` rows of
`proxy_size`), the side and the order (luced-3d re-runs an Align when a CV of those rows is dragged
later).

Tests (`tests/regressions/align_tests.luc`), measured by the Check tools across the stitched edge: a
flat quintic strip aligned to a quarter cylinder of radius 2 measures exactly G1, G2 and G3 as asked
(and fails the next), its normal curvature across the edge 1/2 to 1e-6 from G2 on; Tangent Scale 2
doubles row 1's distance (0.56 to 1.12), 1 to 2 runs from one to the other, AutoScale G2 doubles row 2's
too, Tangent Balance evens a fanned strip's distances, Skew 10° reports 10° at both ends still G1, Extra
CV Rows 2 moves rows 2 and 3 by 2/3 and 1/3 of row 1's change and row 4 not at all; an isoparm master
measures G2; a curve's end to the cylinder's U lands on it G1; Position Influence 0 keeps the knots
along the edge; Explicit Control refuses G3 on six rows and off adds spans to eight; Partial 0.2–0.8
leaves the edge outside 0.13–0.87 where it was and lies on the cylinder's edge inside; Modify Both on a
symmetric roof leaves the halves mirror images, the ridge in place and the tangent rows level; Vector
along X slides row 1 onto x = 2 only; two masters align both sides G1; Symmetry Plane Align puts the
edge on x = 0 and row 1 square to it; Refine meets a quarter circle a cubic edge misses (gap 1e-6);
a line's end aligns G2 to a circle (curve_join); Project casts a hanging strip's edge onto a floor and
leaves Align1 on it.

Not yet: Alias's Accept/Restore checkpoints (each Align is a step or node; undo and the history
serve); the Vector type at G3 (Alias offers it at G1 and G2).

## Surfaces from curves (`surfacing/builders.lucb`, `square`, `square_fields`, `skin`, `sweep`, `sweep_frames`, `sheets`, `extend`, `extend_chain`)

Stage 5 of the surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md` §4.6, §6.6):
Alias's Square, Skin, Rail, Profile, Set Planar, the sheet Extrude and Revolve of free curves, and
Extend, as `CadSurfaceBuild`'s static calls. Each takes a geometry set and the words of its curves,
Align's master words apart by `;` ("curve <name>", "edge <face> x y z", "isoparm <face> <0 or 1>
<fraction>"), so a boundary on a surface can be asked for its cross derivatives (`align_master.lucb`),
and returns the set with a new body of one sheet named `name` (Extend changes its face; a
`CadSurfaceEdit`).

**Square** (`square.lucb`; Alias's, Plasticity's and xNURBS's Square, Rhino's EdgeSrf). Four boundaries
in any order and either way round are walked end to end (within a ten-thousandth of their size) and
made the patch's sides: A (u = 0) and B (u = 1) along v, C (v = 0) and D (v = 1) along u; each pair
made compatible (`make_compatible`: one degree, quintic when any side asks continuity, one knot
vector). The surface is a Gordon-Coons patch with Hermite blending, S = P1 + P2 − P12, built as nets
and so exact: P1 the Hermite in u between A's and B's fields (the boundary and its cross derivatives
∂S/∂u, ∂²S/∂u² along it, each a B-spline curve on the pair's knots: the Bezier rows from the end
derivatives, `hermite_rows`), P2 likewise in v, P12 the tensor Hermite of the corners' mixed
derivatives; the three made alike and added. Per picked boundary its continuity: Free and G0 (no
field, the bilinear Coons patch from four lines; Blend Cubic gives them the G0 patch's own slope), G1
(∂S/∂u = L − (L·n)n, n the neighbor's normal, L the cubic Hermite of the corners' tangents and the G0
patch's twists), G2 (also ∂²S/∂u² = M − (M·n)n + II(∂S/∂u, ∂S/∂u)n, II the neighbor's second
fundamental form, so the normal curvature across matches; along the edge the shared curve gives the
rest), Implied Tangent and Curvature (a planar boundary's plane as the mirror: ∂S/∂u square to it,
∂²S/∂u² in it) and Tangent Angle (the neighbor's tangent plane turned about the edge; positive lifts the
surface toward its normal side). The corners' mixed derivatives are one set every field holds
(`corner_set`: the boundaries' own along them, the mean of the two sides' targets where both have a
field, measured by fourth-order differences a thousandth apart), each field fitted on its pair's knots
holding them to the second derivative at both ends; spans are added where a field misses its target
by more than a millionth until it does. The G1 slope keeps L's part along the boundary and turns its
part across onto the neighbor's own way on (its tangent plane, square to the boundary) with its
length, so walls whose next boundary leaves their plane (a cap on a box's four walls) still make the
patch rise square off them instead of shrinking the slope to nothing.

*Corners whose neighbors disagree.* At a cap on four perpendicular walls each wall asks the patch to
rise square off it while the next wall's top edge leaves the corner flat: no surface meets both at
the corner itself (the corner is singular). As Gregory's patch keeps each side's own field everywhere
but at the corner, each field here is its own target plus the corner set's mismatch (value and first
two derivatives) carried down to nothing by a quintic Hermite within a corner zone, 1/256 of the side,
a span of its own ending in a C2 joint (the knot degree − 2 times), so the fit is exact; off the zones
every boundary meets its continuity. Only a mismatch in the field itself makes a zone; twists alone
that disagree are spread along the side by the fit, as Coons patches do.

*Alias's other options.* **Boundary Blend** (1-3 and 2-4, 0.17 to 0.83) moves where a pair's influence
meets: the first boundary's slope across scaled by 2b and the second's by 2(1 − b); **Influence** scales
both; each acts off the corners (a quintic smoothstep over the first and last quarter, so the corners'
data stay) on boundaries with a slope across (G1, G2, Implied, Cubic), their continuity unchanged.
**Colinear** (per boundary) lays the slope along the neighbor's own isoparm across (Align without
skew) off the corners, so isoparms run on across the boundary. **Rebuild** (per boundary) refits the
curve by its arc length at its degree (5 at least) on the fewest uniform spans within the build
tolerance, its ends and tangents kept (`rebuilt_curve`, shared by the builders).
Explicit Control refits to the degrees and spans asked, its sides held.

**Skin** (`skin.lucb`; Alias's Skin, Loft). The sections turned to run alike (each the way nearer the
one before), made compatible (rational ones kept rational), and their control points interpolated
across at the mean chord between neighbors by a B-spline of the degree asked (quintic, at most one
less than the sections), homogeneous, so two coaxial circles give exactly a cylinder or a cone and
three a surface of revolution. Closed: interpolated with three loops more either side and cut back to
the loop, the seam smooth. Start and End continuity match the first and last section's side G1 to
G3 to the surface it lies on by Align's rows (`match_side`: Explicit Control off, Refine). A section may
be a curve set (curves end to end, words apart by `+`, Alias's): Single Surface joins each set into one
curve, **Multiple Surfaces** skins the sets' first curves, their second ones and so on, a sheet each
(bodies `Skin`, `Skin2`...). **Proportional Crown** (two sections, −0.25 to 0.25 of the gap) raises the
surface to degree 2 across and pushes the middle row along the normal at each control point's Greville
place by twice the crown times the gap, so the middle rises by the crown times the gap; then quintic
across.

**Rail and Profile** (`sweep.lucb`; Alias's Rail Surface and Profile, Rhino's Sweep1 and Sweep2). Each
generation curve (profile) is placed where it comes nearest the rail and kept as offsets in the rail's
frame there; a section anywhere is the offsets blended between the curves on either side and put back
in the frame. Frames: Natural (rotation-minimizing, by double reflection along 2048 steps), Parallel,
Vector (a fixed direction up), Surface normal (Profile on a curve on a face). Two rails: each section
reaches from the first to the second, its offsets along the chord scaled to it (Scale), along the chord
only (Non-proportional scale) or not at all (Rotate). Blends: Rail's linear with a bias; Profile's
Linear, Local (eased), Global (the natural cubic spline through all) and Limited (Fritsch and Carlson's
monotone cubic: no overshoot). Everything runs by the fraction of the rail's length (a Gauss-Legendre
arc table, Newton within a step), not its parameter: a circle's quarters are only C1 in parameter, the
length as smooth as the rail. The surface is the sections' control points fitted along the rail at
degree 5, the first and last exactly, knots at each section's place as often as the blend's kink asks,
spans doubled until the sections between lie within a ten-millionth. A rail or the end generation curve
on a surface matches that side G1 to G3 (Align's rows). **Spine** mode takes the frame of a spine curve
(its natural frame where the plane square to it passes the rail's point), so every section stays square
to the spine. **Sweep Pivot** sets where the first generation curve meets the rail: Closest (as above),
On Curve (fractions along both) or Off Curve (a fraction along the rail and a point in space); **Fixed
Curve** Rail moves the generation curve so its pivot is the rail's, Gen. sweeps so and then moves the
whole sheet back by the same, the rail carried onto the generation curve (refused with a rail's
continuity). **Curve Segments** sweep only a stretch of each rail and of the generation curves
(fractions of their domains, `segment_of`).

**Set Planar, Extrude, Revolve** (`sheets.lucb`). Set Planar: the curves' plane (Newell's normal of their
samples; refused beyond a ten-thousandth off it), a plane face wide enough, trimmed by the curves cast
onto it (`trim_face`, Trim's machinery), the regions inside an odd number of curves kept (a curve inside
another a hole); a curve that does not close is refused. Extrude carries each curve's control points
along a direction (degree 1 that way); Revolve turns each round its circle about the axis (rational
quadratic arcs, `arc_spline`): both exact, the sheets joined where they meet.

**Extend** (`extend.lucb`; Alias's Extend, Parasolid's PK_BODY_extend, Rhino's ExtendSrf). A face's
natural edge (or a curve's end), by a distance along the middle isoparm across the edge or a percentage
of its length, column by column of the net (one parameter length for all), each extension a Bezier
joined with full multiplicity and its knot removed as far as the curve stays: Linear (E + hsD1: G1),
Natural (the last span's own polynomial carried past the end by de Casteljau beyond 1, homogeneous, so
an arc stays on its circle: the same surface), Soft (C2, the second derivative dying to straight:
E + hsD1 + h²D2 g(s)/2, g'' = 2(1 − s)²) and Reflective (C2, the curvature's change mirrored:
E + hsD1 + (hs)²D2/2 − (hs)³D3/6). Merge off keeps the extension a sheet of its own. A trimmed face is
refused (Untrim or Trim Convert it first). Several inputs (words apart by `;`) extend one after
another; **Chain Select** adds, from each picked face edge, the natural sides of other untrimmed faces
that meet it end to end (within a millionth of the set's size) and run on the way it arrived (within a
degree), both ways along (`chain_from`).

For the viewport, `CadSurfaceBuild.picked_lines` gives each named curve (curve, edge, isoparm) as a
polyline, which luced-3d lights while a builder waits, and `curve_of` gives the curve itself (Blend
Curve between temporary isoparms).

**Stage 3 and 4 follow-ups that came with it.** Stitch's Detach periodic splits closed B-spline faces too
(`detach.lucb`: a natural face whose seam edge it uses both ways, cut at the middle of the direction it
closes in, exactly; a lone body may be stitched for Detach alone). `CadTrimRegions` hands the cutting
curves as the trim takes them (`cut_count`, `cut_point`: 3D Trimming's cast curves for the viewport).
Project takes temporary isoparms ("isoparm <face> <0 or 1> <fraction>" among its curves).

Tests (`tests/regressions/surface_build_tests.luc`), against closed forms and measured by the Check
tools: Square of four lines picked in any order is the bilinear patch to 1e-12; Square of a hole in a
biquadratic height field meets its four neighbors G0 (and a Coons patch not G1), G1 and G2 on every edge,
quintic; Blend Cubic of a plane's four lines is the plane; Implied Tangent leaves the symmetry plane
square; Tangent Angle 30° from a floor leans 30°; Skin of two circles is a cone and a cylinder and of
three their surface of revolution to 1e-12; a closed Skin's seam closes and turns less than 0.05°;
Skin Start G1 to a floor's edge measures G1; Rail of a circle round a circle is a torus to 1.2e-6 (a
ten-millionth of its size); two rails: each section reaches both within 1e-5 (Scale and
Non-proportional); Profile's transitions pass every profile, Linear is a pair of cones, Limited never
swells past the widest; Set Planar of a circle with a hole has the ring's area; Extrude and Revolve are
exact to 1e-12; Extend Natural keeps the old surface inside the new to 1e-9 and reaches 0.5, Linear
meets G1 (not G2), Natural G3, Soft and Reflective G2; a quarter circle extended Natural stays on its
circle; Detach periodic splits a closed Skin into two faces of the same area; Align's proxy is the
input's own hull; a curve's Modify Both against a cylinder's edge meets the surface halfway; Project
casts an isoparm. The follow-ups: a G1 (and G2) cap on a box's four walls measures G1 (G2) on all four
walls, its fields fitted exactly (12 rows), its middle rising; Boundary Blend 0.3 and 0.7 and Influence
2 and 0.4 keep G1 on every edge of the hole and move its middle; Colinear leaves along the neighbor's
isoparm (square to the boundary without it); Rebuild of a bunched eight-span line puts its middle by
length at the surface's middle along it, with fewer control points; Spine keeps every section square
to the spine (Natural leans); On Curve with Fixed Rail moves a post onto the rail, Fixed Gen. moves the
sheet to the post, Off Curve by the point; Curve Segments sweep the middle half of the rail and the
lower half of the post; Skin of two curve sets is one sheet joined, or two with Multiple Surfaces;
Proportional Crown 0.1 between lines 2 apart lifts the middle 0.2 all along; Extend with Chain Select
carries a neighbor's edge on too, as two picks do; `picked_lines` gives one line a curve.

Not yet: Skin's Flow Control and Modify Range, Rail's Colinear Options and Rebuild per curve, the
builders' Explicit Control on Skin, Rail and Profile; Alias's Multi-Surface Draft and N-sided (Stage 7's
fill). Align's Vector type stays at G1 and G2, as Alias's (its 2015 reference offers Vector at those
orders only; G3 asks the Edge type).

## Blends between surfaces (`surfacing/along.lucb`, `blend_surface`, `surface_fillet`, `fillet_ball`, `fillet_trims`, `tube`)

Stage 6 of the surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md` §4.1, §5.3,
§6.7): Alias's Freeform and Profile Blend, Surface Fillet (Parasolid's face-face blend) and Tubular
Offset, as `CadSurfaceBlend`'s static calls, each returning the set with a new sheet (Surface Fillet's
trims change the faces it rolls on).

**Sections along a run** (`along.lucb`). A source gives the section at any fraction of the run, each the
same count of homogeneous control points on the same knots across; their rows are fitted along at
degree 5, the first and last sections exactly, on uniform spans doubled until the sections between lie
within the tolerance. Homogeneous rows keep rational sections exact wherever they vary as a polynomial
of degree 5 or less along the run.

**Blend Surface** (`blend_surface.lucb`; Alias's Freeform Blend and Profile Blend, Plasticity's Bridge
Surface, Rhino's BlendSrf). Two sides, each a chain of Align's master words (edges, isoparms, curves on
faces, free curves; a chain's apart by `+`, joined end to end), matched by the fraction of their length
(Flow Control Connect ends; Free: each station on side 2 where the plane square to side 1 cuts it),
within each side's range (Modify Range). At a station the section is a degree 5 B-spline across, as many
control points as the two sides hold (six at least; eight, three spans, at G3 both sides), its end rows
the curve continuing each side's surface across it by arc length (§6.1's rows: A' = vT, A'' = v²K,
A''' = v³K', the cross curve's unit tangent, curvature vector and its rate; an edge's face continued
past the edge, a curve on a face or an isoparm's face toward the other side), v the side's Shape times
the chord; the loose middle and the orders a side does not ask follow the ends' cubic Hermite (a line
between G0 sides) bowed by Proportional Crown along the normal of the chord and side 1. A profile
curve (Profile Blend) is passed at its station: the section there fitted to it with the same end rows,
the difference from the free section carried along by a cubic between profiles (constant past the
ends), so the end rows stay exact; a section has two more control points when profiles shape it. A
free curve's side is G0 only (refused above, as Alias's).

**Surface Fillet** (`surface_fillet.lucb`). A rolling ball between two face sets (face masters), each
rolling on its side toward the other set (summed over samples) or flipped. The ball at a station: the
two contacts' chart points by Newton on the two sets moved by the radius along their normals agreeing,
in the station's plane (four equations, four unknowns, a finite-difference Jacobian, halved steps);
a contact leaving its face for another of its set carries on there. The seed is the best of the first
set's samples moved onto the second; the spine is marched both ways along the normals' cross until a
contact leaves its set, the end found by bisection to a hundred-billionth of the model; the centers are
interpolated by a cubic by chord length, and sections are cut square to it. Sections: G0 Chamfer (the
chord), G1 Circular (the ball's arc, rational quadratic), G2 and G3 Curvature (`fillet_section`, the edge
fillets' quintic sections on the same contacts and corner, bent by each face's normal curvature toward
it; G3's rate of curvature is matched on flat and round faces), their arms by the Form Factor (0.1 to
2). Chordal: each station's radius scaled until the contacts lie the chord apart. Trim Automatic: each
face's contacts as a cubic cast onto it along its normals (Trim's 3D Trimming, loose ends extended),
the part off the far side of the other set's contact kept; Curves on surface: the contacts cast onto
the faces as curves on them (Project along the normals); Off.

`CadMeasure.check` takes `faces` (names apart by `;`) to measure only the edges of those faces: the
builders' and blends' Continuity Check of a new sheet against its neighbors, without the neighbors' own
edges.

**Tube** (`tube.lucb`; Alias's Tubular Offset). Exact circles (nine rational control points) square to a
curve, about a center moved by Normal offset (along the face's normal for a curve on a face or an
edge's face, else the curve's rotation-minimizing frame's up; Flip negates it) and Tangent offset,
fitted along the curve by its length.

Tests (`tests/regressions/blend_build_tests.luc`), against closed forms and the Check tools: a G1 Surface
Fillet between perpendicular 4 by 4 planes is the cylinder of radius 1 about x = z = 1 to 1e-12, from y = 0
to 4 within 1e-9; trimmed (Automatic) each plane keeps 12 of 16 and the contacts measure G1, G2 and G3
as the section asks; Chordal 1 puts the contacts 1 apart, 1/√2 from the edge; a ball between a floor
and a cylinder 1.5 above it keeps every section point a radius off its center (to 1e-9) with its
contacts on the floor and the cylinder; Curves on surface leaves two curves on faces. Blend Surface
between two cylinders' edges measures exactly G0, G1, G2 and G3 at both edges as asked (G3: degree 5,
eight rows across, three spans); Crown 0.1 lifts a G0 blend's middle 0.5, a tenth of its chord of 5; a
profile curve lies on the Profile Blend within 1e-5. Tube along a line is the cylinder to
1e-12, along a circle the torus within 1e-7; Normal offset moves the axis, Flip the other way.

Not yet (Stage 8 and later): Alias's Bias and Lead sections, one-sided Curvature, variable radius and
chord along the fillet, Flow Control Edge align, Interior Edge Align, Explicit Control on the blends;
G3 Curvature's rate on faces whose curvature changes; Tube's variable radius and its trims; Round,
Ball Corner, Corner Blend, Bridge Fillet; conic sections.

## Fill (`surfacing/fill.lucb`, `fill_bounds`, `fill_net`, `fill_bands`, `fill_reference`, `fill_solve`, `fill_refine`, `fill_api`)

Stage 7 of the surfacing plan (luced-3d `docs/research/SURFACING-ALIAS-PLASTICITY.md` §5.2, §6.8): xNURBS's
fill (Plasticity's xNURBS and Patch, Alias's N-sided, Rhino 9's Patch), as `CadSurfaceFill.fill`, which
returns a `CadFill`: the set with the fill added as a body of one sheet, each boundary's worst gap, angle
and curvature difference, the guides' worst miss, and the conflicts (`FillConflict`: the boundary, the
order that fails, where, how much, whether a corner). `CadSurfaceFill.hole` turns one edge of a hole into
the words of all its edges.

**Boundaries** (`fill_bounds.lucb`). Align's master words in any order, walked end to end into one closed
loop (a single closed curve is one); a gap over a ten-thousandth of their size is refused with its size.
Each has Free, G0, G1 or G2 (Square's codes; Free keeps the position and is not checked; G1 and G2 only
beside a surface). The neighbor beside a boundary gives at a station its normal and its second
fundamental form as a 3 by 3 quadratic form H (II(X, X) = Xᵀ H X on its tangent plane), so the fill can ask
its own normal curvature across, whatever its parameterization. A hole picked by one edge: the open edges
of every body walked end to end; where sheets touch at a corner, the edge whose face shows the same side
and turns toward the hole.

**The net and the energy** (`fill_net.lucb`, `fill_bands.lucb`). A polynomial net, quintic both ways on
uniform spans (Explicit Control: degrees 2 to 9 and 1 to 64 spans each way, no refining). It minimizes
w_b ∫|∇²(S − R)|² + w_t ∫|∇³(S − R)|² + w_s ∫|∇S|² + w_f ∫(n0 · S)²: bending, and with a G2 boundary also
its rate (Moreton and Séquin; the order in which position, slope and curvature are well posed), both
measured from the reference R; the membrane (Tension) and the pull toward the loop's best plane
(Flatness) of S itself. The Gram matrices of the bases' derivatives are exact (eight Gauss points a span).
The normal equations are banded (unknowns u-major, half-width 3 (du·nv + dv) + 2) and factored by band
Cholesky, n h² work; the unknowns are the change from the reference (or a base plane), so a fill that
is nearly its reference comes out to the last digits: a planar pentagon is its plane to 1e-12.

**Constraints** (`fill_solve.lucb`). Rows at stations along the boundaries (four a span crossed, never on
a corner but G0 at each corner): G0 S = B; G1 n · S_d = 0 (d the chart direction across); G2
n · S_dd = II(S_d, S_d) (Pegna and Wolter), quadratic, by Gauss–Newton: each solve's rows linearized about
the last answer (iterating the right side alone diverges: a larger S_d asks a larger S_dd); guide curves
and points S = G (with Guide G1 also n · S_u = n · S_v = 0 on a face); Quad's flow (Normal: the cross
derivative square to the boundary; Next: along the blend of the neighboring boundaries; Adjacent:
along the neighbor's own cross direction) a hundredth as strong. The rows weigh a million times the
energy's diagonal and the method of multipliers drives them to hold: penalties alone trade exactness
for the system's condition (a weight of ten million left a planar fill 2e-8 off its plane).

**The reference** (`fill_reference.lucb`). Bending alone makes a ball's cap a paraboloid's (the clamped
plate's minimizer is a quadratic of the radius, 1% of the radius high on a 30° cap). So the energy is of
S − R, R the neighbors carried on into the hole: each side's ribbon is its neighbor's own surface where
the line through a chart point along the chart's normal meets it (a plane, sphere or cylinder in closed
form, else Newton within a B-spline's domain, accepted near the side), else its tangent plane met by the
same line; the ribbons are heights over the chart, blended by the inverse fourth power of the distance
to each side (so the blend cannot fold), and fitted into the net on the spans' grid.

**Domains.** N-sided: the loop projected onto its best plane (Newell's normal) gives the stations' chart
points; the net covers the projection's box with a margin of a tenth, and is trimmed by the boundaries
laid on it (their feet on the fill, fitted at degree 5 with breaks at each boundary's own inner knots to
a billionth of the loop, by its own parameter so Stitch meets the middles), keeping the region holding a
point a step inside the loop. Quad: four boundaries on the sides of an untrimmed patch (its domain their
mean lengths), the boundary rows fitted on the net's knots by the curves' own parameters and fixed.

**Refinement, measures, conflicts** (`fill_refine.lucb`). Between the solve's stations each one is measured
as Stage 0 measures an edge (the gap, the angle between normals, the normal curvatures across relative to
the larger with the Check tools' floor); while one fails (or a Quad row misses its side) the spans are
halved the way it fails, to 32 each way. Corners whose neighbors disagree (normals apart, a boundary
leaving the other's tangent plane, the twists II_a(t_a, t_b) and II_b(t_a, t_b) apart, G2 neighbors'
curvatures apart) are found first: their G1 and G2 rows are eased to nothing within 3% of each side and
they are reported; failures left at the end are reported as runs along the boundaries. Each boundary's
worst gap is taken against the fill's nearest points, ends included.

Tests (`tests/regressions/fill_tests.luc`), against closed forms and the Check tools: a planar pentagon
fills with its plane to 1e-9 and a square hole in a tilted plane with the plane, G1 and G2, N-sided and
Quad (one untrimmed face of four edges); a square hole in a smooth height field fills G0, G1 and G2 on all
four edges, N-sided and Quad, quintic, with no conflicts; a 30° cap cut from a ball of radius 2 fills G1 and
G2 within a thousandth of the radius of the sphere, measured G1 and G2 on its edge; the height field's
hole is found from one edge; a five-sided corner between three G2 fillets and two walls fills G1 on all
five edges, and with circular fillets the corners where a fillet meets a wall's strip are reported (their
twists disagree); a guide curve and a point lie on the fill (N-sided and Quad) with G1 kept; Flatness and
Tension lower a cap's dome, Explicit Control's degrees and spans are kept, and Normal flow leaves the
cross isoparms square to the side.

Not yet: Molded domains (a boundary folding over its best plane is refused, Quad is the way round);
the reference's ribbons for B-spline neighbors beyond their domain are tangent planes (a natural
extension would carry their curvature); G2 at corners whose neighbors' curvature rates disagree (a
G2 fillet's end meeting a wall at an angle) is not detected before the solve: what fails is reported as
runs.

## Sketches (`sketch/`)

A sketch is points, lines, circles and arcs on a plane, and the constraints between them, as in
Fusion 360. `CadSketch` holds one for the editor: it reads and writes the sketch's text (what a
Sketch node keeps), adds and removes entities and constraints, edits dimensions, solves, drags a
point, picks what is near a point or held by a box (`select_box`: a screen box cast onto the plane,
any convex polygon there; what lies entirely inside it, points and whole curves, `boxing.lucb`), and gives
the curves Extrude and Revolve read. Lengths are millimeters, angles radians.
`docs/research/SKETCH-SOLVER-STUDY.md` has the design, after SolveSpace's and FreeCAD's solvers.

- **Parameters.** One array: a point owns x and y; a circle or arc owns its radius, and an arc's
  ends are real points that two rows keep on its circle. No angles are unknowns, so nothing wraps.
- **Constraints.** Coincident, horizontal, vertical, concentric, equal radii, fix, radius and
  diameter only say parameters are equal or constant: they merge parameters into classes
  (union-find, `substitution.lucb`) and are never solved for. Two different constants meeting
  in a class conflict at once. Point on curve, parallel, perpendicular, angle, equal length,
  tangent (line–circle, circle–circle, inside or out), midpoint, collinear, symmetry and the
  distance dimensions are rows (`residuals.lucb`): each computes its residual and every partial in
  one pass; ratio and angle rows are scaled by the sketch's size so all rows are lengths.
- **Solving** (`solver.lucb`). Unknowns tied by rows form components, each solved alone. A step is
  the least motion that satisfies the rows to first order, weighted so a dragged point hardly
  moves: (J W⁻¹ Jᵀ + μI) z = F, step −W⁻¹ Jᵀ z, Gauss–Newton with Levenberg damping only when a
  step does not help. An under-constrained sketch so moves as little as it can; one that does not
  converge is left as it was. A drag passes its target apart from the parameters, so a fixed point
  stays fixed.
- **Freedom** (`freedom.lucb`), after a change rather than per drag frame: one column-pivoted QR of
  Jᵀ gives the rank (degrees of freedom left), the null space (which entities are fully
  constrained, for coloring) and the rows that depend on others (redundant when they hold,
  conflicting when they do not).
- **Curves** (`curves.lucb`): lines as two-point polylines, circles and arcs as exact rational
  quadratic arcs, on the plane given by an origin and the sketch's x and y axes. Construction
  entities make none.

- **Modify** (`modify.lucb`), Fusion's MODIFY tools: Trim takes away the piece of a curve between
  the nearest crossings round where it is picked (the whole curve when nothing crosses it; a
  circle becomes the arc left); Extend runs a line's or arc's nearest end on to the next curve in its
  way; Break splits a curve at the crossings round the pick. A new end lies on the curve it was cut at
  (point-on-curve), a piece split off a line stays collinear, one off an arc keeps its circle.
  Fillet rounds the corner of two lines sharing a point (an arc tangent to both, its radius
  dimensioned, the lines shortened); Chamfer cuts it. Offset copies a chain of lines and arcs to one
  side, the copies meeting where they cross (lines kept parallel). Move/Copy and Scale act on
  entities (Scale scales their dimensions too). Ends left over are reused, not removed, so only
  trimming a curve away whole renumbers entities.

- **Projection** (`projection.lucb`, `face_projection.lucb`), Fusion's Project and its
  auto-projection of a sketch's face. `CadSketch.project(projection, edge)` casts one body edge;
  `project_face(model, face, origin, u, v)` casts the outer and inner loops of the faces going by
  that name, one fixed curve per edge (a line, circle or arc when the shadow is one, else a fit
  point spline), neighbors sharing their sketch points, so each loop is one closed chain: a
  profile, and a chain Offset takes whole. `project_end(projection, edge, end)` casts a body
  vertex as a fixed point. Every projected curve and point is linked to its edge's reference
  (`references.lucb`), and the face is remembered by name. `relink(model, origin, u, v)` casts
  them again from the body as it is now:
  - each curve's ends move first, from where they were: a point several curves share goes where
    the first puts it, and a curve that would put it elsewhere gets a point of its own;
  - an edge split upstream is followed by its piece nearest where it was;
  - a link whose edge is gone, or is now another kind of curve, is lost: it stays as it was and
    is counted (a warning);
  - edges a projected face's loops gained (a notch cut into it) are projected too, `gained()`
    of them.

  Lost links are named: `lost_link(at)` after a relink. `link_to(at, reference)` points a curve
  at another edge (Re-link), `break_link(at)` makes it a plain curve (Break Link), `linked(at)`
  and `projects_face(name)` say what is projected. A cast can travel without its body
  (`shadows.lucb`): `CadShadows.text_of(projection)` writes every edge's samples and reference
  where the body is (the editor's compute worker), `CadShadows.parse` reads them where it is
  not, and `project_shadow(shadows, edge)` projects one, linked as `project` would.
- **Offset relation.** `offset(curves, distance, x, y, relation = true)` holds the whole chain's
  copies by one constraint (`ConstraintKind.offset`): one value, one dimension. Its first pair is
  the constraint's own curves; the others are the sketch's `offsets`, grouped by the first copy.
  A line copy stays parallel to its source with its start at the distance; an arc or circle copy
  shares its source's center, its radius the source's plus or minus the distance. Corners follow
  because neighboring copies share their meeting points. Free-form copies (splines, ellipses,
  conics) are fit point splines outside the relation. Removing the constraint removes the
  relation. Without `relation`, each line copy gets its own parallel and distance (what the
  editor uses until Stage 3 moves it over).
- **Polygon** (`ConstraintKind.polygon`, `add_polygon(center, corners, inscribed)`), Fusion's: a
  regular polygon held by one constraint about its one construction circle. Its sides are lines
  made in a row (the constraint names the circle and the first and last side; a chain each starting
  where the one before ends). Inscribed, its corners lie on the circle; circumscribed, its sides
  touch the circle at their middles, so its corners lie on the circle's radius over cos(π/n): the
  row is the corner's distance from the center against the radius plus (sec(π/n) − 1) times it.
  With every side as long as the first, that is 2n − 1 rows for the 2n − 1 freedoms a regular
  polygon's corners lose; the circle's radius alone sizes it (a radius dimension is Fusion's), and
  centered and sized it keeps one freedom, its turn. Removing any side removes the constraint.
- **The origin** (`entities.lucb` `origin_entity`), Fusion's origin point and axes in every
  sketch: `origin(which)` gives the point (0) or the x (1) or y (2) axis as an entity, made when
  first asked for: the point fixed at (0, 0); an axis a construction line from it to a fixed point
  a millimeter along (only its direction counts). Each carries its part (`SketchEntity.origin`;
  the text's `r` lines), `origin_part(at)` reads it. They are references, not curves: never
  picked, snapped to or boxed as themselves (the editor draws and picks them across the view),
  never a crossing that trims, never a profile.
- **Sweep** (`ConstraintKind.sweep`): an arc's sweep held at an angle, the angle row on its
  center's rays to its ends (wrapped, so past half a turn holds), drawn as an angle at the center
  inside the arc with extension lines out along the rays.
- **What an over-constraint runs through** (`freedom.lucb` `dependent_constraints`,
  `find_dependent`, `dependent(at)`): each constraint that, made driven in a copy, leaves fewer
  constraints redundant or conflicting. The editor lights them when a new one over-constrains.
- **Project of a face and of a body** (`shadow_faces.lucb`), from shadows carried without the
  body (`shadows.lucb` now carries each edge's height off the plane and each face's name and
  edges): `CadShadows.face_at(x, y, toward)` is the face whose shadow holds a point, the one
  highest toward the camera where several do; `project_shadow_face(shadows, face)` brings every
  edge of its loops, neighbors sharing points by place, the face remembered by name so relinking
  adds edges it gains; `project_shadow_outline(shadows, toward)` brings the body's outline: the
  edges whose shadow bounds the union of the faces' shadows, one of each pair casting the same
  shadow (the higher). Silhouettes are brought too, as Fusion's Project brings them
  (`silhouettes.lucb`): where a curved face turns away along the plane's normal (a cylinder
  seen from its side, a ball's rim), found on the body's display mesh (n·N changing sign across
  a triangle's side, the crossings chained) and each sample moved onto the exact silhouette.
  Which shadows bound the outline is told where the body is, on that mesh (one side of an edge
  in a triangle's shadow, the other in none; a silhouette's outer side clear), and carried in
  the text. A silhouette is linked to `<face>~x,y,z`, its face and a point of it; relinking
  takes that face's silhouette nearest the point. A rim seen edge-on (its shadow turning back
  along a line) casts the line it covers; a seam (a face meeting itself) is no edge to Fusion
  and never bounds the outline. An edge already projected is not brought twice.
- **Dimensions drawn** (`dimensions.lucb`), as drawings draw them. `lay_out(at, pixel)` gives a
  dimension's lines, arrowheads and label place on the plane, with screen sizes (gaps,
  overshoots, the default offset) in multiples of `pixel`, a screen point's length there:
  - a length: extension lines square to a dimension line through the label, with an arrow at
    each end (outside, pointing in, when the ends are close on screen); a point's distance from
    a line starts its extension on the line;
  - a radius or diameter: a line through the center at the label's angle, arrows on the curve;
  - an angle: an arc between the lines' rays, with extension lines along them when it lies past
    their segments; an Offset relation is a length across the copy and its source.

  `place_label(at, x, y)` keeps where a label was put as two numbers measured from what the
  dimension measures (`SketchConstraint.label_a`, `label_b`; the text's `g` lines), so it moves
  with the geometry. An unplaced label sits a little off the geometry, outward of the sketch's
  middle; `lay_out(at, pixel, step)` puts it `step` places on (a length's on the other side, then a
  row further out on each; a radius's round its circle; an angle's arc wider), which the editor
  tries when another label is in the way. A placed label stays.

Not yet: ellipses, splines, slots and text; curvature (G2) constraints; Blend Curve.

## Limits, and what comes next

Roughly in order of usefulness:

1. **Fillet corners beyond three edges** where no one ball touches every face (an irregular apex;
   Setback and G2/G3 corners of four or more), and three unequal radii at a corner.
2. **Variable radius fillets meeting other fillets at a corner** of three edges within one
   feature, and full rounds of curved ribs whose ends are not flat through the axis.
3. **Blends on other curved edges:** a flat face meeting a curved one that is not ruled away from
   it (beyond rims, ruled walls and sweeps); open chains between two curved faces; open ends where
   the loop turns a sharp corner into an end wall that is not a plane square to the edge (Fusion
   builds these: its strip carried on and trimmed by the wall); an arc of a dome's rim alone
   filleted.
4. **Variable radius laws Fusion shapes its own way**: a piece from an open end into a flat run, a
   steep piece beside a shallow one, and the law round a closed loop between its extremes (not
   decoded; Variable radius above). A G1 ball rolled along an open chain (at weight 1 ours scales
   the section there: 0.09 off Fusion on the plate chain), and round a loop whose radius comes near
   the loop's own curvature (refused with that reason). Whether Fusion measures a single edge's or a
   closed loop's law along the balls' path too, as it does an open chain's, is not checked (the
   laws read back on a box's edge fit its edge's length).
5. **Sphere faces with more than two loops going round every axis** away from the trims (a
   face whose loops do not make one band about any direction): skipped and named on the node.
   Every other sphere face meshes: holes all round a ball and through its poles, half a ball
   whose rim runs round them (`meshing/sphere_axis.lucb`).
6. **Sweep and loft as solid features** (their sheets are Stage 5's Rail and Skin; capped into
   solids, and Fusion's Sweep and Loft dialogs on sketches).
7. **Profiles on faces:** a sketch's plane from a picked face, kept by name.
8. **Non-manifold results:** bodies touching along a line or a face (a box and a pipe lying on
   it, joined; a pipe inside a box touching its wall, subtracted; a pipe grazing a box's top and
   bottom, taken from it) are refused as non-manifold.
9. **Offsets where faces vanish or reconnect** (a shell thicker than a fillet's radius), B-spline
   offsets of other than profiles swept straight past their surfaces' ends (no extension yet), and
   draft; Delete healing several faces at once beside curved faces
   beyond a band round one face (strips apart, inside corners).
10. **Needles where shared samples crowd:** a grid line of one face crossing a shared edge a hair
   from one of its samples becomes a station beside it, and both faces keep a needle there (the
   walls and floors of holes drilled in a ball). Sphere and other curved faces are otherwise
   graded to their trims and shaped on the surface (`meshing/chart_delaunay.lucb`).
