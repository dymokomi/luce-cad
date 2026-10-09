# CAD Modeling Study: Fusion-style features on our own B-rep kernel

Research notes, 2026-10-07. They cover what to build, what to learn from, and in what order. Open-source code is
listed for reference only. We write our own code.

Starting point: luce-cad already has the B-rep data model (analytic plane, cylinder, cone, sphere and torus supports,
plus rational B-splines, trimmed by edge loops with shared-edge topology). It also has a trimmed-surface tessellator,
STEP import, analytic primitive solids and planar sketch curves. It has no modeling operations yet.

---

## 0. Executive summary

* **The order of work matters more than which algorithm we pick.** About 80% of real Fusion parts are sketches on planes
  or planar faces, extruded or revolved, then Join or Cut, then chamfered or filleted. Almost every intersection in
  those parts involves only planes, cylinders and cones, and usually their axes are parallel or perpendicular to the
  sketch plane. We should build analytic intersection cases first, add a single general marching fallback, and
  postpone NURBS-against-NURBS work.
* **Booleans should follow the "general fuse" structure** (OCCT `BOPAlgo`). The steps are: intersect everything in
  order of dimension (V/V, V/E, E/E, V/F, E/F, F/F), split edges and faces, classify the pieces, then select and
  sew the result. Coplanar ("same-domain") faces must work from day one, because sketching on a face and
  extruding Join produces them on every single part.
* **Robustness comes from tolerant modeling.** We should not chase exact arithmetic for curved geometry. Use a global
  resolution, give each edge and vertex its own tolerance where an intersection had to be approximated, and use
  exact or adaptive predicates only for the cheap linear decisions (orientation and side tests). This is what
  Parasolid (Jackson 1995) and OCCT do.
* **Fillets should be built as local operations with an analytic special-case table** (OCCT `ChFiKPart`).
  Plane/plane gives a cylinder, plane/cylinder gives a torus, and so on. The general case uses a rolling-ball
  spine (the intersection of the two offset surfaces), sampled and approximated as a NURBS. Chamfers use the same
  machinery with a simpler cross-section, so they should come first.
* **Topological naming must be designed into every operation from the start.** Each operation returns a history
  map (generated, modified, deleted), and each face and edge carries a generative name: feature id, plus a role
  in the operation, plus the names of its sources. Downstream references are stored as queries (Onshape-style),
  with a geometric fingerprint as a fallback. Failures are shown in the timeline and never guessed silently.
* **Plasticity's main lesson is about UX, not the kernel.** Every command is a live, dimension-driven gizmo with an
  immediate preview, and the user commits when done. Offset Face and Push/Pull work on any face. Fillets get
  variable-radius points from a single key. We can offer the same feel inside a parametric node: each node is a
  "factory" that stays editable.

---

## 1. Fusion 360

Sources: Fusion help ([Fillet](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-FILLET-SOLID.htm),
the Model workspace docs next to it), the Fusion API reference
([FilletFeatureInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/FilletFeatureInput.htm),
[BRepFace.entityToken](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/BRepFace_entityToken.htm)), and
Autodesk forum threads on lost references. Fusion runs on Autodesk Shape Manager (ASM), a fork of ACIS. The sketch
solver and the naming internals are not documented publicly, so the parts below that describe internals are
marked as inference.

### 1.1 Sketch workflow

* **Planes.** A sketch is created on an origin plane (XY, YZ or XZ), a construction plane (offset, angle,
  tangent, midplane, through 3 points, and so on) or any planar face. A sketch on a face keeps an associative
  reference to that face and is re-attached when the face moves. This is the most common naming dependency in
  real models.
* **Geometry.** Line, rectangle (2-point, 3-point, center), circle, arc, polygon, ellipse, slot, spline (fit
  point and control point), conic, point and text. Every curve can be a normal curve or a construction curve.
  Other tools: Project, Include, Intersect (body edges into the sketch), Offset, Mirror, Pattern, Trim, Extend,
  Break and Fillet (sketch fillet).
* **Profiles and regions are detected automatically.** All closed regions formed by the curve arrangement
  (intersections included) become selectable "profiles". Hovering a region highlights it. Overlapping circles
  produce multiple lens and crescent regions, and the user may pick any subset. Construction curves don't take
  part. Open curves that cross a closed region split it. Planar faces of bodies can also be picked as profiles.
  *Implementation implication:* we need a 2D curve arrangement (intersect all curves, split, build a planar
  half-edge graph, extract the minimal cycles, nest holes into outer loops). Each region must be identified by
  the sketch curve ids that bound it, so that "profile 3" survives edits.
* **Constraints.** Coincident, collinear, concentric, midpoint, fix, parallel, perpendicular, horizontal,
  vertical, tangent, smooth (G2), equal, symmetric and curvature. Constraints are inferred while drawing (snap
  to endpoint, horizontal and so on). **Dimensions** are driving and accept expressions over named user
  parameters (the *Change Parameters* table), which is how Fusion models become "configurable". A fully
  constrained curve turns black, and an under-constrained curve stays blue. Over-constraining is refused, with an
  offer to make the dimension driven.
* **Solver (inference).** Fusion uses a variational 2D geometric constraint solver of the D-Cubed / SolveSpace
  family. Constraints become equations, the system is decomposed, then solved with Newton or least squares
  starting from the current geometry (which keeps it close to what the user drew). Rank analysis reports
  degrees of freedom and redundant or conflicting constraints. Open references we can actually read:
  * SolveSpace: [`src/system.cpp`](https://github.com/solvespace/solvespace/blob/master/src/system.cpp)
    (Newton plus least squares, Jacobian rank for "redundant/failed" constraints, substitution of trivial
    equalities) and [`src/constrainteq.cpp`](https://github.com/solvespace/solvespace/blob/master/src/constrainteq.cpp)
    (constraint-to-equation generation).
  * FreeCAD planegcs: [`src/Mod/Sketcher/App/planegcs/`](https://github.com/FreeCAD/FreeCAD/tree/main/src/Mod/Sketcher/App/planegcs)
    (`GCS.cpp`: DogLeg, Levenberg–Marquardt and BFGS, QR-based detection of redundant and conflicting
    constraints, subsystem partitioning).

### 1.2 Solid features users rely on

| Feature | Key options | Kernel needs |
|---|---|---|
| **Extrude** | Profiles (sketch regions or planar faces). Start: profile plane, offset, from object. Direction: one side, two sides (independent extents), symmetric. Extent: distance, *To Object* (face, body or vertex, with an offset), All. **Taper angle** per side. **Thin** extrude (wall thickness, inside, outside or center). **Operation**: Join, Cut, Intersect, New Body, New Component, with "objects to cut" (participating bodies). | Prism sweep of the profile, then a Boolean. Taper turns a line into a plane, an arc into a cone and a spline into an offset/draft surface. To Object needs ray or face intersection, or a "sweep then boolean with a half-space". |
| **Revolve** | Profile, axis (sketch line, edge or origin axis), angle, full, to object, one side, two sides or symmetric, same operations as Extrude. | Revolution sweep: line to plane, cylinder or cone; arc to torus or sphere; spline to rational surface of revolution. |
| **Sweep** | Single path, path plus guide rail, or path plus guide surface. Orientation: perpendicular or parallel. Distance, taper and twist. | Frenet or rotation-minimizing frames, skinning, mostly NURBS output. |
| **Loft** | Ordered profiles (points allowed at the ends), rails or a centerline. End conditions: free, direction, tangent (G1), smooth (G2). Closed loft. | Compatible NURBS skinning, with section alignment and twist control. |
| **Fillet** | Constant, **chord length**, **asymmetric** or **variable** radius (multiple points along an edge). Edges, faces and features as input, with tangent-chain propagation. **Corner type: Rolling Ball or Setback**. **Continuity: G1 tangent or G2 curvature**. *Rule Fillet* (all edges between feature or body sets) and *Full Round Fillet* (three faces). | Rolling-ball blend, corner (vertex) blends, edge-chain propagation, face removal when a fillet consumes a face. |
| **Chamfer** | Equal distance, two distances, distance plus angle. Tangent chain. Corner type: chamfer, miter or blend. | Same framework as fillet, with a ruled cross-section. |
| **Shell** | Faces to remove, inside, outside or both thickness, tangent chain. | Offset every face, re-intersect the neighbors, handle self-intersection. This is the hardest of the common features. |
| **Combine** | Target body, tool bodies, Join, Cut or Intersect, keep tools, new component. | Booleans. |
| **Mirror / Pattern** | Mirror faces, bodies, features or components about a plane. Rectangular, circular or on-path pattern. Compute option: Optimized, Identical or Adjust (Adjust re-evaluates each instance's extents, so a "to object" extent recomputes per instance). | Transform plus Boolean. Feature patterns re-run the feature at each instance. |
| **Draft** | Pull direction (plane), faces, angle, one side, two sides or symmetric. | Replace face (rotate each face about its neutral curve), re-intersect the neighbors. |
| **Press/Pull** | Context sensitive: a face becomes Offset Face, an edge becomes Fillet, a profile becomes Extrude. | Offset or replace face, then local re-intersection. |
| Others | Hole (simple, counterbore, countersink, tapped), Thread (cosmetic or modeled), Rib, Web, Emboss, Boundary Fill, Thicken, Offset Face, Replace Face, Split Face, Split Body, Move/Copy, Scale. | Mostly combinations of the operations above. |

### 1.3 Timeline and parametric history

* The timeline is an ordered list of features. Each feature stores its inputs: parameters, selections stored
  as references, and operation and target bodies. Recompute replays the features from the first dirty one.
* The user can drag the **rollback marker** (insert features earlier), **reorder** features (if dependencies
  allow), **suppress** features, **group** them, and **edit** any feature in place. Components each have their
  own timeline. A *Base Feature* container holds direct-modeling edits inside a parametric design. The
  "capture design history" option can be turned off for pure direct modeling.
* **Failure reporting:** a feature turns yellow (warning, for example "references were lost, using cached
  geometry", or "Pattern source lost") or red (compute failed). The user repairs it by editing the feature and
  re-selecting. Forum threads ([example](https://forums.autodesk.com/t5/fusion-support-forum/references-repeatedly-lost/m-p/13203525))
  show this happens in practice even for features nobody touched. Lost references sometimes appear only after
  a full re-compute.

### 1.4 How Fusion handles the topological naming problem (what is observable)

* Selections are stored as kernel-level persistent identifiers, not as indices. The API shows this as
  `entityToken` (`Design.findEntityByToken`). The docs state that a token string may change over time while
  still resolving to the same entity, so tokens must not be compared as strings. The API also exposes an
  `associativeID` that Fusion "uses as the identifier for the face … for tracking this geometry for parametric
  recomputes" (Fusion API, BRepFaceDefinition). So identity lives in *attributes attached to topology* that
  the kernel carries through operations.
* ACIS and ASM attributes have documented split, merge, copy and transform behavior. When a face is split by a
  boolean, the attribute is copied to both halves. When two faces merge, a merge policy decides the result.
  Fusion builds feature-scoped identities on top of that (inference: "face created by Extrude1 from sketch
  curve X, side face").
* When the identity resolves to nothing (the face was consumed) or to several entities (it was split), Fusion
  either picks the one it judges best and warns, or fails the feature and asks for re-selection. There is no
  general geometric re-matching visible to the user.

---

## 2. Plasticity (plasticity.xyz)

Sources: [product tour](https://plasticity.xyz/product), [manual](https://doc.plasticity.xyz/),
[Fillet/Shell docs](https://doc.plasticity.xyz/solid/fillet-shell), CG Channel release coverage
([1.3](https://www.cgchannel.com/2023/10/nick-kallen-ships-plasticity-1-3/),
[2024.1](https://www.cgchannel.com/2024/05/plastic-software-releases-plasticity-2024-1)),
[Wikipedia](https://en.wikipedia.org/wiki/Plasticity_(software)).

* **What it is.** "CAD for artists": a NURBS and solid direct modeler by Nick Kallen, positioned against MoI and
  Fusion for concept and hard-surface artists, with Blender and Cinema 4D bridges. It was released in 2023 on
  the **C3D** kernel and moved to **Parasolid** (plus xNURBS for G2 surface filling in the Studio tier) around
  2024–2025 ([Onshape forum note](https://forum.onshape.com/discussion/27002/plasticity-2025-1)). It is a
  direct modeler: no feature tree, no constraint sketcher. Its robustness comes entirely from Parasolid.
* **Feature set.** Booleans (union, difference, intersect, plus "Cut" of a solid by curves or a sheet).
  Extrude, revolve, loft, sweep, pipe, patch. **Fillet** with constant or variable radius (press **V** to "add
  variable point" on the edge and drag it), profile shapes **Round, Conic, Chordal, G2, Full** (tangent to
  three faces), **Y-blend** corner option. **Chamfer** (Offset or Apex shape). **Fillet Shell**, Hollow,
  Thicken. **Offset Face**, **Offset Face Loop**, **Draft Face**, **Delete Face** (heals the hole), **Push/Pull**
  of any face, **Extend Sheet**, **Imprint**, **Offset Curve**. Bridge curves, trim, control-point and fit
  splines, curvature combs, continuity inspection, section analysis, linear, radial and grid arrays.
  PolySplines convert subdivision-style meshes to G2 NURBS patches.
* **UX ideas worth adopting:**
  1. **Every command is a live gizmo.** You select, a manipulator appears, you drag or type a number (D for
     distance, C for chamfer…), the preview updates continuously, and the action commits on confirm. You never
     go through a modal dialog first. In our node app the node's parameter panel *is* this, plus a viewport
     manipulator bound to the parameter.
  2. **Selection is the command's input, and the command is inferred** (as in Fusion's Press/Pull):
     select an edge and drag to get a fillet; select a face and drag to get an offset; select a region and drag
     to get an extrude. Combined with the owner's "remove repeated typing" principle, this is the fastest
     loop.
  3. **Boolean-by-default extrude.** Dragging a region off a solid face auto-joins, and dragging into it
     auto-cuts. Fusion does the same, picking New Body, Join or Cut by direction.
  4. **Offset Face and Delete Face on any face**, in history, as "local operations". These need the same
     replace-face plus re-intersect-neighbors machinery as Draft and Shell.
  5. Fillet variable points placed directly on the edge, and **fillet profile choice** (round, chordal, conic,
     G2). Users repeatedly praise "fillets that just work" on messy geometry.
  6. A command palette, a radial menu, snapping with tangent and midpoint guides.
* **History.** Plasticity is fundamentally history-free. Edits are "factories" that stay live until you commit,
  plus undo. Users praise the speed and the lack of a fragile tree. The cost is that you cannot go back and
  change a sketch dimension. For us the lesson is: **a parametric timeline is fine if the evaluation is fast
  and incremental, and if failures are local and repairable.** Keep direct-edit nodes (Offset Face, Move Face,
  Delete Face) available inside the history, the way Fusion's Base Feature and Onshape's direct-edit features
  are.
* **What users praise** (forums, reviews): speed of interaction, booleans and fillets that rarely fail
  (Parasolid), clean artist-friendly UI, a perpetual license, and good mesh and Blender export. Common
  complaints: no parametric history, no constraints, limited drawings.

---

## 3. Open-source kernels and literature to learn from

### 3.1 OpenCASCADE (OCCT)

Repository layout (current master): `src/ModelingAlgorithms/...`.

**Booleans: the General Fuse Algorithm (GFA)**

* Code: [`src/ModelingAlgorithms/TKBO/`](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKBO).
  * `BOPAlgo/BOPAlgo_PaveFiller*.cxx`: the intersection part, one file per interference type (`_1` V/V, `_2`
    V/E, `_3` E/E, `_4` V/F, `_5` E/F, `_6` F/F, `_7` building section edges, and so on).
  * `BOPDS/`: the data structure: paves, pave blocks, common blocks, interference tables, face info.
  * `BOPAlgo/BOPAlgo_Builder*.cxx`: the building part (split edges, faces and solids). `BOPAlgo_BuilderFace`
    builds faces from a "soup" of edges on one surface. `BOPAlgo_BuilderSolid` builds solids from faces.
    `BOPAlgo_BOP.cxx` selects the pieces for a Boolean.
  * `IntTools/IntTools_FaceFace.cxx`: face/face intersection, which dispatches into `IntPatch`.
  * `BRepAlgoAPI/`: the user API (`BRepAlgoAPI_Fuse/Cut/Common/Section`).
* Specification: [Boolean Operations spec](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html).
* **Outline.** (1) Bounding-box filter on all sub-shape pairs. (2) Compute interferences in order of increasing
  dimension: V/V, V/E, E/E, V/F, E/F, F/F. Each step adds **paves** (vertex plus parameter on an edge). Edges
  are cut into **pave blocks**, and pave blocks that coincide across edges or faces become **common blocks**
  (shared split edges). F/F produces **section curves**, which are approximated, given p-curves on both faces,
  and then cut by paves too. (3) The building part: split edges from the pave blocks; split faces by putting
  each face's original edges plus its section edges into `BuilderFace` (2D loop finding in the face's UV
  domain); detect **same-domain faces** (coincident faces from different arguments) and keep one
  representative; build split solids. (4) The Boolean is a *selection* from the GF result, by classifying each
  split face as IN, OUT or ON with respect to the other argument (ray or point classification, with face-state
  propagation through connected faces).
* **Robustness strategies:** every vertex and edge carries its own tolerance, which is grown when an
  intersection is only approximately on both surfaces (`CorrectPointOnCurve`, `CorrectCurveOnSurface`). A
  **fuzzy value** adds tolerance for nearly coincident input. **Gluing** modes cover inputs with shared or
  coincident faces. Results of one stage are reused topologically and never recomputed, so a vertex is decided
  once. Documented limits: arguments must not self-intersect, and the geometry should be at least C1.
* **Lessons for us:** the pave/pave-block/common-block bookkeeping is the core idea. It guarantees one shared
  split edge where two faces meet, which is exactly what avoids cracks. The F/F step is where the
  surface-surface intersection (SSI) quality matters. Everything else is combinatorics.

**Surface-surface intersection**

* [`src/ModelingAlgorithms/TKGeomAlgo/IntPatch/`](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKGeomAlgo):
  * `IntPatch_ImpImpIntersection*`: quadric/quadric, analytic. Plane/plane, plane/quadric and
    cylinder/cylinder (with parallel and perpendicular sub-cases). The general quadric pair is handled
    semi-analytically.
  * `IntPatch_ImpPrmIntersection`: quadric/parametric. It marches on the implicit function.
  * `IntPatch_PrmPrmIntersection`: parametric/parametric. Start points come from a polyhedral pre-intersection
    (`IntPolyh`), followed by marching (`IntWalk`).
  * `GeomInt`: approximation of the walking lines into B-splines plus p-curves (`GeomInt_IntSS`).

**Fillets and chamfers**

* [`src/ModelingAlgorithms/TKFillet/`](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKFillet):
  * `BRepFilletAPI/BRepFilletAPI_MakeFillet.cxx`, `BRepFilletAPI_MakeChamfer.cxx`: the API.
  * `ChFi3d/ChFi3d_Builder*.cxx`: the generic engine. It (1) builds **stripes** (tangent-continuous edge
    chains, the "spine"); (2) computes the blend surface along each stripe; (3) computes **corners** at
    vertices (`ChFi3d_Builder_C1/C2`: one, two or three stripes meeting); (4) reconstructs the topology by
    trimming the neighboring faces along the contact curves and inserting the blend faces.
  * `ChFi3d_FilBuilder.cxx` (fillets), `ChFi3d_ChBuilder.cxx` (chamfers, including two-distance and
    distance-angle).
  * **`ChFiKPart/`: analytic special cases.** `ChFiKPart_ComputeData_FilPlnPln` (plane/plane gives a
    cylinder), `_FilPlnCyl` (gives a torus or cylinder), `_FilPlnCon`, `_ChPlnPln`, `_ChPlnCyl`, `_ChPlnCon`,
    `_ChAsymPln*`, `_Rotule` (sphere corner). **This table is the first thing we should copy conceptually.**
  * `BlendFunc/` (the rolling-ball equations: constant radius, evolutive radius, chamfer, chord) and
    `Blend/`, `BRepBlend/` (walking along the spine, `BRepBlend_Walking`), with `AppBlend` approximating the
    result as a B-spline surface.
* **Rolling-ball outline.** For faces S1 and S2 and radius r: the ball center path C(t) is the intersection of
  the offset surfaces S1 + r·n1 and S2 + r·n2 (signs depend on convexity). The contact curves are the
  projections of C(t) back onto S1 and S2. The fillet surface is the family of circular arcs in the plane normal
  to C'(t), from contact 1 to contact 2. Variable radius makes r = r(t), a law along the spine. Chord-length
  and chamfer variants change the cross-section constraint. The walk is a Newton-corrected predictor-corrector
  on 4 unknowns (u1, v1, u2, v2) per section.

**Extrude (prism)**

* [`src/ModelingAlgorithms/TKPrim/`](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKPrim):
  `BRepPrimAPI/BRepPrimAPI_MakePrism.cxx` and `BRepPrimAPI_MakeRevol.cxx` on top of
  `BRepSweep/BRepSweep_Prism.cxx` and `BRepSweep_Rotation.cxx`, using the generic `Sweep/` topology builder.
  The topology rule: generating shape × direction, so a vertex becomes an edge, an edge becomes a face and a
  face becomes a solid. Side surfaces are `Geom_SurfaceOfLinearExtrusion`, specialized to plane or cylinder by
  the caller when possible. `MakePrism` keeps a full history (`Generated`, `FirstShape`, `LastShape`), which
  is exactly what naming needs. Feature-level "extrude to face" lives in
  `TKFeat/BRepFeat/BRepFeat_MakePrism.cxx` (with boolean fusion).
* **Shape history:** `BRepBuilderAPI_MakeShape::Generated/Modified/IsDeleted` (TKTopAlgo). Every algorithm
  reports it, and FreeCAD's naming is built on it.

**What's practical for a small team:** OCCT's design is right, but its code is huge and full of special
cases. Copy the *architecture*: pave filler, builder face, same-domain handling, the KPart table and the
history API. Write lean versions of each.

### 3.2 SolveSpace (NURBS booleans in `src/srf/`)

* Code: [`src/srf/`](https://github.com/solvespace/solvespace/tree/master/src/srf):
  [`surface.h`](https://github.com/solvespace/solvespace/blob/master/src/srf/surface.h),
  [`boolean.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/boolean.cpp),
  [`surfinter.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/surfinter.cpp),
  [`raycast.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/raycast.cpp),
  [`ratpoly.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/ratpoly.cpp),
  [`curve.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/curve.cpp),
  [`merge.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/merge.cpp),
  [`triangulate.cpp`](https://github.com/solvespace/solvespace/blob/master/src/srf/triangulate.cpp).
* **Data model:** `SSurface` is a single **rational Bézier patch of degree ≤ 3** (4×4 control points plus
  weights), not multi-span NURBS. Trim curves (`SCurve`) carry an optional exact `SBezier` (`isExact`) plus a
  **piecewise-linear polyline** (`pts`), which is what the boolean actually uses. Surfaces come only from
  extrusion, revolution or helix of sketch curves, so the set of surface pairs is small.
* **Boolean outline** (`SShell::MakeFromBoolean`):
  1. `CopyCurvesSplitAgainst`: split every existing trim curve where it crosses the other shell's
     surfaces, so that T-junctions don't appear.
  2. `MakeIntersectionCurvesAgainst`: SSI for every surface pair (`SSurface::IntersectAgainst`).
  3. `CopySurfacesTrimAgainst` / `MakeCopyTrimAgainst`: for each surface, gather its old trims plus the new
     intersection curves as a 2D (UV) edge list. Classify each edge by probing just to either side of it
     against the other shell (`ClassifyEdge`, giving `SURF_INSIDE`, `SURF_OUTSIDE`, `SURF_COINC_SAME` or
     `SURF_COINC_OPP`). Keep edges per `KeepEdge` or `KeepRegion` for union, difference or intersection, then
     re-assemble the loops. Tangent surfaces: add the edge in both directions and let classification keep the
     right one.
* **SSI** (`surfinter.cpp`): exact special cases first: plane/plane (line), plane/extrusion with the
  direction parallel to the plane (lines), plane/extrusion otherwise (the projection of the extruded curve),
  extrusion/extrusion with the same direction (lines), and coincident surfaces (reuse the trims). Otherwise a
  general **marching** method. Start points come from intersecting each surface's boundary with the other
  surface. Each step is Newton-refined (`ClosestPointOnThisAndSurface`) and stepped by chord tolerance. Curves
  found entirely outside either trimmed region are discarded.
* **Why it's simple:** restricted surface classes, polyline trims, classification done by local probing with
  a chord tolerance, and a mesh fallback. About 5k lines total.
* **Limitations:** fails, or leaves "naked edges", on tangent or near-tangent surfaces, on coincident curved
  faces and on high-degree or multi-span NURBS. Its accuracy is bounded by the chord tolerance, because trims
  are polylines, so the result is not an exact B-rep. It cannot do fillets or chamfers on solids. It cannot
  fix loops missed by marching (it has no loop detection). It sets a `booleanFailed` flag and the user sees
  the error.
* **Lesson:** this is the **minimum viable NURBS boolean** and a good proof that exact special cases plus one
  marching fallback go a long way. We should keep exact trim curves (we already have them), and use polylines
  only for classification probes.

### 3.3 truck (Rust): `truck-shapeops`

* Code: [`truck-shapeops/src/`](https://github.com/ricosjp/truck/tree/master/truck-shapeops/src):
  `transversal/{polyline_construction, intersection_curve, loops_store, divide_face, faces_classification, integrate}`,
  `fillet/`, `healing/`.
* **Outline** (`transversal/integrate/mod.rs`):
  1. Triangulate both shells at a tolerance.
  2. Intersect the meshes to get intersection polylines.
  3. Turn each polyline into an `IntersectionCurve`, an exact implicit curve defined by (surface0, surface1,
     leader polyline) and evaluated by Newton projection from the leader.
  4. `loops_store`: insert the curves into each face's boundary loops.
  5. `divide_face`: split the faces.
  6. Classify the face components by ray crossing against the other mesh, keeping inside for AND and outside
     for OR.
* **Documented limits:** only **transversal** intersections; tangent or coincident faces are not supported.
  Fillets only on a single edge whose end vertices each touch exactly three faces.
* **Lesson:** mesh-first intersection gives robust *start points* and loop detection for free (no missed
  closed loops, which is the classic SSI failure). An implicit "intersection curve with leader" representation
  avoids early approximation. Its lack of same-domain handling is exactly what makes it unusable for "sketch on
  face, extrude Join", which is the case we must handle.

### 3.4 Fornjot (Rust)

* [github.com/hannobraun/fornjot](https://github.com/hannobraun/fornjot): a B-rep kernel prioritizing
  "reliability over features". It only ever reached sketches plus sweeps and simple models. Mainline was
  frozen for a year in favor of experiments, and the **repository was archived in June 2026** (project shut
  down).
* **Lesson (a cautionary one):** years spent on validation infrastructure and on reworking the core
  representation, without shipping booleans, ended the project. Pick a representation (ours already exists),
  then get to Join/Cut on the common analytic cases quickly. Its earlier "approximation-first" idea (geometry
  converted to polylines at a known tolerance, with topology exact) is still a useful mental model for the
  *classification* stage.

### 3.5 CGAL Nef polyhedra and exact predicates

* Code: [`CGAL/cgal/Nef_3`](https://github.com/CGAL/cgal/tree/master/Nef_3), docs
  [doc.cgal.org/latest/Nef_3](https://doc.cgal.org/latest/Nef_3/). Paper: Granados, Hachenberger, Hert,
  Kettner, Mehlhorn, Seel, *Boolean operations on 3D selective Nef complexes*, ESA 2003.
* Selective Nef complexes represent polyhedra *exactly* with rational arithmetic, including non-manifold
  and open sets. Booleans are overlays of the local sphere maps at each vertex. They are fully robust and very
  slow, with heavy memory use, and **linear only**. OpenSCAD replaced its CGAL Nef backend with
  [Manifold](https://github.com/elalish/manifold) (which uses symbolic perturbation and a floating-point
  guarantee of manifold output) for speedups of orders of magnitude.
* Related robust *mesh* boolean work: Zhou, Grinspun, Zorin, Jacobson, *Mesh Arrangements for Solid
  Geometry* (SIGGRAPH 2016); Cherchi, Livesu, Scateni, Attene, *Fast and robust mesh arrangements using
  floating-point arithmetic* (SIGGRAPH Asia 2020) and *Interactive and Robust Mesh Booleans* (SIGGRAPH Asia
  2022), which use indirect predicates. Shewchuk, *Adaptive Precision Floating-Point Arithmetic and Fast
  Robust Geometric Predicates* (1997).
* **Lesson:** exact arithmetic is the right tool for *linear decisions*: orientation of a point to a plane in
  sketch-region building, polygon nesting and 2D arrangement in UV. Use Shewchuk-style adaptive predicates
  there. It is not viable for curved SSI. A Manifold-style mesh boolean is a reasonable **preview or fallback**
  ("the exact boolean failed, show the mesh result and flag the feature"), never the model of record.

### 3.6 Papers: robust B-rep booleans, SSI, blends

Tolerances and robustness:
* D. J. Jackson, *Boundary representation modelling with local tolerances*, ACM Symp. Solid Modeling 1995. This
  is the Parasolid tolerant-modeling paper (per-edge and per-vertex tolerances).
* M. Segal, *Using tolerances to guarantee valid polyhedral modeling results*, SIGGRAPH 1990. Segal & Séquin,
  *Consistent calculations for solids modeling*, SoCG 1985.
* C. M. Hoffmann, *Geometric and Solid Modeling* (1989; [free online](https://www.cs.purdue.edu/homes/cmh/distribution/books/geo.html)),
  and *Robustness in geometric computations* (JCISE 2001).
* C.-Y. Hu, N. M. Patrikalakis, X. Ye, *Robust interval solid modelling, Parts I & II*, CAD 28(10), 1996
  (interval arithmetic B-rep).

Surface-surface intersection:
* N. M. Patrikalakis, T. Maekawa, *Shape Interrogation for Computer Aided Design and Manufacturing*,
  ch. 5 SSI ([MIT hyperbook](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/)). This is the best
  single reference for subdivision, marching, loop detection and tangential cases.
* R. E. Barnhill, S. N. Kersey, *A marching method for parametric surface/surface intersection*, CAGD 1990.
* T. W. Sederberg, R. J. Meyers, *Loop detection in surface patch intersections*, CAGD 1988 (the Gauss-map /
  normal-cone test that rules out closed loops inside a patch pair).
* S. Krishnan, D. Manocha, *An efficient surface intersection algorithm based on lower-dimensional
  formulation*, ACM TOG 1997.
* T. A. Grandine, F. W. Klein, *A new approach to the surface intersection problem*, CAGD 1997 (Boeing).

Blends and fillets:
* J. R. Rossignac, A. A. G. Requicha, *Constant-radius blending in solid modelling*, Comp. Mech. Eng. 1984.
* B. K. Choi, S. Y. Ju, *Constant-radius blending in surface modelling*, CAD 21(4), 1989.
* J. Vida, R. R. Martin, T. Várady, *A survey of blending methods that use parametric surfaces*, CAD 26(5),
  1994.
* G. Lukács, *Differential geometry of G1 variable radius rolling ball blend surfaces*, CAGD 15(6), 1998.

Topological naming: see §4.

---

## 4. Topological naming

### 4.1 How existing systems handle it

**OCCT TNaming** ([`src/ApplicationFramework/TKCAF/TNaming/`](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ApplicationFramework/TKCAF)).
* Each label in the OCAF data tree stores a `TNaming_NamedShape` with **evolution records**: `PRIMITIVE`,
  `GENERATED`, `MODIFY`, `DELETE`, `SELECTED`, written by `TNaming_Builder` from an algorithm's
  `Generated/Modified` history.
* A selection (`TNaming_Selector`) is stored as a **`TNaming_Naming` structure**: a typed name such as
  `IDENTITY`, `INTERSECTION` (an edge is the intersection of these two named faces), `FILTERBYNEIGHBOURGS`,
  `ORIENTATION`, `WIREIN`, `SHELLIN`, with arguments that are themselves named shapes. Re-solving after
  recompute follows the evolution forward and re-evaluates the name: for example, intersect the new images of
  the two faces.
* It is powerful but heavyweight, and it is tied to OCAF. Few applications use it fully.

**FreeCAD 1.0** (realthunder's algorithm, upstreamed in 2023–2024).
* Docs: [Topological Naming](https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming),
  [Topological Naming Algorithm](https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming-Algorithm).
  Code: [`src/App/ElementMap.cpp`](https://github.com/FreeCAD/FreeCAD/blob/main/src/App/ElementMap.cpp),
  [`src/App/ElementNamingUtils.cpp`](https://github.com/FreeCAD/FreeCAD/blob/main/src/App/ElementNamingUtils.cpp),
  [`src/Mod/Part/App/TopoShapeExpansion.cpp`](https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/App/TopoShapeExpansion.cpp)
  (`makeElementShape`, `mapSubElement`, …).
* Every `TopoShape` carries an **element map** from an indexed name (`Face6`) to a **mapped name** that encodes
  history. Postfixes: `;:M` modified, `;:G` generated, `;:U` named from an upper element, `;:L` named from lower
  elements, `;:T<tag>:<len>:<type>` the source shape tag. Example: `Face6;:M2;FUS;:T1:5:F`, meaning a face
  modified (index 2) from Face6 of shape tag 1 by a fusion. Multiple sources go in parentheses. Long names are
  hashed into a per-document `StringHasher` table (`#a8;:M#a7;RFI;:T2:2:F`).
* The algorithm runs in four passes per operation: (1) copy unchanged names; (2) name generated and modified
  elements from the OCCT `MakeShape` history; (3) name unnamed lower elements from their named upper element
  (`;:U`, indexed); (4) name the remaining upper elements from their named lower elements (`;:L(e1,e2,…)`).
* Selections (sketch attachment, fillet edges) store mapped names, so `Face6` becoming `Face7` no longer
  breaks them. The documented cost is about 30% more recompute time and about 27% larger files.
* Remaining weaknesses: index ordering still matters when one edge splits into several pieces. Fuzzy recovery
  (`getRelatedElements`) is weak across many steps.

**Onshape** (Parasolid plus FeatureScript; explained by Onshape developers on the forum, e.g.
[thread](https://forum.onshape.com/discussion/comment/120342/)).
* Every operation has a hierarchical **feature/operation id** (`id + "extrude1" + "sub"`). For every piece of
  topology an operation creates, the system records **its dependencies**: which operation made it and from
  which input entities (for example, "side face of opExtrude X generated from sketch edge E").
* A user selection is turned into a **query**, an encoding of that record. On regeneration, the query is
  **evaluated by pattern-matching against the new history**, starting with the operation id, giving the
  current *transient ids*. Transient ids are valid only within one regeneration.
* If an operation's id has an **unstable component** (inside loops, or when the number of inputs changes),
  all results of those operations are candidates, and **external disambiguation** (extra recorded
  information, for example adjacency to other named entities) picks the right one. FeatureScript exposes
  this through `qCreatedBy(id, EntityType.FACE)`, `qSketchRegion`, `qCapEntity(id, CapType.START)`,
  `qNonCapEntity`, `qEdgeAdjacent` and so on. Feature code refers to topology *semantically* rather than by
  index.
* Onshape is generally regarded as the most stable of the major systems on this front. When a query fails, the
  feature shows a red "missing reference" and asks the user to re-select.

**Fusion 360**: see §1.4. It uses kernel attributes (ASM) with split, merge and copy behaviour that carry a
persistent `associativeID` through operations. Feature references point to those ids. On ambiguity it warns or
fails and asks the user to repair the selection.

**Literature:** J. Kripac, *A mechanism for persistently naming topological entities in history-based
parametric solid models*, CAD 29(2) 1997 (the "topological ID system" with face, edge and vertex histories,
used in Inventor's lineage). V. Capoyleas, X. Chen, C. M. Hoffmann, *Generic naming in generative,
constraint-based design*, CAD 28(1) 1996 (names built from the generating operation plus topological
context). S. H. Farjana, S. Han, *Mechanisms of persistent identification of topological entities in CAD
systems: a review*, Alexandria Eng. J. 2018.

### 4.2 Recommended approach for luce-cad

The aim is a lean combination of Onshape's queries with FreeCAD's generative names, built in from the start:

1. **Every modeling operation returns a history.** For each output face, edge and vertex it records
   `generated_from` (the input entity, or the sketch curve id plus a role) and `modified_from` (input
   entities). It also records `deleted` inputs. Primitives, extrude, revolve, boolean, fillet, chamfer, shell
   and pattern all report it. This is a kernel API requirement, not a UI feature, so it costs almost nothing
   if done now and is very hard to add later.
2. **Stable generative names as attributes on topology.** A name is a small structured value, not a string:
   `(feature_id, role, sources[], disambiguator)`.
   * Extrude: `cap.start`, `cap.end`, and `side[sketch_curve_id]` (plus `side[curve, segment k]` when one
     curve yields several faces, for example a circle split at a seam). Edges are named `cap.start ∩
     side[c]`, and so on.
   * Revolve: `side[curve_id]`, `start`, `end`.
   * Boolean: faces keep their input names (`modified`). Faces split in pieces get a disambiguator computed
     from a stable rule: the names of the neighboring faces that bound the piece, never an index. New
     intersection edges are named `face_a ∩ face_b [#k]`, with k ordered along a deterministic parameter.
   * Fillet and chamfer: `blend[edge_name]`, `corner[vertex_name]`; trimmed neighbors keep their names.
   * Pattern and mirror: `instance[i]` prepended.
   The sketch provides stable curve ids (the sketch entity ids), which makes most references to extrude
   faces immune to edits. This is the 80% case.
3. **References stored by downstream nodes are queries**, Onshape-style: "faces whose name matches
   pattern P", plus, for edges, "the edge between faces A and B". A geometric **fingerprint** is stored
   alongside: surface type, normal or axis, centroid, area or length, and the bounding box at the time of
   selection. Resolution: exact name match → if several, disambiguate by the fingerprint and neighbor names
   → if none, follow `modified/generated` forward (it may be a split, so prefer the piece most like the
   fingerprint) → otherwise **fail visibly**.
4. **UX for failure:** the node turns yellow (resolved by fallback, with "check this") or red (unresolved),
   shows the old selection as a ghost overlay, and offers one-click re-pick. Never silently pick something
   else.
5. **Recompute determinism:** the same inputs must give the same topology order and names. Iterate in stable
   orders (by name, not by pointer or hash-map order). This is a property to test: recompute twice and
   compare the name maps.

---

## 5. Recommended build order (small team)

Each phase is usable on its own and unlocks real parts. SSI = surface-surface intersection.

### Phase 0: foundations (before any operation)
* **Tolerance policy.** One model resolution (for example 1e-7 model units for coincidence, scaled to a
  size box, as Parasolid uses 1e-8 resolution in a 1000-unit box). Per-edge and per-vertex tolerance fields
  that operations may enlarge when they approximate.
* **Validity checker.** Closed shells, each edge used by exactly two coedges with opposite orientation,
  consistent loop orientation, p-curve to 3D curve deviation within the edge tolerance, Euler characteristic,
  no self-intersections (sampled). Run it after every operation in tests. This is what keeps a kernel honest.
* **History and naming API** (§4.2), plus a stable iteration order.
* **2D exact or adaptive predicates** for orientation and in-polygon tests.

### Phase 1: sketch → profile regions → constraint solver
* **Regions:** 2D arrangement of the sketch curves. Intersections needed: line/line, line/arc, arc/arc
  (closed form), and line, arc or spline against spline (Bézier subdivision plus Newton). Split the curves,
  build the half-edge graph, extract the faces by "next edge with the smallest clockwise turn", compute
  signed area to tell outer loops from holes, nest holes by point-in-polygon with exact predicates, and name
  each region by its bounding curve ids. Exclude construction curves.
* **Solver:** start with Newton or Levenberg–Marquardt on all equations, using the current geometry as the
  initial guess (it stays close to the drawing). Add Jacobian rank or QR to report DOF and
  redundant/conflicting constraints. Add graph decomposition later for speed (SolveSpace `system.cpp`,
  planegcs `GCS.cpp`). Dimensions are bound to the app's parameter and expression system.
* No SSI is needed in this phase.

### Phase 2: Extrude and Revolve as "New Body" (constructive, no booleans)
* Extrude a region: two planar caps trimmed by the region loops. Side faces: line → **plane**, arc or circle
  → **cylinder**, spline → **linear extrusion surface** (exact NURBS, degree 1 in the extrusion direction).
  Options: two-sided, symmetric, thin (offset the 2D profile, which needs 2D curve offsetting: arcs offset
  exactly, splines approximately).
* Taper: line → plane, arc → **cone**, spline → approximate draft surface. Defer spline taper.
* Revolve: line → plane (perpendicular to the axis), **cylinder** (parallel) or **cone** (oblique); arc →
  **torus** or **sphere** (when its center is on the axis); spline → exact rational surface of revolution.
  Handle profiles touching the axis (degenerate edges or poles).
* Topology is built directly (vertex → edge → face → solid), with full history. **No SSI needed.**
* Also: Mirror and Pattern of bodies (transforms) as New Body.

### Phase 3: Booleans (Join, Cut, Intersect, Combine, extrude/revolve operations, "To Object")
This is the biggest phase. Use the GFA structure (§3.1):
1. Bounding-box (BVH) filtering of face and edge pairs.
2. Edge/face intersections (curve/surface): line/plane, line/quadric (closed form), circle/plane, circle/quadric
   (closed form or a polynomial), curve/NURBS (subdivision plus Newton).
3. Face/face SSI, **in this order of priority**:
   * **plane/plane**: a line. This is every box-like part.
   * **plane/cylinder**: two lines (axis parallel to the plane), a circle (axis perpendicular) or an ellipse.
     This covers holes, bosses and slots.
   * **plane/cone**: a conic (circle, ellipse, parabola, hyperbola, or lines through the apex). Needed for
     taper and countersinks.
   * **plane/sphere**: a circle. **plane/torus**: circles in the perpendicular and axial special cases,
     otherwise march.
   * **cylinder/cylinder**: coaxial (coincident or none), parallel axes (lines), equal radii with
     intersecting axes (two ellipses, the classic "tee"), otherwise a quartic space curve that must be
     **marched** and approximated as a NURBS with p-curves.
   * cylinder/sphere and sphere/sphere with coaxial cases (circles). Other quadric pairs are marched.
   * **plane/NURBS and NURBS/anything**: general marching with start points from boundary curve/surface
     intersections, plus a subdivision or normal-cone loop check (Sederberg–Meyers) so that interior closed
     loops are not missed. Alternatively, use mesh-based start points as truck does.
   * **Same-domain (coincident) faces: plane/plane coplanar first**, then coaxial cylinder/cylinder with equal
     radius. Without this, "sketch on face, extrude Join" (the most common operation) fails. The face pieces
     are merged in 2D in the shared parameter domain.
4. Split edges (pave blocks), split faces (2D loop building in UV, reusing the Phase 1 arrangement code),
   classify the pieces (point-in-solid by ray casting with a fallback to a different ray on a degenerate hit,
   then propagate across connected pieces), select the pieces per operation, and sew with shared edges.
5. Post-process: merge coplanar or co-cylindrical neighbor faces created by the operation (optional, but it
   gives cleaner topology for later fillets). Run the validity check.
* Approximated intersection curves raise the edge tolerance. They never "snap" the surfaces.
* Defer to later: tangent intersections between curved faces (fail with a clear error first), NURBS/NURBS
  coincidence and non-manifold results.

### Phase 4: Chamfer
* Equal distance, two distances and distance-angle on edges between **plane/plane** (a planar chamfer face),
  **plane/cylinder** where the edge is a circle on a plane perpendicular to the axis (a **cone**), and
  **plane/cone** (a cone). This is the KPart idea.
* General case: compute the contact curves on each face (the intersection of the face with the offset of the
  other, or a geodesic distance approximation), then build a **ruled surface** between them.
* Topology: remove the edge, trim the neighbor faces along the contact curves, insert the chamfer face. At
  vertices where 2 or 3 chamfered edges meet, use a planar or ruled corner patch (convex corner: intersect the
  chamfer faces; mixed corners: a small corner face). Chain propagation along tangent-continuous edges.
* SSI needed: plane/plane, plane/cylinder, plane/cone, and the Phase 3 machinery to trim the neighbors.

### Phase 5: Fillet
* Constant-radius rolling ball. KPart cases first: **plane/plane** → cylinder; **plane/cylinder** (circle
  edge) → **torus**; plane/cone → torus; cylinder/cylinder with parallel axes → cylinder. Convex vertex where
  3 equal-radius fillets meet orthogonally → **sphere** patch.
* General case: walk the spine (the intersection of the two **offset surfaces**, which means offset-surface
  evaluation: exact for quadrics, where an offset is the same type with a new radius, and approximated for
  NURBS), get contact points by projection, and approximate the arc family as a NURBS surface (the `BlendFunc`
  / `AppBlend` approach). Tangent-chain propagation.
* Corners: 3-edge vertex blends (rolling-ball corner → sphere or n-sided patch), **setback** corners (an
  n-sided patch with a setback distance, using Coons or Gregory patches at first). Detect a fillet that
  consumes an adjacent face, report it as an error first, and add face removal later.
* Then: variable radius (a radius law along the spine), chord-length and asymmetric fillets, then G2
  (curvature-continuous, not circular) cross-sections.
* SSI needed: offset-surface/offset-surface (the same quadric table, with radii shifted) and contact
  curve/face trimming, using everything from Phase 3.

### Phase 6 and later (deferred)
* **Shell, Offset Face, Draft, Press/Pull, Replace Face, Delete Face.** All of them are "move or replace face
  surfaces, then re-intersect the neighbors and re-trim" (local operations). Shell adds self-intersection
  handling for large thicknesses. Draft is easiest on plane and cylinder faces. This family is what makes
  Plasticity-style direct editing possible, so it is worth doing right after fillets.
* **Sweep and Loft:** constructive NURBS skinning, which is simple to *build*. But they produce NURBS faces,
  so booleans on them need the general NURBS SSI. Ship them as New Body first.
* Hole and Thread (Hole = revolve profile plus Cut; a modeled Thread = a helical sweep, so cosmetic threads
  come first). Rib, Web, Emboss, Boundary Fill: later.
* Fusion-style Pattern "Adjust" mode (re-run the feature per instance) comes after feature patterns work.

### What can be deferred, in summary
NURBS/NURBS SSI beyond marching, tangent curved intersections, non-manifold results, variable and G2
fillets, setback corners, fillets that consume faces, Shell, spline taper, sweep and loft booleans, modeled
threads. With Phases 0 to 5 plus the analytic SSI table, most Fusion-style mechanical parts can be modeled.

---

## 6. Sources (quick list)

* Fusion help: [Fillet](https://help.autodesk.com/cloudhelp/ENU/Fusion-Model/files/SLD-FILLET-SOLID.htm);
  API: [FilletFeatureInput](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/FilletFeatureInput.htm),
  [BRepFace.entityToken](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/BRepFace_entityToken.htm);
  forum: [References Repeatedly Lost](https://forums.autodesk.com/t5/fusion-support-forum/references-repeatedly-lost/m-p/13203525).
* Plasticity: [product](https://plasticity.xyz/product), [manual](https://doc.plasticity.xyz/),
  [fillet/shell](https://doc.plasticity.xyz/solid/fillet-shell), [CG Channel 1.3](https://www.cgchannel.com/2023/10/nick-kallen-ships-plasticity-1-3/),
  [Onshape forum: Plasticity 2025.1](https://forum.onshape.com/discussion/27002/plasticity-2025-1).
* OCCT: [repo](https://github.com/Open-Cascade-SAS/OCCT), [Boolean spec](https://occt3d.com/dev/doc/overview/html/specification__boolean_operations.html),
  [TKBO](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKBO),
  [TKFillet](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKFillet),
  [TKPrim](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKPrim),
  [TKGeomAlgo](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ModelingAlgorithms/TKGeomAlgo),
  [TKCAF/TNaming](https://github.com/Open-Cascade-SAS/OCCT/tree/master/src/ApplicationFramework/TKCAF).
* SolveSpace: [src/srf](https://github.com/solvespace/solvespace/tree/master/src/srf), [system.cpp](https://github.com/solvespace/solvespace/blob/master/src/system.cpp).
* truck: [truck-shapeops](https://github.com/ricosjp/truck/tree/master/truck-shapeops/src).
* Fornjot: [repo (archived 2026)](https://github.com/hannobraun/fornjot).
* CGAL: [Nef_3](https://github.com/CGAL/cgal/tree/master/Nef_3); Manifold: [repo](https://github.com/elalish/manifold).
* FreeCAD: [realthunder naming](https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming),
  [algorithm](https://github.com/realthunder/FreeCAD_assembly3/wiki/Topological-Naming-Algorithm),
  [ElementMap.cpp](https://github.com/FreeCAD/FreeCAD/blob/main/src/App/ElementMap.cpp),
  [TopoShapeExpansion.cpp](https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Part/App/TopoShapeExpansion.cpp),
  [planegcs](https://github.com/FreeCAD/FreeCAD/tree/main/src/Mod/Sketcher/App/planegcs).
* Onshape: [forum explanation of queries](https://forum.onshape.com/discussion/comment/120342/).
* Papers: as listed in §3.6 and §4.1.
