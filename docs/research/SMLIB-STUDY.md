# SMLib Study: what NVIDIA's open B-rep kernel can teach luce-cad

NVIDIA published `usd-brep` (https://github.com/NVIDIA-Omniverse/usd-brep, Apache-2.0) in 2026. Its
core is SMLib, the solid modeler of the NLib / GSNLib / TSNLib / SMLib family: about 970,000 lines
of C++ with NURBS tools, surface/surface intersection, booleans, fillets, healing, offsets,
tessellation and a Radial Edge non-manifold B-rep. Around it sit a flat C API (`SM_API`), USD
readers and writers, OpenCASCADE import and export, and a Hydra delegate. The repository also holds
the AOUSD proposal for B-reps in USD (`proposals/UsdSolid`).

This study reads SMLib against luce-cad's weak areas, as listed in [../MODELING.md](../MODELING.md)
(Limits), [CAD-MODELING-STUDY.md](CAD-MODELING-STUDY.md), [FILLET-STUDY.md](FILLET-STUDY.md),
luced-3d's `docs/CAD_TESSELLATION.md` and the Fusion probe findings. It does not repeat those
documents.

SMLib is reference only. Nothing here or in our repositories is copied from it: no code, comments
or tables. The notes below describe its algorithms in our words and name the files and functions
they come from (paths under `source/SMLib/src/` unless given). The comparison build and its drivers
stay outside our repositories, under `luce_work/cad/donors/`.

## 0. Summary

SMLib is an old, broad kernel. Where it is ahead of luce-cad, it is ahead in **breadth and in
topology machinery**, not in the quality of its surfaces:

- **It runs the contact cases we refuse.** Tangential booleans, two equal pipes crossing,
  B-spline faces lying on each other, a shell of B-spline faces, fillets at a four-edge vertex: all
  ran through SMLib in the comparison below, and all are refused or broken in luce-cad today.
  Three of the tangential results are genuinely non-manifold (an edge with four faces), which
  SMLib can hold and we cannot.
- **Its intersection marches in both surfaces' parameters at once** with a plane-constrained
  Newton corrector, and it treats tangency as a kind of point (classified from the two surfaces'
  second fundamental forms), not as the end of a trace.
- **It classifies by local geometry, not by counting.** Pieces next to an intersection edge are
  classified by stepping off the edge into each face until the faces part (radial sectors); a
  point far from any edge by the nearest ray hit and that hit's own sector. No parity, no voting.
- **Coincidence is detected for any surface pair**, B-splines included.
- **Its fillet engine is a case analysis**: corners typed by how many edges meet and how many are
  filleted, overflow handled by classifying each rail against its face, and a fillet-face removal
  that extends B-spline neighbours naturally.
- **Its surfaces are weaker than ours.** Circular fillets are cubic approximations by default,
  corners are G1 within an angle, there is no tangency weight, and its variable radius never
  rolls a ball (0.97 off Fusion's volume on the 2 → 6 edge where we are within 2e-4).
- **It was faster on the one timing case.** The 100-hole plate took 290 ms in SMLib built
  single-threaded, 1975 ms in luce-cad.

The ranked list of changes is in §11, what not to take in §12, and the UsdSolid plan in §10.
The top five: size shared edges from the faces before meshing (no more dropped faces), coincident
B-spline faces, a sphere chart for any trims, one crossing vertex per piercing with tangency as a
point type, and marching in both surfaces' parameters.

## 1. What SMLib is

### 1.1 Layers

Approximate line counts in `source/SMLib/src`:

| Layer | Main files | Lines | What it does |
|---|---|---|---|
| NLib | `NL_Crv*`, `NL_Srf*`, `NL_Basis*`, `NL_Bezier`, `NL_*Fit`, `NL_*Approx`, `NL_SrfOffset`, `NL_Tessellate`, `NL_Iges` | 254k | A C NURBS toolkit: knots, evaluation, fitting, approximation, extension (`N_SrfExtendByDist`), offsets, N-sided fills, IGES |
| Topology | `SmBrep`, `SmFace`, `SmEdge`, `SmEdgeuse`, `SmVertex`, `SmVertexuse`, `SmLoop(use)`, `SmShell`, `SmRegion`, `SmProtoTopology`, `SmTopologyTraverser` | 109k | The Radial Edge B-rep and its Euler-style edits (glue, squeeze, split, delete) |
| Curves | `SmCurve`, `SmBSplineCurve`, conics, `SmCrvOnSurf`, `SmProjectedCurve`, `SmOffsetCurve`, `SmCurveClass` | 85k | Curve classes; classifying a curve against a face |
| Surfaces | `SmSurface`, plane, cylinder, cone, sphere, torus, revolution, extrusion, `SmBSplineSurface`, `SmOffsetSurface`, `SmBlendSurface` | 88k | Every analytic surface keeps a NURBS twin |
| Intersection and merge | `SmSurfaceIntersector`, `SmAdvSurfaceIntersector`, `SmTopologyIntersector`, `SmMerge`, `SmGlobalSolver`, `SmLocalSolve*`, `SmSurfaceDrop*`, `SmPointClass` | 67k | SSI, point and curve classification, booleans |
| Fillets | `SmFillet*`, `SmCurveFilletExecutive` | 48k | §6 |
| Offset | `SmOffsetExecutive`, `SmOffsetSurface`, `SmOffsetGeometryCreation` | 10k | §8 |
| Healing | `SmHealData`, `SmStitch`, `SmGap`, `Sm*Props`, `SmTol` | 25k | §5, §7 |
| Caches | `SmCacheMgr*`, `SmSurfaceCache`, `SmCurveCache`, `SmTrimSrfCache`, `SmBrepCache`, `SmPolarBox`, `SmExtent*` | 28k | Bézier subdivision trees with boxes and normal cones; spatial trees of the B-rep |
| Mesh | `SmPoly`, `SmTess`, `SmPolyDecimate`, `SmPolyMerge`, `SmPolyIntersector` | 63k | A polygon B-rep, the tessellator, decimation, mesh booleans |
| Other | sweeps, primitives, volumes and deformers, attributes | 50k | |

Around it, in `source/`:
- **SM_API**: a flat facade (`SmApiBoolean*`, `SmApiFilletEdges`, `SmApiShellBrepFull`,
  `SmApiStitch*`, `SmApiTessellate`, …) that mostly fills in defaults.
- **BREP_USD_DATA**: arrays mirroring the `BrepArray` USD schema, with readers, writers and
  builders. **BREP_SM_USD**: SMLib B-rep to and from those arrays.
- **BREP_STAGING**: neutral records importers fill before writing a `BrepArray`.
- **OCCT_BREP_IMPORT / EXPORT**: parsers and writers of OpenCASCADE's `.brep` text and binary
  formats, without linking OpenCASCADE.
- **HYDRA_USD_BREP**: a Hydra scene index that loads `BrepArray` prims into SMLib, tessellates them in
  parallel and hands meshes to Storm.

### 1.2 The data model

SMLib's B-rep is Weiler's Radial Edge structure: `SmBrep` → `SmRegion` (solid or void) → `SmShell`
→ `SmFaceuse` → `SmLoopuse` → `SmEdgeuse` → `SmVertexuse`, with the geometric owners (`SmFace`,
`SmLoop`, `SmEdge`, `SmVertex`) in flat lists. Every face has two faceuses, one per side. An edge's
uses form one radial list ordered by the right-hand rule, mates on the same face alternating with
radial neighbours in the same shell: a manifold edge has four uses, a sheet's boundary edge two, a
non-manifold edge 2k. Shells can hold faces, wire edges or a lone vertex. A manifold solid is two
regions (the infinite void and the solid); a sheet is faces with both sides on the infinite region;
a non-manifold body is any number of regions, each marked void or solid.

Each pair of mated edge uses stores one B-spline trim curve in its face's parameters
(`SmEdgeuse::CreateUVTrimCurve`, which also records how far the drop was), and caches its measured
gaps (edge to surface, end to next start). Change tracking: every edit sends an event (split,
merge, remove, add, geometry changed) through `SmObject::Notify`; attributes get split, merge and
copy callbacks, which is a topological naming hook, and `SmTrackTopologyChanges` collects what an
operation created and deleted.

luce-cad's `BrepModel` is packed arrays: faces with signed edge uses, two uses per edge, no regions
or shells, no trim curves, one tolerance plus per-edge and per-vertex precision. It is smaller and
faster to walk, and cannot hold B1's answer.

## 2. Head-to-head runs

SMLib's kernel and `SM_API` were built locally with clang (`-O2`, arm64, 16 s on ten cores). Its
only outside dependency, TBB, was replaced by a serial stand-in, so SMLib ran **single-threaded**
here; its own build runs face pairs and faces in parallel. Each case was written twice, once against
`SM_API` in C++ and once against `CadModeling` in Luce, with the same shapes (y up, boxes centered
on the origin). luce-cad was at `b465b0f`.

Volumes: SMLib's are its exact B-rep integral (`SmApiBrepComputeVolume`, relative accuracy 1e-8).
Ours are mesh volumes at 32 and 64 divisions, extrapolated as in `fusion_probe_tests` (at 16 and 32
when the 64 mesh skipped a face). "Closed" means the tessellation is watertight (SMLib's own
report; ours by `checks.closed`'s rule, without trapping). Times are the operation alone, not the
tessellation; luce-cad's clock reads whole milliseconds, so 0 means under 1 ms.

### Booleans

| Case | Exact | luce-cad | SMLib |
|---|---|---|---|
| B1 box ∪ cylinder touching a face along a line | 9570.7963 | refused (edges not used twice) | 9570.7964, non-manifold, 1.0 ms |
| B2 box − cylinder touching a face from inside | 6429.2037 | 6429.223, mesh **not closed** | 6429.2037, non-manifold, 1.1 ms |
| B3 box ∪ ball touching a face at a point | 8523.5988 | 8523.5987, closed | 8523.5988, closed, 1.2 ms |
| B4 two r5 pipes crossing at right angles | 4045.7223 | **refused** | 4045.7222, closed, 6.2 ms |
| B5 two r5 pipes touching at a point | 4712.3890 | 4712.378, closed | 4712.3893, closed, 0.9 ms |
| B6 two cylinders touching along a line | 3141.5927 | refused | 3141.5927, non-manifold, 1.8 ms |
| B7 boxes with flush faces | 13000 | 13000, closed | 13000, closed, 0.5 ms |
| B8 two G2-rounded boxes, one turned (∪, ∩, −) | adds up | adds up to 0.08 (mesh), 34 ms each | adds up exactly, 4–5 ms each |
| B8 the same with every face turned to NURBS | adds up | (no such input) | adds up exactly, 5–6 ms |
| B9 G2-rounded box and its copy moved half its height (∪, ∩, −) | 1.5, 0.5, 0.5 of one | **refused** (Limits 6) | exact to 2e-7, closed, 20 ms each |
| P1 100 holes drilled one by one in a 40 × 4 × 40 plate | 5693.1417 | 5693.18, closed, 1975 ms | 5693.1416, closed, 290 ms |

B1, B2 and B6 have an edge where four faces meet, so the right answer is a non-manifold body.
SMLib returns it (its own manifold check says no, its tessellation leaves those edges unpaired). We
refuse B1 and B6, and B2 passes our edge check but meshes open: the line where the cylinder touches
the wall is a seam of zero thickness. The G2 strips in B8 and B9 differ between the kernels (ours
follow Fusion's G2 section, SMLib's is a faired quintic), so their volumes differ; what is compared
is whether the pieces add up and whether the operation runs.

### Fillets

| Case | Reference | luce-cad | SMLib |
|---|---|---|---|
| F1 20 cube's edge, r 4, G1 | 7931.3274 (Fusion) | 7931.3276 | 7931.3274, 1.0 ms |
| F2 three edges at a corner, r 4 | 7804.6961 (exact) | 7804.6957 | 7804.6961, 2.9 ms |
| F3 all twelve edges, r 2 | 7804.6961 (exact) | 7804.6942 | 7804.6961, 12.5 ms |
| F4 variable 2 → 6 along an edge, G1 | 7926.5667 (Fusion's cone) | 7926.5665 | **7925.6001** (scaled section, not a rolled ball) |
| F5 dome rim (ball 10 cut at its middle), r 2 | 2020.5625 (Fusion) | 2020.5624 | 2020.5578 (cubic arc approximation); its blend-curve G1 2028.02 |
| F6 plate 4 thick, top edge r 5 and 4.5 (overflow) | — | refused (radius must be under 4) | failed (generic error) |
| F7 T of pipes r5 and r3, saddle loop r 1 | — | 3433.159, the 64 mesh drops a face | 3433.1671, closed, 11 ms |
| F8 pyramid, every edge r 1 (a four-edge apex) | — | not possible (Limits 1) | 1933.1271, closed, 10 ms |

SMLib's `SmApiRemoveFillet` returned success without changing a box with four G1 or four G2 strips
(its strip removal wants B-spline neighbours, §7). Our `deleted_faces` gives the box back exactly
(8000) and removes one G2 strip of four to 7747.61 (exact 7747.606).

### Shell

| Case | Exact | luce-cad | SMLib |
|---|---|---|---|
| S1 20 box, wall 1 inside, top open | 1844 | 1844 | 1844, 3.1 ms |
| S2 the G2-rounded box (strips r 5), wall 1 inside, top open | — | **refused** ("a wall or a rounded face is thinner than it") | 1717.8458, closed, 97 ms |
| S3 the same, closed (a void) | — | refused | 2034.2402, closed, 88 ms |

Our refusal names the wrong cause: the strips' radius is 5, the wall 1. The real cause is that
B-spline faces are not offset (Limits 9).

### Tessellation

| Case | Exact | luce-cad | SMLib |
|---|---|---|---|
| T1 ball 10 drilled through at 30° (r 3) | 3636.2246 | closed at 16, 32, 64; 3635.73 extrapolated (16: 3620.8, 32: 3632.3, 64: 3634.9) | closed, 13,844 triangles at chord 0.01, 59 ms |
| T2 ball 10 with eight blind holes all round (none round a pole) | — | closed at 16 and 32; **six faces skipped at 64** ("adaptive patch exceeds face budget") | closed, 74 ms |
| F7 the T of pipes | — | the main pipe's face skipped at 64 | closed |

Our drilled spheres now mesh (since `b465b0f`), but they converge slowly in volume, and the face
budget drops whole faces at 64 divisions on curved faces with holes. SMLib met every case closed at
a 0.01 chord height.

## 3. Surface/surface intersection

### 3.1 How SMLib does it

**Dispatch.** `SmSurface::GlobalSurfaceIntersect` always ends in `SmAdvSurfaceIntersector::DoIntersection`.
Analytic classes override the entry point first: `SmPlane::GlobalSurfaceIntersect` sends plane
against plane, cone, cylinder, sphere, extrusion and revolution to closed forms, as do `SmCone`,
`SmSphere` and `SmSurfOfRevolution`. The table is centred on planes and is narrower than ours
(`intersect.lucb` also has cylinder/cylinder, sphere/sphere and coaxial revolutions).

**Culling.** Each surface keeps a tree of Bézier patches (`SmSurfaceCache`). Root boxes, grown by
the tolerance, reject disjoint pairs. `SolveIt` with an intersection-test option walks leaf pairs
whose oriented boxes overlap. Two shortcuts catch grazing contact: a single thin overlap box is
resolved as a point or corner touch (`GrazingBoxProcess`), and opposing planar boundary planes give
"disjoint" or "graze" without tracing (`DoBoundaryPlaneTest`).

**Start points.**
- *Boundary seeds* (`FindBoundaryStartPoints`): each surface's four iso-boundary curves, exact, are
  intersected with the other surface (`SmSurface::GlobalCurveIntersect`). A point solution is typed
  (`ComputePointValues`); a tangent touch becomes a point result, the rest become starts, longest
  curves first (points on both boundaries). A *range* solution, a boundary lying in the other
  surface, is output directly as a curve.
- *Interior seeds* (`SmAdvSurfaceIntersector::FindInteriorCurves`, `SolveBranch`, `LocalSolve`):
  every overlapping pair of leaf patches gives a guess from the patches' closest midpoints, refined
  at a hundredth of the tolerance, skipped when it lies on a curve already traced. Seeds whose
  normals are within 5° are dropped, so interior tangential loops are not seeded. A normal-cone
  test (`BranchIsGaussSeparate`, `SmPolarBox`) exists but is switched off by a flag that is never
  cleared.

**Marching** (`TraceCurve`, `ComputeNextPoint`, `RefinePoint`, `TestSpanAccuracy`, `TestPoint`).
Every point carries both surfaces' parameters, their first and second derivatives and normals.
- Predictor: a straight step in both parameter domains along the curve tangent (the normals' cross
  product) dropped into each surface, clipped to the domains.
- Corrector (`SmSurface::LocalSurfaceIntersect`): a damped Newton in (u1, v1, u2, v2) whose
  equations are "the two surface points coincide" plus "the point lies on the plane through the
  guess whose normal is the previous tangent". The plane keeps the system square near tangency. It
  knows about domain edges: a guess on a boundary first intersects that iso-curve with the other
  surface.
- Acceptance: the tangent may not turn past an angle; cubic Hermite spans are built in 3D and in
  both parameter domains, and at five points inside the span the 3D curve must stay within half the
  tolerance of each surface. Accepted easy steps grow by 1.5, rejected ones shrink by 1.5.
  (`ComputeStepSize`, a curvature-based step, is a stub.)
- End: the trace snaps to a known start or end point when a span passes near it, closes when it
  passes its own start, and stops at a domain boundary or when it runs onto a curve already traced.

**Tangency and branch points** (`SmAdvSurfaceIntersector::ComputeSingularityPoint`). When the
normals become parallel the point is classified, not abandoned. Each surface's local shape
(planar, elliptic, hyperbolic, cylindrical) is read from its second fundamental form. A plane
against a cylinder-like surface touches along a line (two directions); a plane against a saddle has
four branches along the asymptotic directions; a plane against a bowl touches at a point. In
general the two surfaces' Dupin indicatrices are intersected and each crossing is one branch: none
is a tangent point, two a tangent curve (keep the one nearest the incoming direction), more a branch
point. Near a crossing whose normals are within 3°, a solve for parallel normals lands on the exact
tangency. Tangent points get ten times the tolerance in the corrector. Two gaps: branches leaving a
starting branch point are not queued, and starts at a pole are skipped.

**Output** (`FlushCurve`): three cubic B-splines on one arc-length parameter, the 3D curve and the
curve in each surface's parameters, with every interior knot triple (C0 in form, G1 in practice).
Each curve is typed crossing, tangent or near-tangent and carries its largest deviation.

### 3.2 Against luce-cad

`march.lucb` traces in 3D on two fields: an analytic surface's implicit function, a B-spline's
signed distance to its nearest point. Seeds are sign changes along 128 samples of each boundary edge
and on a 32 × 32 grid; the step is fixed (0.4% of the model's size); the corrector is a 2 × 2
Gauss-Newton; a trace ends where the gradients become parallel; each curve is one interpolating
cubic, traced again at half the step until it is within half the tolerance. There are no curves in
the faces' parameters.

What this explains in the runs:
- **Tangential contact gives no sign change**, so the seeds miss it, the 2 × 2 system is singular
  there, and the trace stops where the curve goes on. B4 (two equal pipes crossing) fails because
  its two ellipses cross at two points where the surfaces are tangent.
- **The nearest-point field is not smooth** on curved B-splines (the nearest point jumps), every
  corrector step pays for a nearest-point search, and one global step over- or under-samples.
  B8 took 34 ms against SMLib's 4–5 ms.
- **A tangent piercing point is accurate only to about √(tolerance × radius) along the curve**, so
  the end points that two sections compute for one crossing land apart and become two vertices.
  That is the "edges not used twice" refusal in B1 and B6.

What to take (§11 items 1 and 2): march in (u1, v1, u2, v2) with the plane-constrained corrector,
Hermite acceptance in 3D and both charts, and local step control; output the 3D curve with both
parameter curves; seed from exact iso-boundary/surface intersections and from Bézier leaf pairs
(our `spline_rays.lucb` already builds the nets), with a working normal-cone test to skip pairs that
cannot hold a loop; and treat tangency as a point type with a parallel-normal solve and branch
classification.

## 4. Booleans and classification

### 4.1 How SMLib does it

`SmMerge::ManifoldBoolean` (`SmMerge.cpp`; `NonManifoldBoolean` sets a flag and calls it) is one
long function:

1. **Move both bodies to the origin** for the duration when they are far from it
   (`SmTemporaryBrepTransform`).
2. **Intersect and imprint** (`SmTopologyIntersector::IntersectInsertRelate`). Vertices and wire
   edges of each body are tested against the other; then every face pair whose tolerance-grown
   boxes meet (`SmTopologySolver`, `SmGlobalSolver::BranchMayContainAnswers`) goes through
   `SmSurface::CoincidenceCheck` and then `SmFace::FaceIntersect`: SSI of the untrimmed surfaces, each
   curve classified against both faces' trims (`CurveOnClassify` in `SmCurveClass.cpp`) and kept
   where it is inside both. Results go into **proto-topology** (`SmProtoTopology.cpp`): a proto-vertex
   is a cluster of point classifications, each naming the lowest-dimensional entity (vertex, then
   edge, then face) of each body that the point lies on within that entity's tolerance; proto-edges
   are split where they cross and where proto-vertices lie on them. Only then are the real edges and
   faces split (`ImprintIntersections`), and mated entities take the larger of their tolerances.
3. **Repair before giving up.** Up to ten passes look for intersection loops with a gap: a vertex
   on only one new edge. `CloseIntersectionLoop` mates a neighbouring edge onto the other body's lone
   edge or re-intersects it; `FixGaps` squeezes two lone vertices closer than their summed tolerances
   when no long edge joins them. Gaps that remain are flagged, not fatal.
4. **Classify at the new edges.** For each face on an intersection edge, `SmEdge::FindRadialSector`
   finds which sector between the other body's faces round the mated edge it falls in (§4.2). The
   face and everything reachable from it without crossing an intersection edge
   (`SmTopologyTraverser::CollectFaces`) are kept or dropped together: our connected blocks.
5. **Classify the rest** with one point on one edge of each untouched face
   (`SmBrep::Point3DClassify`).
6. **Clean up** (`BooleanPostProcess`): merge faces on one surface, remove needless edges and
   vertices.

### 4.2 Classifying without counting

- **Points** (`SmBrep::Point3DClassify`, `SmBrep::RayFire`). A ray along +x collects every hit; the
  nearest hit wins, and among near ties the lowest-dimensional entity. The segment from the point to
  that hit is classified by `SmBrep::LocalCurveSector`, which looks at the faces round the hit (an
  edge's or vertex's sectors when the hit is on one). Only when the hit is at the start point does
  it try −x, ±y, ±z. There is no parity and no vote, so a ray grazing a face or hitting an edge is
  answered locally.
- **Faces at an edge** (`SmEdge::RadialSectorClassify`). At a distance d along each face, square to
  the edge, take each face's inward direction (`SmEdgeuse::EvaluateBinormalStepOff`), project into
  the plane normal to the edge and test whether the classified face lies inside the sector. When
  two directions are within 10°, step further: d = tolerance, twice that, then a secant guess on how
  the step-off points separate, up to twenty times. The faces are declared coincident only after
  eight steps in a row with no separation. This is how faces tangent along the new edge are
  settled: look further away until they part. `FindRadialSector` retries at up to ten positions
  along the edge.
- **Coincident surfaces, any kind** (`SmSurface::CoincidenceCheck`). The whole-surface test matches
  domain corners to find how the parameters correspond, then drops a 6 × 6 grid of one surface onto
  the other. The partial test, which booleans use, intersects the boundary iso-curves of each with
  the other's and compares position, normal and normal curvatures at the hits
  (`smsurf_CheckShapeMatch`), or checks either surface's middle on the other. Coincident faces then
  imprint each other's boundaries. The partial test is a heuristic; the eight-step sector test is
  what keeps it honest.

### 4.3 Against luce-cad

Our booleans follow the same outline (OpenCASCADE's General Fuse, MODELING.md "Booleans"): one
working shape, exact intersections, clipping, shared edges, face splitting, classification,
selection, merging. The differences that matter are:
- **Where the crossing vertices come from.** We compute the point where face A's boundary pierces
  face B inside each section's clipping and merge the results by distance (`clip.lucb`,
  `vertex_within`). SMLib computes each piercing once, as a proto-vertex classified against both
  bodies, and snaps every section end to it. With tangency the two can be a long way apart.
- **How pieces are classified.** We fire three rays at a point well inside each piece and take the
  majority (`rays.lucb`), with a mesh for B-spline faces (`spline_rays.lucb`). Parity is fragile on
  tangent double roots and on hits exactly on edges; the sector test needs no ray for pieces next to
  a new edge, which are most of them.
- **Coincidence.** Ours recognises planes, cylinders, cones, spheres and tori only (Limits 6), which
  is why B9 fails.
- **Failure.** We refuse any result whose edges are not each used twice. SMLib repairs small gaps
  first, and can return a non-manifold body when the answer is one.

## 5. Tolerances

**SMLib.** `SmTol.h` describes typed tolerances: a zone round each entity, an intersection
tolerance that is the sum of two zones, an approximation tolerance of half a zone, a scaled numeric
zero, and parameter-space tolerances mapped from 3D. The default zone comes from the model's size
(1e-5 for sizes 0.5 to 500, one decade per two decades beyond). The build in the repository uses
the old model (`SM_USE_OLDTOL` in `SmConfig.h`): every face, edge and vertex stores its own
tolerance. `SmEdge::SetTolerance` and `SmVertex::SetTolerance` only grow, and an edge's growth
raises its vertices, so a vertex's zone always holds its edges'. Tolerances grow where the evidence
is: an edge rebuilt from its faces takes its largest gap (`SmEdge::RefineGeometry`), a vertex from
three surfaces twice its gap (`SmVertex::RefineGeometry`), a new section edge its traced deviation,
mated entities the larger of the two. The model-wide default never grows. The authors' own notes
in `SmTol.h` say per-entity values go stale and become ambiguous, and sketch a newer model (one
zone per body, measured gaps stored on the uses) that is not finished.

**luce-cad.** One tolerance from the model's size (1e-7 of it), plus per-edge and per-vertex
precision (`edge_precision`, `vertex_precision` in `brep/model.lucb`; `edge_slack` and `point_slack`
in the booleans), set by Stitch and now used by the booleans. That matches SMLib's stored zones in
spirit. The rules worth adopting (§11 item 6):
- a vertex's precision is at least the largest of its edges', kept so whenever an edge grows;
- two entities meet when their distance is under the sum of their zones, used everywhere the
  booleans now use ad hoc factors (`max(tol, slack · 0.1)`, ×10);
- precision is set from what was measured: a section edge from its traced deviation, a merged
  vertex from its gap;
- the model's base tolerance never grows, which keeps our rule that a result can be fed back in.

## 6. Fillets, corners and overflow

### 6.1 The pipeline

SMLib's fillets are a rolling-ball engine that builds a separate fillet body and merges it in
(`SmFilletExecutive::CreateFilletBrep`, then `DoFilleting`). The phases, in order:

1. **Tangent chains** (`ProcessSmoothEdgeSequences`, `CreateSmoothEdgeSequence`). The edges of a
   chain are oriented alike, their curves reparameterised so the joints are C1, and joined into one
   B-spline that every edge of the chain uses as its carrier. Each edge keeps its own fillet, but
   each can march past its ends along a consistent curve.
2. **Corner topology, then geometry** (§6.3), in phases: typed vertices and edges with no geometry,
   positions that do not depend on fillet surfaces, degenerate corners collapsed, setbacks and
   unequal radii adjusted.
3. **Stripes.** Each support surface is wrapped as an offset surface (`SmOffsetSurface`) at the
   radius, allowed to extend past its domain. The ball's centre is a point on both offsets:
   `SmConstantRadiusFS::LoadJacobian` (`SmFilletStandardSolver.cpp`) solves (u1, v1, u2, v2) for
   offset₁ = offset₂, and the marching (`SmFilletSolver::StepSolve`, `SmFilletIntersector`) adds the
   fourth equation as a plane, the same predictor/corrector as the SSI. The contacts are the support
   surfaces at the same parameters: no separate intersection. The trace goes through forced points
   (corner points, the chain's G1 knots) and past the edge's ends where the corner allows it.
4. **Surfaces.** Constant radius between analytic supports tries an exact cylinder or torus first
   (`CalcCylinderFilletGeom`, `CalcTorusFilletGeom`). Otherwise the marched points become cubic
   Hermite spans with one parameter for the spine, both rails and both rails' parameter curves, and
   a cross-section generator skins sections through them (`SmCircularCrossSectionFSG::CreateSurfaceFromCurves`):
   circles approximated by non-rational cubics to 5% of the radius by default, or exact rational
   arcs; chamfers ruled; "blend curves" Hermite of degree 3, 5 or 7 for G1, G2, G3, their end
   tangents faired to equal leg lengths (`SmBSplineCurve::FairBlendCurveDerivatives`).
5. **Trim in by imprinting.** Each rail is merged into its face with its known parameter curve
   (`SmTopologyIntersector::MergeCurveOnSurfaces`); the face/fillet intersection is registered as
   already known. Faces inside the loops of new edges are deleted (`TrimOriginalSurfaces`), the
   fillet body merged, and the result stitched, doubling the stitch tolerance up to five times.

### 6.2 Variable radius

Laws (`SmFilletLaw` and subclasses in `SmFilletExecutive.cpp`) are linear or B-spline (the radius
as the x of a curve). B-spline laws come from (parameter, radius) pairs with a choice of
interpolation: chord-length cubic, or Hermite with zero slope at interior extremes and averaged
slopes elsewhere, with zero slope or zero curvature at the ends. A law is mapped onto each edge
linearly in the edge's parameter; over a chain each edge takes its share of the chain's length.
Outside its domain the law continues as its end polynomial. The solver (`SmVariableRadiusFS`) adds
the edge parameter t as a fifth unknown and requires the centre to lie in the plane square to the
edge at t. So every section is a circle square to the edge, scaled with the radius: never the
envelope of rolled balls. That is why F4 lands 0.97 under Fusion's cone, where ours rolls the ball.
Our laws (natural splines with the Fritsch–Carlson limiter, Fusion's spine-length law on open
chains) are closer to Fusion's.

One trick is worth taking: `SmTemporaryRadiusChange`. To build a corner where a variable fillet
meets others, it makes the law constant at its end radius, builds the corner with the
constant-radius machinery, and restores the law. With a law whose slope is zero at that end, the
stripe then joins the corner exactly. That would lift Limits 2.

### 6.3 Corners

`SmFilletCorner::Create` picks a class from N, the edges at the vertex, and M, how many are filleted:

| M | Classes |
|---|---|
| 1 | `SmFillet1x1Corner` and `SmFilletNx1ClosedCorner` (closed edges), `SmFilletDegenerateCorner`, `SmFilletOpenCorner` (sheet edge), `SmFilletNx1Corner` |
| 2 | `SmFillet2x2Corner` (tangent joint), `SmFillet3x2MixedConvexityCorner`, `SmFilletNx2Corner` (N = 3, 4) |
| 3 | `SmFillet4x3Corner` |
| N = M ≥ 3 | `SmFilletConvexNxNCorner`, `SmFilletConcaveNxNCorner` |

Other combinations (5 × 2, 5 × 3, two adjacent filleted edges of four) report "not implemented".

The corner is described by typed vertices (rail × rail where two rails cross on a face, rail ×
side edge, rail × extended side edge, fillet × fillet, setback, mate) and typed edges (a fillet's
end section, a curve on a face between two rail ends, fillet × end face, fillet × fillet, setback
rail), each with its own `Calc*` routine in `SmFilletGeom.cpp`. For N fillets of one convexity the
hole is bounded by N end sections and N rail × rail points; with a setback each point becomes a
curve on the face and the hole has 2N sides; a bevel adds a centre vertex and N fillet × fillet
edges (a miter). Unequal radii stretch the one rail × rail point whose neighbours' end sections
would cross into a curve (`SmFilletConvexNxNCorner::AdjustTopology`). Two concave edges in a row
get an extra pair of vertices on the face between them (`SmFilletConcaveNxNCorner`). A fillet
ending at a vertex whose side edges differ in convexity extends the side edge of matching
convexity and caps the remaining hole with the end face's surface extended G1
(`SmFilletNx1Corner`, `CreateEndPatch`). A large fillet inside a smaller-radius edge, found because
its two rails run opposite ways, becomes a corner patch (`TestAndConvertBigRadFillet`).

The patch (`SmFilletCorner::CreateCornerPatch`) is analytic when it can be (a plane or bilinear
patch for chamfers, a sphere from three arcs, a surface of revolution for four sides, or a fillet
run between a face and the opposite fillet), else a G1 blend
(`SmBSplineSurface::CreateCornerBlend` in `SmBSplineSurfaceNLib.cpp`): three sides with a
degenerate fourth, four sides as a bicubically blended Coons patch with cross-boundary derivatives
from a tangent field (twist mismatch only warned about), more sides as N quadrilaterals meeting at
a centre point (`N_FillNSidedHole` in `NL_SrfAdv.cpp`), G1 within an angle.

**Against ours.** SMLib covers more corners: four-edge vertices (F8), N-way miters, unequal radii,
mixed convexity at a fillet's end. Ours are better surfaces where we have them: exact spheres and
tori, and the setback corner as six degree-5 quads with exact sides, G1 seams and least-squares G2
(`patch_fit.lucb`). The useful part is the topological recipe: generalise our six-sided fit to 2N
sides, with an end section per fillet and a face curve (or a rail × rail point) per face between
fillets, and take the stretch rule and the concave-pair rule as they are (§11 item 4).

### 6.4 Overflow

After a stripe is built and its rails trimmed, `SmFilletSolver::TestRollOver` classifies each rail
against its face's boundary (`SmFace::CurveOnClassify`), and `ProcessRollOver` acts at each edge a
rail runs into, by that edge's convexity:
- **tangent** to the next face: split the stripe there and re-solve the far part with that rail's
  support switched to the next face (a roll-over onto it);
- **same convexity as the filleted edge**: the fillet runs into the next face, which trims it;
- **opposite convexity (a cliff)**: extend the side face G1 by about the radius and add a piece of
  it between the cliff rail and the side edge (`CreateCliffRollOverSideFG`); rolling the ball along
  the cliff edge is documented but not implemented.
A rail lying wholly outside its face is handled for closed edges only (`ProcessFullRollOver`), and
only one edge per gap. In the runs, SMLib failed F6 (a plate thinner than the radius) as we do; the
case needs a roll-over onto the bottom face across the side face's far edge, which it does not
attempt. The procedure is still the one to build (§11 item 5): we only diagnose and refuse.

### 6.5 Fillet removal

`SmBrep::DeleteFilletFace` (`SmFilletRemoval.cpp`) removes a four-edge strip with B-spline
neighbours. It recognises the strip (four edges alternating tangent, within 2°, and sharp), finds
the new corners by a three-surface Newton between the end face and the two tangent faces with their
domains doubled (`CalcNewCornerVertices`), extends the side edges by marching end ∩ tangent
surfaces to the new corners (`FindSideEdgeExtensions`), and traces the new sharp edge between the
two tangent surfaces (`IntersectTangentSurfaces`). All of it is computed first with the neighbours
*evaluated outside their domains* (the boundary span's polynomial continued), and only then
committed (`DoRemoval`): the surfaces' end spans extended to the new corners, the strip deleted, the
edges glued. Any failure before the commit leaves the body untouched. A corner with more than three
edges keeps its end edge and moves it onto the tangent face. Through the API it did nothing on our
boxes, whose neighbours are planes. The generic fallback, `SmFeatureExecutive::ExtSurfaceRebuild`,
extends every face round a hole G1, stitches the extensions and validates the patch.

## 7. Healing, face removal and stitching

### 7.1 How SMLib does it

**Import healing** (`SmBrep::HealBrep` → `SmHealData`). A fixed order of steps, each working from
property records filled by the steps before (`SmFaceProps`, `SmEdgeProps`, `SmLoopProps`,
`SmVertexProps`): tolerances from measured gaps (faces, then edges at least their faces', vertices
at least their edges'); coincident vertices merged; degenerate faces and edges squeezed; periodic
seams moved out of the way before splitting at them; sheets wrongly gluing two shells split; trim
curves made; the infinite region fixed. New topology created by a step goes back through the earlier
steps (`SmTrackTopologyChanges`). The authors chose a fixed order over independent fixes because one
error cascades into many. Automatic loop repair (`Fix_BadLoops`) is disabled as unreliable.

**Gaps are closed by moving geometry** (`SmGap`, `SmEdge::RefineGeometry`,
`SmVertex::RefineGeometry`): an edge's curve is rebuilt as the intersection of two of its faces
between its vertices, a vertex moved to the meeting point of three of its surfaces, and the
tolerance then shrunk to what is left.

**Stitching** (`SmStitch::DoStitching`), repeated until nothing changes (at most ten rounds): split
edges passing within tolerance of a vertex; find coincident vertices by sorting their projections
on (1, 1, 1) and glue them; glue pairs of edges with the same end vertices whose curves lie within
the stitch tolerance plus their own (`GlueEdgesGeneral`, which can build non-manifold edges when
asked). `StitchIntoSolid` starts at a tenth of the model tolerance and **doubles** each pass while
open edges remain, up to 1/200 of the model's diagonal: tight joins first, so a large tolerance
cannot glue the wrong neighbours. `UnifyNormals` walks faces from a start face and, across an
unmatched edge, accepts a neighbour only when the edges run parallel and the faces' inward
directions are more than 90° apart.

**Delete face.** Three mechanisms. The fillet strip removal of §6.5. Local operations
(`SmBrep::LocalOperation`) that replace face surfaces through the offset engine of §8, which may
change topology. And defeaturing (`SmFeatureExecutive::DoDefeature`): the body is cut along the
feature's loop, and the hole is patched by a planar cap or by `ExtSurfaceRebuild`: every face next
to the loop gets its surface extended G1 (twice the loop's size by default; `CreateExtendedSurface`,
`N_SrfExtendByDist`), the extended faces are intersected with each other
(`SmMerge::PiecewiseMerge`), and the pieces kept are those with an edge on the loop that no other
kept piece duplicates or contains (`sm_TrimPatches`). `ValidatePatch` refuses a patch that is open
anywhere but along the loop or that cuts the body inside a face.

**Surface extension** (`N_SrfExtendByDist`): ruled tangent, curvature-continuous "reflection", or
polynomial continuation of the end span with no new knot; distance turned into parameter by
averaging a few iso-curve extensions; never across a pole; long extensions in halves; retried
shorter when the result is not valid.

### 7.2 Against luce-cad

Our `face_delete`, `face_band` and `face_heal` are exact and predictable where they apply: strips,
corners, holes, bands round a face, caps between planes. In the runs, `deleted_faces` restored the
G2-rounded box exactly where SMLib's API did nothing. Our remaining gap (Limits 9: several faces
beside curved faces, strips apart, inside corners) is where SMLib's extend-and-arrange approach is
general. `face_heal`'s caps are already the planar case of it (a box cut by half-spaces), so the
step is to replace the planes by extended neighbour surfaces and the half-space cut by our booleans'
intersection of the extended patches (§11 item 7).

Our `stitch.lucb` is one pass at a fixed gap and keeps the first face's curve with the gap as the
edge's precision. SMLib's doubling passes, short-edge and sliver removal, the orientation guard and,
most of all, rebuilding a tolerant edge from its faces and shrinking its precision afterwards are
small and worth taking (§11 item 10).

## 8. Offset and shell

### 8.1 How SMLib does it

`SmOffsetExecutive` (`OffsetBrep`, `ShellBrep`) builds an offset body from a **soup of independent
faces** and lets a merge sort it out:

1. **Each face offset on its own** (`SmOffsetGeometryCreation::FaceuseOffset` →
   `SmSurface::CreateOffsetSurface`). Analytic surfaces change radius exactly; when the new radius
   falls under tolerance no surface is made and the face simply drops out. B-spline faces become an
   `SmOffsetSurface`, an evaluator of base + d · normal, with an extended base outside the domain
   and a step-off at singular points. At the end every such surface is fitted as a B-spline
   (`ReplaceImplicitOffsets`, `ApproximateOffsetSurface` on `N_SrfOffset`): exact for quadrics,
   by offsetting the generator for revolutions and extrusions, else sampled, interpolated and
   reduced by knot removal.
2. **Gaps at convex edges.** By default a rolling-ball cap: a circular section swept between the two
   offset rails round the old edge (`BuildOffsetOfEdgeuses`), and a sphere at convex vertices
   (`BuildOffsetOfVertex`). In extended mode the two offset faces are extended and intersected
   instead (`DoExtendedOffset`).
3. **Merge** every piece against every other (`SmMerge::PiecewiseMerge`, skipping pairs known to
   be adjacent), stitch, and **keep every bounded region** (`MakeManifold`). The result is the outer
   envelope of the soup: swallowtails and self-overlaps disappear and topology may change.
4. **Shell** is a boolean: inward, the original less the offset body; outward, the offset body less
   the original. Opened faces are copied into the offset body unchanged, so the difference opens
   there. A thickness under twice the tolerance is refused.

For insets with self-overlap, "keep every bounded region" can keep an inverted pocket; classifying
the cells by winding number is the safer rule.

### 8.2 Against luce-cad

Our `shell.lucb` is the same last step (outer body less inner body), but both bodies come from
`face_moves.lucb`'s substitution: the same topology with new surfaces and vertices re-solved by
Gauss-Newton. That only works while no face vanishes or reconnects, and only for analytic faces,
hence S2 and S3's refusal and Offset Face's "needs planes, cylinders, cones, spheres or tori". The
plan (§11 items 3 and 8): a B-spline offset surface fitted to tolerance (degree 5), and, when
substitution refuses, an offset body built as SMLib does: independent faces, caps, our booleans'
intersection run on one face set, cells kept by winding number. Substitution stays the fast exact
path.

## 9. Tessellation

### 9.1 How SMLib does it

`SmApiTessellate` (`source/SM_API/src/SmApiBrep.cpp`) calls `SmBrep::ConvertToPolyBrep`, which
**copies the B-rep and tessellates the copy by splitting its edges**. That one decision is what
makes the mesh watertight.

**Phase 1** (`SmTess::Phase1SetupBrep` in `SmTess.cpp`):
1. Per face, in parallel: a subdivision tree over the face's parameter box (`SmTessSrfCache::BuildTree`).
   A cell is split, one direction at a time, until its patch is flat enough
   (`SmSurfaceCache::TestAgainstTolerances`): chord error is the thickness of the patch's control
   net hull (a bound, not a sample), angle the turning of its control polygon; the direction split
   is the one with the larger 3D side, error or turn. B-spline faces use their real subdivided nets;
   analytic surfaces approximate 3 × 3 Bézier patches.
2. Pcurves for every edge use, same-sense uses then opposite ones (two parallel passes).
3. Edges whose pcurve is an iso-line are split where they cross the tree's cell boundaries, so
   rectangular boundaries line up with the interior grid.
4. Each edge is split by chord, angle and length (`SmCurveTessDriver::TessellateCurve`). **Every
   sample becomes a real vertex** of the copied B-rep (`MakeVertexSplitEdge`).
5. Per face, each edge use longer than the smallest cell it overlaps is split at its middle
   (`SplitFaceEdgesToUVSize`, `CheckAgainstSurfaceCache`). The split is made on the shared edge, so
   every face using it gets it: boundaries are never coarser than any neighbour's interior.
   Segments tiny in 3D (near a pole) are left alone.
6. A loop over all faces until nothing changes (at most 100 rounds): build each face's parameter
   polygon (`MakePolygon`), split shared edges where linearised loops cross
   (`CheckPolygonIntersections`) or nest wrongly (`CheckPolygonNesting`). Any split re-runs every face.

**Phase 2**, per face: loop segments are implanted into the tree's leaf cells, which are flooded
inside or outside (`MarkFaceContainment`); every inside cell becomes a quad, cells of different sizes
sharing vertices so there are no T-junctions; cells touching the boundary are dropped, and the band
between the quads and the trim polygon is triangulated by scored ear cutting (`TriangulateFace`,
`TriangulateSingleFace`): an ear concave in 2D *or on the surface* is rejected, the score mixes 2D
and 3D angles, and new diagonals longer than their cells are split (on every face sharing them) and
the faces redone.

Output (`OutputFacePolygons`) places each B-rep vertex once, at its 3D point. An edge with more
than two faces is still one edge with one set of samples.

**Seams and poles are fixed in the topology first** (`SmFace::MoveSeam`, `SmFace::SplitAtSeam`):
a periodic face's seam moves into an angular gap of its trims, or a real seam edge is inserted;
missing pole vertices are added. The mesher then always works in the surface's own parameters,
cut to the trim box: a pole is a vertex whose missing side is filled with segments, each row ending
at its own copy of the pole. There is no other chart.

**Failure.** A face fails whole; the others go on, and since edges are shared the hole has no
cracks. `SmTessellationFailure` records the face, the stage (boundary preparation, face, output)
and the status; `SmTessellationReport` counts open and spine edges and whether input and output are
manifold. A partial mesh is returned only when the caller asks and gives somewhere to put the
diagnostics.

Defaults are coarse (25° angle, no chord), and the repository's known issues list inside-out
triangles next to fillets at that setting.

### 9.2 Against luce-cad

Our mesher (luced-3d `docs/CAD_TESSELLATION.md`; `tessellation/`, `meshing/`) has better triangles
where it succeeds (clipped cells, sampled deviation checks, parallel face jobs with a fixed order),
more charts (per-loop unwrapping, a turned hemisphere for spheres, scaled NURBS metrics), and fails
in the places the runs show:

- **"Adaptive patch exceeds face budget"** (T2 and F7 at 64 divisions). From reading the code (not
  traced in a debugger): shared boundary samples are frozen before faces mesh, and `refine_pass` in
  `meshing/adaptive_uv.lucb` splits a triangle whenever its centre lies further from the surface than
  the face's target. A child keeps its frozen boundary edge, whose sag does not shrink, so when the
  boundary is coarser than the interior target the triangle splits on every pass until the budget
  (131,072 faces) stops it. At 64 divisions a cylinder's interior target is about 3e-5 of its radius
  while a rim circle sampled at 256 points sags 7.5e-5 of it. SMLib cannot get there, because
  phase 1 step 5 splits the shared edges down to the faces' interior size before any face meshes.
- **Spheres no chart holds** (Limits 5). SMLib's answer is topological (a seam in the gap, poles as
  vertices). We can do the same without touching the B-rep: choose the sphere's axis away from every
  trim (score a few hundred directions by their angular distance to the trim samples), put the seam
  meridian in a longitude gap, and mesh the rectangle with the seam's samples shared on both sides
  and pole fans (our `spherical_cap` code), the holes as inner loops.
- **Tangential booleans meshing open** (B2). SMLib refines the shared edge and re-runs the faces
  touching it when loops cross or nest wrongly; we can only add points inside one face. A bounded
  "split the shared edge, re-plan those faces" loop closes this.

## 10. UsdSolid: B-reps in USD

### 10.1 The schema

The AOUSD proposal (`proposals/UsdSolid/README.md` and `schema.usda`, draft, library prefix
`PrelimUsdSolid`) stores B-reps on one prim type, `BrepArray`, a `Gprim`. One prim holds one or
more B-reps packed into flat arrays, the way `NurbsCurves` packs curves: each B-rep's entities are
concatenated and delimited by per-B-rep and per-entity counts. The topology is Weiler's Radial Edge
model, the same as SMLib's:

| Arrays | What they hold |
|---|---|
| `brep:intersectTol3d`, `brep:extent`, `brep:regionCount` | per B-rep: one tolerance, its box, how many regions |
| `region:shellCount`, `region:type` | per region: its shells (outer first), `solidRegion` or `voidRegion`; the infinite outside is a void region |
| `shell:faceuseCount`, `shell:wireEdgeCount`, `shell:pointType` | per shell: its faceuses and wire edges, or a lone point |
| `faceuse:faceIndex`, `faceuse:orientationType` | two per face, `same` (the side the surface normal points to) and `opposite` |
| `face:loopCount`, `face:surfaceType`, `face:trimType`, `face:range` | per face: loops (outer first), which surface API gives its shape, `rectangular` or `general` trimming, its parameter box |
| `loop:edgeuseCount`, `loop:vertexIndex` | per loop: its edgeuses, or a single vertex |
| `edgeuse:edgeIndex`, `edgeuse:orientationType`, `edgeuse:nextRadialEUIndex`, `edgeuse:thisRadialEntryType` | per edge-face incidence: the edge, its direction against the edge's curve, the next use round the edge (right-hand rule), and which side of the face the radial walk enters |
| `edge:curveType`, `edge:range`, `edge:vertexIndices` | per edge: the curve API, its parameter interval, start and end vertex |
| `wireEdge:*`, `vertex:pointType` | edges in no face; vertices |

Shapes are multiple-apply API schemas on the prim: `BrepPointAPI` (instances `vertexPoint`,
`shellPoint`), `BrepCurve3dNurbAPI` (instances `edge3dNurb`, `wireEdge3dNurb`; double-precision
points, orders, knots, weights), `BrepCurveUvNurbAPI` (optional trim curves per edgeuse) and
`BrepSurfaceNurbAPI`. The proposal allows only NURBS curves and surfaces so far, and lists the PRC
curve and surface types (circles, cylinders, offsets, blends, …) as future additions. NVIDIA's
working schema in the same repository (`source/schema/omniSolid`) already has `BrepCurve3dLineAPI`,
`…CircleAPI`, `…EllipseAPI` and `BrepSurfacePlaneAPI`, `…SphereAPI`, `…CylinderAPI`, `…ConeAPI`,
`…TorusAPI`. Its validation rules include one tolerance per B-rep, a single outer loop per face
(seam edges required), no faces or edges smaller than the tolerance, and curves running from their
start vertex to their end vertex. Assemblies, B-rep/mesh links and naming are out of scope.

### 10.2 How it maps onto luce-cad

luce-cad's `BrepModel` (`brep/model.lucb`) is already packed arrays: vertices; curves by kind (line,
circle arc, B-spline, ellipse arc) with end vertices; surfaces by kind (plane, cylinder, cone,
sphere, torus, rational B-spline); faces with a support, a sense, signed edge uses (+(edge + 1)
forward) and loop sizes (outer first); one tolerance plus per-edge and per-vertex precision. It is
a manifold model: an edge has two uses, and there are no regions or shells.

**Writing** (`Usd.save` of a set's CAD models, beside or instead of today's one `NurbsPatch` per face
in `convert/export_cad.lucb`):
- one `BrepArray` per CAD model (or one per lump), each lump a solid region with one shell, plus
  the void region holding every lump's outer shell;
- two faceuses per face, the `same` one in the solid region's shell when the face's sense agrees
  with its surface's normal;
- one edgeuse per signed use in a loop; orientation from the use's sign; the two uses of an edge
  linked as each other's radial neighbour, entry types from the faces' senses;
- surfaces as NURBS, converted exactly as `convert/cad_patches.lucb` already does for `NurbsPatch`
  (planes over their loops' box, quadrics as rational surfaces of revolution, B-splines as they are),
  so the file meets the proposal; the omniSolid analytic APIs as an option, since they keep a
  cylinder a cylinder;
- edge curves as NURBS (lines degree 1, arcs rational quadratics), with `edge:range` in that
  curve's parameter;
- trim curves left out (they are optional, and we hold none); `face:range` from the support's
  parameter box over its loops;
- the model's tolerance as `brep:intersectTol3d`; per-edge precision has no place in the schema and
  is folded into it (the largest), with a warning when that loosens the model by more than ten
  times;
- face names and paths, which the schema does not carry, as custom `face:name` and `face:path`
  token arrays, and face colors as `GeomSubset`s, so a round trip keeps names.

**Reading:** each B-rep of a `BrepArray` whose regions are one void and solids, with every edge used
twice, becomes a CAD model:
- vertices from `vertexPoint`; edges' curves from the curve APIs (lines, circles and ellipses
  recognised from rational NURBS when they are exactly one, so our analytic code paths apply);
- each face's support from the surface API; planes, quadrics and tori recognised from NURBS as
  `luce-step` does for STEP's B-spline surfaces, with the omniSolid analytic APIs mapped directly
  (their frames and parameterizations checked against ours);
- each face's sense from which faceuse lies in the solid region; loops from the faceuse's loops,
  each edgeuse becoming a signed use;
- a non-manifold B-rep (an edge with more than two uses, a wire edge, a vertex shell) is refused
  with a message naming it, until luce-cad has non-manifold bodies (§11 item 9); sheet bodies
  (a face used on both sides by the void region) become open surface bodies, which luce-cad has.

**What it takes.**
1. The schema in luce-usd's tables (`prim_definitions.lucb`, `schema_table.lucb`): `BrepArray` and
   the multiple-apply APIs with their namespaced attribute names, so stages compose them like
   any typed prim.
2. A packer and an unpacker in `convert/` (two files, a few hundred lines each), on luce-cad's
   public B-rep accessors; a recogniser of analytic NURBS shared with luce-step.
3. Tests in luce-usd: round trips of a box, a filleted box (B-spline strips), a dome, a drilled
   ball, a two-lump model and an open surface body, checking face and edge counts, volumes, the
   tessellation closed, names kept; the refusal of a non-manifold array; a `.usda` written here read
   back by luce-usd's text and binary paths alike.
4. Outside our repositories, a local check that NVIDIA's reader (`BREP_USD_DATA`, `BREP_SM_USD`)
   loads our files and ours loads theirs (its `TestFiles/usd_TestFiles`); this needs their USD
   build, which this study did not do.

About a week for writing and reading manifold bodies with NURBS shapes, a few more days for the
analytic APIs and the recogniser. The schema is a draft: names may change before it lands in
OpenUSD, so the code should keep the tokens in one table.

## 11. Ranked improvements

Ordered by what each removes for its cost. Efforts are for one person who knows the code. "Cases"
are the comparison cases of §2, which should become luce-cad regressions with the exact or Fusion
numbers given there.

| # | Change | Impact | Effort | Tests that prove it |
|---|---|---|---|---|
| 1 | **Size shared edges from the faces before freezing them** (§9): each face asks its edges for a largest segment from its own interior target; the largest demand wins; `refine_pass` gets a reachable target (at least the sag of its frozen edges) and stops on lack of progress instead of at the budget | High: no more faces dropped on curved faces with holes | 3–5 days | T2 and F7 closed at 64 divisions with no skipped face; the existing 64-division regressions unchanged; triangle counts recorded before and after |
| 2 | **Coincident B-spline faces** (§4): identical nets after knot alignment first, then a grid over one face's chart projected onto the other with distance and normal checks; report as `meet_same` so the existing imprint runs | Closes Limits 6 | 4–6 days | B9: union, common and difference at 1.5, 0.5 and 0.5 of the rounded box; a G2 strip cut by a box flush with it |
| 3 | **Spheres no chart holds** (§9): choose the sphere's axis away from every trim, the seam meridian in a longitude gap, mesh the rectangle with shared seam samples and pole fans | Closes Limits 5 | 4–6 days | A ball with holes all round (T2 and one with holes near both poles) closed at 16, 32, 64, volume converging to the exact B-rep's; existing sphere tests unchanged |
| 4 | **One crossing vertex per piercing, tangency as a point type, and gap repair** (§3, §4, §5): each boundary-edge × face piercing computed once and classified against both solids with a capture distance, every section end snapped to it; tangent points found by a parallel-normal solve and classified from the surfaces' curvatures; a lone vertex squeezed onto its partner within their summed zones before refusing; zones as sums, vertex ≥ edges, precision from measured deviation | High: the tangential contact failures | 2–3 weeks | B4 closed and exact; B3, B5 unchanged; B2 meshes closed or is refused; B1 and B6 refused with a message naming the non-manifold edge; a G2-rounded box touching a plane along a strip |
| 5 | **March in both surfaces' parameters** (§3): (u1, v1, u2, v2) predictor, plane-constrained damped Newton, Hermite acceptance in 3D and both charts, local step growth; 3D curve plus two parameter curves; seeds from exact iso-boundary/surface intersections and Bézier leaf pairs with a working normal-cone test | High: B-spline booleans faster and steadier; parameter curves for clipping | 1.5–2 weeks | `spline_boolean_tests` and B8 adding up; B8 time recorded (34 ms today); two bicubic patches with a known intersection (a small closed loop inside both) found and within tolerance |
| 6 | **B-spline offset surface and a surface extension operator** (§7, §8): base + d · normal evaluated with a step-off at singular points, fitted degree 5 to tolerance; natural growth for analytic surfaces, end-span continuation for B-splines | Offset Face, Move and Shell on B-spline faces whose topology does not change; a base for 8, 9, 10 | 1.5 weeks | S2 and S3 run; shell volume plus the inner offset body equals the outer body; Offset Face on a G2 strip moves every point by d (sampled); the refusal message no longer blames the wall |
| 7 | **Fillet corners of N edges** (§6.3): N end sections and N rail × rail points (or face curves with a setback); our six-quad fit generalised to 2N sides; the stretch rule for unequal radii and the concave-pair rule | Closes Limits 1; four-edge vertices, N-way miters | 2–3 weeks | A four-sided pyramid with every edge filleted (F8: closed, G1 seams measured at the patch boundaries); a box corner where four fillets meet after a boss; three unequal radii at a corner |
| 8 | **Overflow by rail classification** (§6.4): after a stripe is built, classify each rail against its face; tangent edge → switch support and continue; same convexity → trim; cliff → extended side face as a cover | Large radii on small faces stop being refusals | 6–9 days | F6 (plate 4 thick, r 4.5 and 5) closed, volume against a sampled section integral; a fillet rolling onto an earlier fillet's strip |
| 9 | **Delete healing by extended neighbours** (§7): extend every neighbour past the gap, intersect the extended patches with the booleans' face/face step, keep pieces on the gap loop not contained by another, validate | Closes Limits 9's healing part | 1–2 weeks after 6 | Two strips apart on a G2-rounded box deleted (8000 minus the other two strips); an inside corner between a boss and a curved wall; failures leave the body untouched |
| 10 | **Variable fillets at corners** (§6.2): build the corner as if the law were constant at its end radius, with the law's slope zero there | Lifts Limits 2's corner part | 3–5 days | A box corner of three fillets, one of them 2 → 4 into the corner: closed, G1 at the seams, the variable strip's end section equal to the corner's |
| 11 | **Offset body from a face soup** (§8): independent offset faces, caps, the booleans' intersection run on one face set, cells kept by winding number; used when substitution refuses | Shell and offset when faces vanish or reconnect | 2–4 weeks | A shell thicker than a fillet's radius (the strip disappears); an inset that swallows a small face; volumes against a mesh offset of high resolution |
| 12 | **Stitch passes and edge refinement** (§7): tolerance doubling from a tenth of the gap, short-edge and sliver removal, the orientation guard; afterwards each tolerant edge rebuilt as its faces' intersection and its precision shrunk | Fewer wrong joins; precise edges after Stitch | 1–1.5 weeks | Stitch of a box whose faces are moved apart by 1e-4, 1e-3 and mixed: one solid, edge precision back under the model tolerance; a near-neighbour that must not be glued |
| 13 | **Find where the boolean time goes** (§2, P1): 1975 ms against 290 ms for 100 holes | Speed of every feature | 2–3 days to measure, then targeted fixes | P1 time recorded per stage; the per-hole time flat as the plate fills |
| 14 | **UsdSolid write and read** (§10) | B-rep round trip through USD | 1–1.5 weeks | The round trips listed in §10.2 |
| 15 | **Non-manifold bodies** (§1.2): regions and shells in `BrepModel`, an edge with 2k uses | B1, B6 and two boxes sharing a face as results, not refusals | Large (a month or more); only once 4 is done | B1, B2, B6 results with SMLib's volumes; UsdSolid's non-manifold cubes read |

### Status

What has landed since the study, item by item (the runs of §2 repeated with the same drivers).
"Limits n" in this study numbers MODELING.md's list as it was when the study was written.

- **1, shared edges sized from the faces (done).** Each curved analytic face asks its edges for
  its interior target before the samples are frozen (`layout/face_targets.lucb`), the
  refinement's target beside a frozen side is that side's own sag, and it stops when a pass stops
  helping or at the face budget, keeping the mesh it has (`meshing/adaptive_uv.lucb`). Tracing
  the dropped faces found a second cause the reading had missed: the Lawson flips after grid
  insertion ran at most 128 sweeps, which left fans of slivers 17 units long across the pipe of
  F7, and the refinement could not split them away within its budget. The flips now run from a
  queue (`polygons/improve.lucb`). T2 and F7 are closed at every division from 4 to 64 with no
  face skipped (`division_range_tests`): T2 3597.155 (SMLib 3597.120), F7 3432.78 and the T
  before its fillet 3430.63 (exact 3431.022; at 64 divisions the long pipe's face takes the
  trimmed chart, whose target is relative to the face's length). Side effects in the table: T1
  went from 0.50 to 0.006 under the exact volume, and the rounded corners' barycentric grids
  (now placed on great circles and sampled to their stretch) took F3 from 0.0019 to 4e-6 off
  exact with a third of the triangles (218,124 to 72,956). Triangles at 64: T2 93,540 (six faces
  missing) to 510,396; F7 152,894 (one missing) to 281,574.

- **2, coincident B-spline faces (done).** A lattice of one surface's points projected onto the
  other decides it (`modeling/spline_coincidence.lucb`); the existing imprint then runs, and flush
  pieces on B-spline faces stay separate faces (no surface reaches over the other's part). B9:
  union, common and difference closed, at 1.5, 0.5 and 0.5 of the rounded box to 1e-4 by mesh
  volume (11495.2750, 3831.7581, 3831.7581 against 11495.2745 and 3831.7582), 61 ms each against
  SMLib's 20. B8 unchanged (36 ms). Regression: `spline_boolean_tests`, with a box flush with the
  rounded box's side.

- **3, a sphere chart for any trims (done).** `meshing/sphere_axis.lucb`: an axis of the face's
  own (the farthest of 400 directions from every trim), a band between a loop round it or a polar
  circle below and above (rows and a fan close each pole the face holds), holes inside, a seam
  crossing no trim; a fallback after the sphere's own chart and a local hemisphere. A sphere zone
  whose circles have no vertex on the seam meridian is now left for it instead of refused by the
  boolean. A ball with six holes along the axes (through both poles) is closed at every division
  from 4 to 64 and extrapolates to 3743.952 (exact 3743.992); T2 unchanged.

- **4, contact (part).** Curves of one face pair are cut where they cross each other, at one vertex
  (`mutual_crossings` in `boolean.lucb`): B4, two equal pipes crossing, now joins closed with 8
  faces (SMLib's count) in 7 ms; its mesh volume at 32 divisions is 4045.637 (exact 4045.722).
  Tangent contact sections are noted, and the results that would be non-manifold are refused
  with a message naming it: B1, B2 and B6 (B2 meshed open before, silently), with a check that
  no kept face passes through an edge of the result. B3 and B5 are unchanged. Regression:
  `touching_tests`. Still open: tangency classified from the surfaces' curvatures, gap repair,
  zones as sums of tolerances.

- **13, where the boolean time goes (part).** P1 profiled: per hole the cost grew with the
  plate's faces (46 ms for the first ten holes, 390 ms for the last ten), and over half of it was
  finding a point well inside a piece (`piece_point`), asked even of the plate's face with all its
  holes: the walls far from the new hole were classified outside by their box alone, which did not
  record the class for their connected block. Now a face classified by its box classifies its
  block, the block's class is asked before any point inside, each block is classified by its
  piece with the fewest boundary uses, and a few candidates come first (a point a quarter of the
  piece's narrower extent inside is enough). P1: 1975 to 790 ms (SMLib 290 ms, single-threaded).
  What remains: every face of the plate split again for each hole (`face_pieces`, `split_face`,
  about half of what is left) and the face-pair loop (`intersect_faces`, a third).

## 12. What not to take

- **Its surfaces.** Cubic approximations of circles by default, G1-within-an-angle corners, Coons
  patches with unresolved twists, faired Hermites instead of Fusion's sections, scaled sections
  instead of rolled balls. Ours are exact and match Fusion; keep them.
- **Its laws.** Parametric within each edge; ours follow Fusion more closely.
- **Its seeding gaps.** No interior tangential seeds, branches from a starting branch point not
  queued, starts at poles skipped, the normal-cone test switched off, `ComputeStepSize` a stub.
- **Its constants as truths.** 10°, 0.5°, 8 steps, 20 iterations in the sector test; 20° marching
  angle; 5% arc approximation; 25° default meshing angle. Starting values at most.
- **Its trim curves with every knot triple**: large; fit instead, at degree 5.
- **"Keep every bounded region"** for insets: use winding numbers.
- **Its shape of code.** `SmMerge::ManifoldBoolean` is one 2,600-line function; `SmBrep.cpp` is
  39,000 lines. Our files stay small.
- **Full Radial Edge everywhere** before it is needed. Non-manifold structure can live inside the
  booleans' working shape first (for the arrangements of items 9 and 11); a public non-manifold
  body is item 15.
