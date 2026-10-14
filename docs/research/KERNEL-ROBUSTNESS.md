# Kernel Robustness: tolerances, determinism and boolean consistency

Status: the owner approved the design and stages 1–7 on 2026-10-13, with the decisions in section
7. Stage 0 (the harness, 60a9332) has landed; section 6 tracks the stages as they land.

The owner wants luce-cad to stand with Parasolid, ACIS and Alias. He does not want it to behave like
NVIDIA's usd-brep/SMLib, which a review describes this way: "internally inconsistent booleans, loose
unprincipled tolerances everywhere … nondeterministic geometric operations … your model may be
manifold on Monday but not on Tuesday". This document has seven parts:

1. what the best kernels do;
2. where luce-cad stands, with evidence;
3. what the harness measures today;
4. one tolerance model for luce-cad, built around the owner's decision of 2026-10-13: tolerances
   are a per-project setting stored next to the units, and the positional tolerance defaults to
   0.01 mm;
5. a determinism contract;
6. a staged plan;
7. the few questions only the owner can answer.

---

## 1. How the best kernels do it

### Parasolid (Siemens; NX, Solid Edge, SolidWorks, Onshape)

- **Session precision.** Linear resolution is 1e-8 and angular resolution is 1e-11 rad. Both are
  fixed for the whole session and the application cannot change them. Model units are meters, so
  1e-8 means 10 nm.
- **Size box.** All geometry must lie inside a cube 1000 units on a side (±500 m). The ratio of
  box to resolution is 1e11, which keeps every coordinate's rounding error (about 1e-16 × 500) at
  least three orders of magnitude below the resolution. The angular resolution is the linear one
  divided by the box (1e-8 / 1e3 = 1e-11). So "parallel" and "coincident" are one decision: a tilt
  that cannot move a point by the resolution anywhere in the box is no tilt.
- **Tolerant modelling** (D. J. Jackson, *Boundary representation modelling with local
  tolerances*, ACM Solid Modeling '95). An edge or vertex whose geometry cannot be made accurate to
  the resolution, for example after import, a hard intersection or a sewing, gets a **local
  tolerance** of its own. The edge then counts as lying on its faces within that tolerance, and
  every operation that touches the edge uses it. The tolerance is local: it never widens the model
  or the session. Operations make entities tolerant only where they must and report it.
- **The checker** (`PK_BODY_check`) tests a body at increasing cost. It checks the data
  structure, that geometry lies within tolerance of its topology, face orientation and loop
  closure, and finally self-intersection. A body that fails is invalid, and operations may refuse
  invalid inputs.

### ACIS (Spatial; the base of Autodesk ShapeManager, which Fusion runs on)

- `SPAresabs` (default 1e-6) is the distance below which two points are the same.
- `SPAresnor` (1e-10) is the angular and numeric resolution.
- `SPAresfit` (1e-3) is the fitting accuracy for approximated curves and surfaces.
- These are session settings but are documented as constants: changing them mid-model is
  unsupported.
- `TEDGE`, `TVERTEX` and `TCOEDGE` are tolerant entities. They carry their own tolerance and are
  created by healing, import and some booleans; `api_optimize_tvertex_tolerance` tightens them
  again.
- Fusion inherits all of this. Its results are deterministic and exact to the resolution; only
  fitted geometry is approximate.

### Open CASCADE (OCCT)

- `Precision::Confusion()` is 1e-7 and `Precision::Angular()` is 1e-12. Every vertex, edge and face
  carries a tolerance, with the invariant tol(vertex) ≥ tol(edge) ≥ tol(face). `BRepCheck_Analyzer`
  validates a shape.
- **Its failures are the lessons.**
  - Tolerances only grow. `SameParameter`, sewing and booleans each widen tolerances to make their
    output "valid", and the next operation inherits the larger values. After a few operations a
    model carries millimetre-sized tolerances on micrometer features.
  - The fuzzy boolean (`SetFuzzyValue`) adds a user value to every tolerance, which repairs one
    case and changes others.
  - Classification and intersection are computed against different tolerances in different
    algorithms (BOP's "DS" against `IntTools`), which produces the well-known "unable to classify"
    and inconsistent-result reports.
- What OCCT gets right is the General Fuse structure: one intersection and imprint stage records
  every interference once, and every later stage reads it. luce-cad already follows this
  structure; it does not yet share decisions the way OCCT's data structure does.

### Alias (Autodesk; class-A surfacing)

- **Construction Options** hold one master set of tolerances for every tool:
  - Curve Fit Distance: points closer than this are one point, and fills, rails and fillets are
    fitted to it.
  - Maximum Gap Distance: when two edges count as touching (Stitch, continuity checks).
  - Continuity Angle: G1, normals that count as the same.
  - Continuity Curvature: G2, (R1 − R2)/(R1 + R2).
  - Topology Distance: adjacency for Check Model.
  - Trim Curve Fit.
- **Presets.** There is one preset per downstream CAD system (CATIA V5, Inventor, "General
  CAD"). Autodesk's guidance is to choose tight tolerances before you start, because a tolerance
  governs how geometry is built, not only how it is checked. A typical Maximum Gap Distance is
  0.01 mm to 0.001 mm, and a tangent break under 0.1° is normally invisible.
- **Alias compared with Parasolid.** Alias's tolerance is a modelling contract the user chooses.
  Parasolid's precision is a fixed numerical floor, with local tolerances where geometry cannot
  meet it. The two coexist. Alias builds geometry to a user tolerance and hands Parasolid-based
  systems data that those systems mark as tolerant where needed.

### Robustness literature

- **Exact predicates, approximate constructions.**
  - Shewchuk's adaptive predicates (orient2d, orient3d, incircle; *Adaptive Precision
    Floating-Point Arithmetic and Fast Robust Geometric Predicates*, 1997) give the exact sign of a
    determinant at near-floating-point cost.
  - CGAL's `Epick` kernel is built this way. Interval filters answer most cases cheaply and fall
    back to exact arithmetic only when the interval contains zero.
  - The sign that decides topology is exact. The points that are constructed are rounded but never
    re-decided.
- **Symbolic perturbation.** Edelsbrunner and Mücke, *Simulation of Simplicity*, 1990. Ties such as
  a ray through an edge or a point on a plane are broken by a consistent infinitesimal
  perturbation, so a degenerate case has one answer that is the same however it is reached. This
  replaces "cast three rays and vote".
- **Consistency over accuracy.** Hoffmann, *The problems of accuracy and robustness in geometric
  computation*, IEEE Computer 1989; Segal and Séquin, *Consistent calculations for solids
  modeling*, 1985; Sugihara's topology-oriented approach. A kernel fails when it makes two
  decisions about one geometric fact and they disagree, not when a coordinate is slightly off. The
  remedies are:
  - make each topological decision once and record it, and derive every dependent decision from
    that record;
  - when numbers are uncertain, keep the topology consistent and let the geometry absorb the
    error, which is what tolerant entities do.
- **Certified intersections.** Interval Newton and subdivision with exclusion tests (Bernstein
  bounds, Krawczyk) prove that a region contains no intersection, or exactly one. Marching from
  seeds cannot prove either, so it can silently miss a branch.

---

## 2. Audit of luce-cad (2026-10-13, origin/main db29f98)

### 2.1 The tolerance model today

There is a tolerance model, but it is relative to size and is redefined at each stage:

- A new model's tolerance is a millionth of its extent: `tolerance_for(size) = max(1e-9, size ×
  1e-6)` (`src/geometry/basics.lucb:41`). A box sized 10 has 1e-5 and a box sized 1000 has 1e-3.
- A boolean works to `work.tol = max(size × 1e-7, 1e-10)`, where size is the diagonal of both
  operands together (`src/modeling/boolean.lucb:320`). It is redefined the same way in `trim.lucb:63`,
  `face_meets.lucb:34`, `face_split.lucb:86` and `face_heal.lucb:37`, and as `1e-7 × max(1, size)`
  in `split_body.lucb:53`.
- A boolean writes its result at `max(work.tol × 20, precision × 2)` (`boolean_result.lucb:271`).
- Sketches work to `size × 1e-4` (`modeling/api.lucb:34`, `profile_fills.lucb:49`) and, in other
  files, to `1e-6 × max(1, size)` or `1e-9 × size` (`sketch/projection.lucb:164`,
  `sketch/crossings.lucb:44`).

As a result, **whether two faces 0.005 mm apart are one face depends on how big the whole model is,
and on what else takes part in the operation**. A distant body joined into the same boolean loosens
every decision in it. The harness band table (section 3) shows this.

### 2.2 The literals

`src/` has **945** small literals, classified one by one; the table is reproducible with the
script in section 3.4.

| class | count | absolute (unit-dependent) | × size | × tolerance | unitless | mixed |
|---|---|---|---|---|---|---|
| guard (division, zero length) | 188 | 144 | 10 | 0 | 34 | 0 |
| angular | 155 | 0 | 3 | 0 | 151 | 1 |
| parameter / chart | 135 | 28 | 9 | 0 | 84 | 14 |
| model distance (coincidence) | 131 | 16 | 85 | 2 | 1 | 27 |
| display / meshing | 126 | 9 | 34 | 15 | 46 | 22 |
| solver (convergence, steps) | 82 | 4 | 33 | 8 | 32 | 5 |
| fitting | 36 | 4 | 10 | 0 | 10 | 12 |
| other (ratios, probes) | 92 | 14 | 27 | 0 | 47 | 4 |

The multipliers applied to `tol` in the boolean are almost all whole factors that change from stage
to stage: ×5 (vertex zone), ×10 (in a dozen places), ×20 (result), ×100 (edge identity), ×1000 (edge
clear of a surface). Angular tests come in two forms with very different meanings:

- the cross form `|a×b| < e` means an angle of about e;
- the dot form `1 − |a·b| < e` means an angle of about √(2e). So 1e-12 means 1.4e-6 rad, 1e-9 means
  4.5e-5 rad and 1e-6 means 1.4e-3 rad.

The two forms are mixed on adjacent lines.

### 2.3 Findings, by severity

**S1: decisions that can contradict each other inside one operation**

1. **Planes parallel, tangent or coincident: three rules in one boolean.**
   - `intersect.lucb:61` calls planes parallel at |n×n| < 1e-7 rad and coincident at offset ≤ 1 tol.
   - `boolean.lucb:616` (`note_tangency`) calls faces tangent at 1 − |n·n| < 5e-7, about 1e-3 rad,
     and requires a crossing to part by more than 10 tol (`:626`).
   - `spline_coincidence.lucb:42,73` calls surfaces coincident within 10 tol and a dot of 1 −
     1e-6, about 1.4e-3 rad.
   - So a plane and the same plane as a B-spline get opposite answers at 2–10 tol or at tilts of
     1e-7 to 1e-3 rad. Planes tilted by 1e-7 to 5e-5 rad get a section line from `meet`, which
     `note_tangency` then records as a tangency.
   - The harness confirms it: the tilt band fails (E) at 1e-6 and 1e-5 rad at every size.
2. **"Is p on face f's boundary" is answered at four radii in one boolean.**
   - `boolean.lucb:446` gives 10 tol.
   - `hugs_boundary` (`boolean.lucb:595`) passes `tol × 10` into `lookup_contains`, which multiplies
     by 10 again (`face_lookup.lucb:59`), so 100 tol. Its comment says ten.
   - `rays.lucb:157` uses the input's model tolerance, about 100 work tolerances.
   - `trim.lucb:196` uses `max(tol × 1000, size × 1e-3)`.
   - A stretch can be cut as crossing a face's interior by one test and judged to hug its boundary
     by the next.
3. **Tangent contact at 1 tol, slivers kept at up to 20 tol.**
   - `meet` decides tangency at 1 tol: plane/cylinder `intersect.lucb:91`, plane/sphere `:124`,
     cylinder/cylinder `:135`.
   - A plane a few tol inside tangency therefore makes two section lines bounding a face thinner
     than the result's own tolerance (×20, `boolean_result.lucb:271`).
4. **Revolution axes.**
   - Plane against an axis "across" uses a dot of 1 − 1e-12 (1.4e-6 rad): `intersect.lucb:80`,
     `:115`, `:212`.
   - "Along" the axis uses 1e-12 rad: `:85`, `:107`, `:205`.
   - Coaxial uses 1e-9 rad: `:130`, `:231`, `:261`.
   - A cylinder whose axis lies in a plane to 1e-11 rad becomes an ellipse with semi-major axis
     r/tilt, up to r·1e12 (`:99–104`), where plane/plane would have said "parallel".
   - A blend tool accepted as turned about an axis at 4.5e-5 rad (`curved_turned.lucb:14`) is not
     coaxial for `intersect` (1e-9 rad), so it is marched instead of being cut in circles.
5. **Ray classification by majority vote** (`rays.lucb:133`). Three fixed world-frame rays vote.
   - A hit within a billionth of the size of the start is ignored (`:148`).
   - An analytic hit on a face's boundary band counts as a hit (`:157`), so a ray through an edge
     shared by two faces counts twice.
   - B-spline hits use exact winding with no band (`:149`).
   - The outcome depends on the orientation of the model (rays are fixed in the world), and
     parity can differ between an analytic face and the same face as a B-spline.
   - **Harness:** a box minus a sphere whose center lies exactly on the box's face gives a volume
     of 754.45 instead of 871.73. The topology is the same, the volume is wrong by 13%, and moving
     the sphere by 1e-7 gives the right answer. That is a degenerate case decided wrongly, the kind
     symbolic perturbation exists for. It shows up in the harness as 5 of its 6 invariance
     failures.
6. **Vertex merging is not transitive and depends on order.** `boolean.lucb:124–133` merges a
   point into the *first* vertex within the *sum* of both zones (5 tol each), and the zone then
   grows. The comment cites "SMLib's rule". A, B, C 8 tol apart in a row merge or not depending on
   arrival order.
7. **Edge identity at 100 tol** (`boolean.lucb:166`) ignores the edges' own precision. The first
   carrier pushed wins (`final_edge`).

**S1: tolerances that leak across the model**

8. **One tolerant edge loosens the whole model.**
   - `shape_of` takes `b.model_tolerance()` (`shape.lucb:256`), which is the largest edge or vertex
     precision anywhere (`brep/records.lucb:9`). Every `w.tolerance` user then sees it: clip,
     lookup, rays, merging, contact, blend, face_split.
   - `brep_of` writes it as the new model's tolerance. This is the OCCT tolerance-growth failure
     that Stitch's own comment (`stitch.lucb:18–20`) says it avoids.
9. **Operand B's tolerance is dropped.** `append_shape` (`boolean.lucb:268`) keeps B's slacks but
   not its tolerance. Non-tolerant entities of a looser B get no slack, and work.tol ignores both
   inputs (`boolean.lucb:320`), although the comment at `clip.lucb:378` says the inputs' own
   tolerances are honoured.
10. **Stitch works from a doubling gap** (`stitch.lucb:7`, "SMLib's passes of a doubling
    tolerance"; work.tol = gap × 0.1 at `:61`). This is the reviewed kernel's pattern, not
    Parasolid's measured local tolerances.

**S2: invariance**

11. **Scale.** Booleans are invariant under scale today because everything is relative. That is
    exactly why they are wrong for an engineer: 0.005 mm means "the same" at 1 m and "different" at
    1 mm. The band table shows it, and Fusion and Parasolid do not behave this way.
12. **Rotation.** The fixed world-frame ray directions (`rays.lucb:133`), the world-axis box
    filters (`boolean.lucb` `boxes_meet` with `tol × 10`) and the probe points in chart space
    (`classify.lucb` 0.0098765432, 0.0123456789) are deterministic but not rotation-invariant at
    degenerate configurations. The harness found no rotation failures in its pairs, and the "turned"
    band rows equal the unturned ones at every size (the X and E cells at 1 m come from size, not
    rotation).
13. **Operand and face order** (MEDIUM; deterministic, but the result depends on the order):
    - first-in-range vertex merge (above);
    - `final_edge`'s first carrier;
    - Stitch's greedy merge with the first point as representative (`stitch.lucb:507–587`);
    - the march's seed order and its Newton seed carried from the previous query (`march.lucb:13–48`,
      `:110–153`), which makes the start, direction and sampling of a traced curve depend on which
      face is `fa`;
    - `arrangement_graph.lucb:70`, `face_delete.lucb:208`;
    - `blend_tools.lucb:505`, where the last match wins.

    Harness: union and intersect of the four pairs are invariant under swapping and face reversal
    in topology and volume. The one failure is in the tessellation: the cylinder-minus-cylinder
    result with A's faces reversed meshes to a volume of 143 at 64 segments, 490.87 at 32, while
    the right volume is 490.87. The B-rep is right and the 64-segment mesh is wrong, depending on
    face order.

**S2: marching can lose branches.** `march.lucb:122,134` skip any seed within 0.008 × size of a
traced curve. Distinct branches closer than that are lost, and the operation then fails as "faces
do not meet edge to edge". Points are accepted at 1 tol (`:73`) but must fit within 0.5 tol
(`:194`), and after five tries the fit is accepted anyway (`:150`). Nothing certifies that a pair
has no further intersection.

**S3: absolute, unit-dependent literals.**
- There are 144 absolute guards. Most are harmless denominators, but they are written as lengths
  (`max(x, 1e-12)` on a distance).
- Absolute lengths with real effect:
  - the zero-extrude test of 1e-9 (`extrude.lucb:54`, `extents.lucb:31`);
  - planarity at 1e-6 "source units" (`model/model.lucb:181`);
  - Stitch's default gap of 0.001 and its cap of 1 unit (`surfacing/api.lucb:465`,
    `stitch.lucb:42`);
  - the Levenberg–Marquardt damping of 1e-12 in length² (`face_moves.lucb:148`);
  - knot equality at absolute 1e-9 (`surfacing/builders.lucb:189–306`, `strip_join.lucb:63`)
    next to relative `span × 1e-12` (`nurbs/splines.lucb:157`).

**S3: values justified by a failing case.**
- Comments: `geometry/inversion.lucb:72–79` ("seed-dependent when STEP curves differ"),
  `layout/mapped_extrusion.lucb:129` ("STEP edges can disagree within import uncertainty"),
  `meshing/surface_trim.lucb:20` ("microscopic UV ears").
- Max/min of unrelated quantities:
  - `blend_fits.lucb:343`: tol × 1000 + depth × 1e-6;
  - `shell.lucb:508`: r × 1e-6 + tol × 10;
  - `square.lucb:285`: max(tol × 100, size × 1e-4);
  - `boolean_result.lucb:271`: min(1, max(tol × 20, precision × 2, least)).

  These are the "bumped until it worked" pattern of the review. They are fewer and better commented
  than in SMLib, but they are there.

### 2.4 Determinism: what holds

The audit read every parallel site, sort, hash table and source of randomness. All of the
following hold, and the harness confirms them:

- **Parallelism.** There are 13 parallel sites in luce-cad and the luce-geocore code they reach.
  All write results into slots indexed by item. Per-face errors are raised in face order, and
  reductions are serial in index order. No site uses the scheduling-dependent `worker` number for
  anything but scratch.
- **Hashing.** luce-base's `.hash()` is seeded from an address (`luce-base src/std/core.lucb:1145`,
  so it differs per run under ASLR). luce-cad never calls it. The geocore tables that do
  (`TableBuilder`, `groups/cache`) are lookup-only and emit in first-seen index order. luce-cad's
  own tables use fixed constants and are probed, never iterated into output.
- **Sorting.** All 14 sort calls sort scalars by their natural, total order; luce-std's introsort
  has no random pivot. The 10 hand-written insertion sorts compare strictly and are stable.
- **No clock or random input** feeds geometry. Addresses are used only for memory accounting.
- **Floating point across OSes.** luce-std's sin, cos, atan2, exp, log and pow are written in Base
  and give the same bits on every target. The platform C library supplies only correctly rounded
  functions (sqrt, floor, fma and similar). The compiler passes `-ffp-contract=off` and never
  reassociates.
- **Measured.** Fingerprints of seven operations are bit-identical across runs, across four
  concurrent threads, and across **macOS arm64, Linux x86-64 and Windows x64**.

Small items, none needed for determinism:

- the trim-crossing memo key folds two z values into one word (`layout/trim_crossings.lucb:21`);
- the face-cache hit is decided by a 64-bit key alone (`tessellation/face_cache.lucb:189`);
- only OOM paths report a scheduling-dependent first error.

So the reviewed kernel's "Monday/Tuesday" problem does not exist in luce-cad. Its problems are the
inconsistent epsilons and the order-dependent representatives above.

### 2.5 Per-entity tolerances

The machinery exists: `edge_precision` and `vertex_precision` (`brep/model.lucb`), set by Stitch
and by traced sections. What is missing:

- it is used globally (findings 8 and 9);
- boundary tests ignore vertex slack (`clip.lucb:455`, `face_lookup.lucb:59`);
- edge identity, curve closure, imprint and crossing tests use the global tol where the entity's
  own precision applies (`boolean.lucb:166`, `:403`, `:447`; `boolean_result.lucb:394`);
- import has no measured per-entity tolerance. The STEP tolerance is one model-wide number
  (luced-3d's "STEP tolerance" field, `node_catalog.luc:332`);
- there is no checker that validates "geometry within tolerance of topology" on a whole body.

### 2.6 The boolean's decision flow

Stages: shape → intersect → vertex cuts → clip → shared edges → split → classify → select → merge.
Each stage asks its geometric questions again, with its own epsilon:

| fact | stage and epsilon |
|---|---|
| point on surface | intersect 1 tol; clip 10 tol; trim 1000 tol |
| point on face boundary | 10 / 100 / ~100 tol (above) |
| surfaces coincident | 1 tol + 1e-7 rad; 10 tol + 1.4e-3 rad |
| vertex identity | sum of 5-tol zones |
| edge identity | 100 tol |
| edge clear of a surface | 1000 tol (`classify.lucb:236`); 10 tol (`boolean_result.lucb:394`) |
| piece inside the other solid | three fresh rays, not the face/face facts already known |

OCCT's lesson, and Hoffmann's, is that each of these is a **fact**, decided once, stored, and read
by every later stage. Section 4.4 gives that design.

---

## 3. Baseline (harness of 2026-10-13, commit 60a9332)

`tests/robustness` runs in about 30 s within `luc test`. It prints every failing property and fails
if any count rises above the baseline, so counts can only go down.

### 3.1 Determinism: holds

Seven operations: box − tilted cylinder, box ∪ sphere, cylinder ∩ crossing cylinder (traced),
torus − tilted box (traced), all-edge fillet, shell, filleted box − cylinder (B-spline strips
against analytic). Each is run 3 times in a row and 4 times at once on threads, and each gives one
fingerprint:

```
box-cylinder           3604635975276724936
box+sphere             3234145142419964986
cylinder*cylinder     16914755356991419701
torus-box              8836276416437977261
filleted box           5709280193451373943
shelled box            5000135758701244965
filleted box-cylinder 14130126335904911820
```

The same numbers appear on macOS, Linux and Windows. The whole harness output is identical on
the three, band tables included.

### 3.2 Invariance and identities: 6 of 224 checks fail

The pairs are boxes, box/cylinder, box/sphere and cylinder/cylinder. For each operation they are
checked with the operands swapped, with the faces reversed, moved and turned, and scaled ×25.4,
×0.001 and ×1000, against topology signature, volume and the two volume identities.

| failure | cause |
|---|---|
| box/sphere subtract: moved, ×25.4, ×0.001, ×1000 differ by 15.5%; V(A−B) + V(A∩B) ≠ V(A) by 11.7% | the unmoved case is wrong: sphere center exactly on the box's face (finding 5) |
| cylinder/cylinder subtract, faces reversed: mesh volume off 71% | tessellation at 64 segments depends on face order (B-rep correct) |

Stage 1 changed the scales to ×25.4, ×0.1 and ×100. The old ×0.001 made the
features 0.002–0.01 mm, at the positional tolerance itself, where no fixed outcome is
right, and ×1000 made a 10 m part. The stage 1 baseline with the new scales is 5 of 224:
the box/sphere case 4 times, and the face-order tessellation case once.

### 3.3 The tolerance band: 169 of 372 cells off the ideal, 5 mixed, 22 failed

Two boxes are placed with faces δ apart:

- **flush:** side faces offset sideways, along y, along the diagonal, and with everything turned;
- **gap:** end faces apart, negative values overlap;
- **tilt:** a face turned by δ rad.

The sizes are 1, 10, 100 and 1000 mm, and both union and subtract are run. The offsets are 0,
0.001, 0.005, 0.009, 0.0099, 0.01, 0.0101, 0.011, 0.02 and 0.1 mm. Each cell is classed:

- M: merged, as at δ = 0;
- D: kept apart;
- X: mixed or slivered;
- E: failed.

With P = 0.01 mm, the ideal is M up to P and D beyond it.

```
flush y union        1 mm: MDDDDDDDDD   ideal MMMMMMDDDD
flush y union       10 mm: MDDDDDDDDD
flush y union      100 mm: MDDDDDDDDD
flush y union     1000 mm: MXDDDDDDDD   (subtract: MEDDDDDDDD)
gap union            1 mm: MMMMDDDDDDDDD  ideal MMMMMMMMMDDDD  (−0.1 … +0.1 mm)
gap union         1000 mm: MMMMEDDDDDDDD
tilt union           1 mm: MMMMEEDDDD   (0, 1e-9 … 1e-1 rad)  ideal MMMMMMMMMD
tilt subtract        1 mm: MMMMEEEDDD
tilt union        1000 mm: MMMMXEDDDD   ideal MMMMMMDDDD
```

Every flush and gap row looks the same from 1 mm to 100 mm: today's tolerance is about size ×
1e-7, so even 0.001 mm is "apart" and leaves a micrometer sliver face. At 1 m, 0.001 mm is near
the relative tolerance, and the kernel answers X (some faces merged, some not) or fails. The tilt
band fails outright at 1e-6 to 1e-5 rad at every size: that is finding 1. Directions and turns
change nothing, which is consistent with the determinism results.

### 3.5 After stage 2 (tolerant coincidence)

The box rows of the band now match the ideal in every one of their 372 cells. The table
below shows a sample:

```
flush y union        1 mm: MMMMMMDDDD   ideal MMMMMMDDDD
flush diagonal union 1 mm: MMMMMMMMDD   ideal MMMMMMMMDD   (each face δ/√2 off)
gap union         1000 mm: MMMMMMMMMDDDD  ideal MMMMMMMMMDDDD
tilt subtract     1000 mm: MMMMMMDDDD   ideal MMMMMMDDDD
```

Before stage 2, 169 cells were off the ideal and 22 failed. Now:

- **Boxes:** 0 cells off the ideal, 0 failed.
- **Coaxial rows (new):** cylinders one δ wider. They are right at 1 and 10 mm. At 100 mm and
  1 m, radii just beyond P fail (12 cells). There the ring between the radii is narrower than
  the chord error of a face's chart (circles sampled every 0.05 rad), so the piece's inside
  point is found off the piece. Stage 3 fixes that with exact classification.
- **Invariance:** 5 of 224 checks fail (unchanged; stage 3).

The diagonal ideal changed with stage 2. Coincidence is decided per face: two faces each δ/√2
apart are one, so the diagonal row merges while δ/√2 ≤ P.

### 3.4 Reproducing

- `luc test` in luce-cad runs the harness.
- `tests/robustness/*.lucb` documents each part.
- The cross-OS runs used `luce_work/tools/robust-remote.sh` with `ROBUST_RUN=1`.
- The literal table came from a script that lists every `\d+e-\d+` and `0.0…` literal outside
  comments, then from reading each one in context.

---

## 4. The tolerance model

### 4.1 Principles

1. **One setting, per project, chosen by the user.** This is the owner's decision. Like Alias's
   Construction Options, it lives with the units and is the contract the model is built to. Like
   Parasolid's precision, it is the same for every operation and never changes during a
   computation.
2. **Absolute, in millimeters.** luced-3d already keeps every length in millimeters; inches are
   display only (`luced-3d src/units.luc`). Kernel tolerances are therefore absolute millimeters,
   never fractions of a model's size.
3. **Everything else derives from it.** No literal in the kernel stands for a model distance or an
   angle. Every such decision calls the predicate layer (section 4.4).
4. **Local tolerances are local.** An entity that cannot meet the setting carries a measured
   tolerance of its own (Parasolid, Jackson '95). Only predicates about that entity use it. It
   never widens the model, the operation or its result.
5. **Tolerances never grow by habit.** A result entity's tolerance is measured from the geometry
   that was built, never "input × k". An operation that would leave an entity looser than the
   setting first tries to rebuild it tighter (refit, re-intersect) and makes it tolerant only when
   that fails.

### 4.2 The settings and their defaults

These are stored per project:

| setting | default | meaning | used by |
|---|---|---|---|
| **Positional** P | **0.01 mm** | Points within P are one point; a curve within P of a surface lies on it; gaps up to P close. The validity contract: every edge within P of its faces, every vertex within P of its edges. | All coincidence, on-surface and on-boundary decisions; vertex and edge identity; Stitch's default gap; the checker |
| **Fit** F | **0.001 mm** (P / 10) | How closely approximated geometry follows its exact definition: fillet sections, offsets, sweeps, traced intersections, projections. This is Alias's Curve Fit Distance. | Every fitter; trace refinement |
| **Tangent** T | **0.1°** | Two directions or normals within T are tangent. This is Alias's Continuity Angle. | Tangent chains (fillet, Fusion's Tangent chain), smooth-edge classification, G1 checks, tangent contacts |
| **Curvature** K | **0.5%** | Curvature deviation (R1 − R2)/(R1 + R2) within K is G2. This is Alias's Continuity Curvature. | G2 and G3 checks, curvature-continuous blends and fits |

The following are derived and are not settings:

| derived | value at defaults | rule |
|---|---|---|
| Resolution R | 1e-5 mm | P / 1000, which equals Parasolid's 1e-8 m. Solvers converge to R; constructed points (intersections, projections) are accurate to R; no decision is made closer than R to its threshold without the exact path. |
| Size box | 1 km cube (±500 m) | 1e8 × P, the same box-to-resolution ratio as Parasolid (1e11 × R). Operations refuse geometry outside it with an error naming the box. The box scales with P, so a coarser P allows larger parts, and a finer P suits smaller ones. |
| Angular resolution | 1e-11 rad | R / size box. Used only where no extent is known, for example to compare two unit vectors as raw data. Every topological "parallel" test has an extent and is posed positionally instead: "does face A stray more than P from surface B over its own extent". |
| Solver tolerances | R (positions), R / extent (parameters) | Newton and Gauss–Newton stop when the step is below R in model space. Parameter-space stops are R mapped through the local derivative, never a raw parameter epsilon. |
| Fit budget | F + R ≤ P / 5 | Fitted geometry plus its construction error stays within a fifth of P, so a result is never born at the decision threshold. The ×5 margin replaces today's chains of ×10, ×20 and ×100. |
| Display | chord default 0.05 mm, angle 15° (view settings) | Tessellation stays a document view setting, floored at P: a mesh never claims detail below P, and adjacent faces' meshes are welded at their shared edge's samples. The single-precision mesh is a display matter only. |
| Guards | none | Division and zero-length guards are not tolerances. They become exact checks (`== 0`, `is_finite`) or checks relative to the operands (`|x| ≤ 64 ε |scale|`) in a few named helpers, never a length literal. |

**Bounds.** P may be set from 1e-5 mm to 1 mm, F from P / 1000 to P, T from 0.001° to 5°, and K
from 0.01% to 10%.

**Presets**, as in Alias:

- "Fine (Parasolid-like)": P 0.0001 mm, F 0.00001 mm;
- "Default": 0.01 / 0.001 mm, 0.1°, 0.5%;
- "Class A": 0.001 / 0.0001 mm, 0.05°, 0.1%;
- "Rough import": 0.05 mm.

### 4.3 Per-entity tolerances

- **Where they come from.**
  - **Import:** each edge's distance to its faces' surfaces and each vertex's distance to its
    edges' ends are measured. Where the result exceeds P, the entity's precision becomes the
    measured value × 1.01; otherwise it stays 0 ("the setting").
  - **Stitch:** the measured gap per edge, as today.
  - **Intersection:** a traced section that cannot be fitted within F after the refit loop. It
    becomes tolerant with its measured distance, and the operation reports this on the node.
  - **Fillet and offset:** the same rule as intersection.
- **Invariant:** tol(vertex) ≥ tol(edge) at each of its edges' ends ≥ P.
- **Use:** a predicate on entities a and b uses max(P, tol(a), tol(b)). That is the "max", not the
  sum, so the rule is the same in every stage. `model_tolerance()`'s whole-model maximum goes away.
- **Tightening.** After an operation, a tolerant entity whose new neighbours allow it is
  re-measured, and its precision is lowered or cleared (Parasolid's tolerance optimisation, ACIS's
  `optimize_tvertex_tolerance`).

### 4.4 One predicate layer and one decision per fact

A new `src/predicates/` module is the only place in the kernel that compares a distance or an
angle with a tolerance. It covers:

- points: `same_point(a, b)`, `on_curve(p, c)`, `on_surface(p, s)`, `on_boundary(p, f)`;
- curves and faces: `curve_on_surface`, `coincident_surfaces` (plane/plane through spline/spline,
  one rule), `tangent_directions`, `parallel_over(extent, a, b)`;
- each predicate takes a `Context`: the project tolerances plus the entity tolerances involved,
  read as data. There are no globals; luce-cad has no module-level state today, and the design
  keeps it that way;
- each predicate returns a three-way answer (in / on / out, or less / equal / greater), with
  "on" meaning within tolerance. Its sign part is exact: orient2d and orient3d use Shewchuk-style
  adaptive arithmetic, generalising `polygons/predicates.lucb`, which already computes exact
  product errors with `fma`;
- ties at exact degeneracy, such as a ray through an edge or a point exactly on a plane, are
  broken by Simulation of Simplicity, so they have one answer however they are reached.

**The boolean's fact table.** `Work` gains a `Facts` store, filled by the intersect and imprint
stage and read by every later stage. It records:

- vertex × face: in, on or out, with the distance;
- edge × face: on, crossing at these parameters, or clear;
- face × face: coincident (same or opposite), tangent along these sections, crossing in these
  curves, or apart;
- vertex identities: an equivalence of input points, made once, with a canonical representative.

`clip`, `split`, `classify`, `merge` and `boolean_result` look facts up and never re-ask the
geometry. Classification of a piece then follows from facts wherever a piece touches the other
solid's faces (OCCT's and SMLib's local classification across a section edge). Only a piece with no
contact needs a point-in-solid test, and that test is exact parity with symbolic perturbation and a
single ray, not a three-ray vote.

**Certified intersection.** Before a surface pair is marched, it is subdivided on Bernstein bounds
until each cell either has no intersection (excluded) or holds one simple branch (a single seed).
Seeds come from the certified cells, ordered canonically. This removes "skip any seed within 0.8%
of the size" and makes a missed branch impossible.

### 4.4a As built (stage 2): tolerant coincidence

`modeling/coincide.lucb` is the fact pass. It runs once per boolean, after both solids are in
one shape and before any face pair is met.

- **Faces.** A face of the second solid whose surface is a first-solid surface of the same kind
  within P (axes, centers and radii within P; for planes, its whole boundary within P) is put
  on that surface. From then on the pair is coincident exactly, and every later step (meet,
  imprint, clip, split, classify, merge) sees one surface.
- **Edges and vertices.** Those left off where their faces' surfaces now meet become tolerant
  by P, or by how far they actually lie where that is further (a corner where two moved faces
  meet). A point is projected onto each of its faces' surfaces in turn until it settles.
- **Vertices of the second solid** within P of the first's become tolerant by P.

The later steps honour that slack wherever they ask whether two things are one:

- face pairs are met within it;
- cuts carry it, so a cut that came from a tolerant edge or vertex merges its vertex that far;
  cuts at one point are taken widest slack first;
- final edges are one when their middles lie within it;
- vertex cuts reach a tolerant vertex's zone;
- a lookup's loop boxes and the classification boxes are padded by it.

A vertex merges into the nearest vertex in reach, not the first one found. Two planes never
count as tangent along a line: lying together within P they were made one before, so a
shallow angle between them is a crossing.

Not done in stage 2: the working tolerances are still a millionth of the model's size. Making
them absolute also changed tessellation, because the mesher uses the model's tolerance for its
own decisions: a 10 m part meshed with degenerate triangles, and a 10 µm contract failed. So
the switch waits until meshing has its own display tolerance (stage 6). Decisions about
identity are absolute (P); the construction slop stays relative to size until then.

### 4.5 Changing the setting on an existing project

- **Re-evaluation.** The tolerances are part of every CAD node's cook key, as the units are part of
  the inspector's key today (`luced-3d src/inspector.luc:388`). Apply in Project Settings therefore
  re-cooks the whole CAD graph from its sources. Nothing is "converted": the model is rebuilt
  under the new contract, just as editing a feature's parameter rebuilds what follows.
- **Inputs are checked.** Imported bodies and Edit CAD bodies are re-run through the checker at
  the new P.
  - **Tightening P:** entities that no longer meet it become tolerant, with their measured value,
    and the import node shows a notice ("12 edges tolerant, worst 0.03 mm"). It is not an error,
    and nothing is moved.
  - **Loosening P:** features smaller than the new P can collapse, for example a 0.02 mm step
    under P = 0.05 mm. Every operation that meets such a feature reports it on its node ("a face
    narrower than the positional tolerance") rather than producing a sliver. The viewport keeps the
    last good result, as for any failing node.
- **Held bodies.** Edit CAD is becoming a held-state node: it keeps its bodies rather than
  replaying a recipe, so nothing re-cooks them. luce-cad gives it one call for a changed
  setting, `CadModeling.rechecked(body, tolerances)`. It returns the body under the new
  tolerances and a report:
  - tightening makes entities that no longer meet P tolerant, with their measured precision;
  - loosening reports edges shorter than P and faces narrower than P.

  The held state calls it on each held body when the project's tolerances change and shows the
  report on the node (landing with stage 5's checker).
- **Confirmation.** Before Apply, the dialog says that the model will be rebuilt. Afterwards, nodes
  whose topology signature changed are flagged, using the harness's signature. Undo restores the
  old setting and the old results.

### 4.6 Imported geometry with looser tolerances

- STEP declares an uncertainty (`UNCERTAINTY_MEASURE_WITH_UNIT`), but the real accuracy varies by
  entity. The import measures each edge and vertex as in 4.3. The declared uncertainty is shown,
  not used.
- The project P is never raised to fit an import. The STEP node's "STEP tolerance" field
  (`node_catalog.luc:332`) is removed (no backward compatibility).
- **The owner's decision: the import asks** (2026-10-13). When an import is looser than the
  project's P, the user chooses between two ways, and the choice is an option on the import node
  ("Tolerances: Adopt the scene's / Keep the file's"), so it replays with the project and is
  never asked again:
  - **Adopt the scene's tolerances:** the import heals and tightens to P. Edges are
    re-intersected and refitted where the surfaces allow; what still cannot meet P is reported
    on the node with its worst gap.
  - **Keep the file's tolerances:** the looser entities stay tolerant, each with its measured
    precision (Parasolid's tolerant modelling). The body is still under the project's
    tolerances, and only those entities are looser.
  - The first time a file turns out looser than P, the node shows a prompt with the two choices
    and the worst gap; the answer sets the option. Until it is answered, the node keeps the
    file's tolerances, so nothing is moved unasked. It follows luced-3d's existing pattern of a
    node notice with actions.
- Operations on a body with tolerant entities work normally. Where a new edge meets a tolerant
  one, the new vertex inherits the larger tolerance only locally.

### 4.7 Where the setting lives in luced-3d

- **Stored with the project next to the units.**
  - `Network` (`luced-3d src/network.luc:70`) gains `tolerances: CadTolerances` beside
    `units: str`, with positional, fit, tangent and curvature.
  - `project_graph.luc:87` writes them into `/scene` next to `"units"` and `:269` reads them back.
  - `network.luc:270` (copy) and `node_clipboard.luc:123` carry them like the units.
- **Shown in Project Settings, Units page** (`project_settings.luc:61`):
  - Positional and Fit, typed and shown in the document unit;
  - Tangent in degrees;
  - Curvature in percent;
  - a Preset menu;
  - the draft and Apply flow as for units.
- **Read once and passed down.** A cook reads `network.tolerances` once and passes the
  `CadTolerances` value, a plain Base struct of luce-cad's, down with the node's other inputs: to
  `Node.compute`, then to the tool's `Invocation` and to each Edit step (`EditKind.run`). Every
  luce-cad call that makes a body from nothing takes it as an argument: the `CadSolids`
  primitives, `CadModeling.extruded`/`extruded_sides`/`revolved`/`revolved_about` from a sketch,
  the surfacing builders (`square`, `skin`, `rail`, `profile`, `set_planar`, `extruded`,
  `revolved`, `blend`, `fillet`, `bridge`, `tube`, `fill`, `patch`) and the STEP import.
- **Bodies record their tolerances.** `BrepModel` carries the `CadTolerances` it was built under
  (`cad_tolerances()`), saved with it in `.geo` files, so a reloaded body is valid exactly as it
  was. An operation on bodies works under its inputs' tolerances (the first operand's), the way
  Parasolid's precision belongs to the session rather than to each call, so `combined`, `filleted`,
  `shelled` and the rest need no extra argument and cannot be handed the wrong one.
  - From stage 5, an operation given a second body built under another P treats it as an
    import: it measures it and makes entities tolerant where needed, so it never trusts a looser
    contract.
- **The cook key.** Every node's stamp includes the project's tolerances (`stamps.luc`), so
  changing them re-cooks everything and the result store never mixes results of two settings.
- **No global state anywhere:** no module variable, no session object. Two documents with
  different tolerances can cook concurrently in one process.

### 4.8 Keeping Fusion parity and the existing tests

- **Fusion-parity results stay.** They are verified to 1e-5 against Fusion: circles, volumes,
  sections. They are analytic constructions, which this design computes to R = 1e-5 mm. Where
  parity covers fitted geometry (G2 and G3 fillet sections, blend rows), the fitters keep their
  accuracy through F, which is 0.001 mm by default and is an operation parameter, so a parity test
  can pin a tighter F. No assertion is loosened.
- **Staging.** Each stage of section 6 lands only when every existing contract, regression and
  parity test is green on all three OSes. The harness ratchet must also not worsen, and should
  improve.
- **Scale tests.** Tests that build models at micrometer or kilometer scale (for example
  `exact_contract.lucb` at 1e-5 and 1e4) pass a matching P, inside the size-box rule. That is the
  only kind of test edit allowed.
- **The harness changes meaning.** Its scale checks become "same topology when every feature is
  far from P", and its band ideal (M up to P, D beyond, at every size) becomes a hard assertion
  once stage 3 lands.

---

## 5. Determinism contract

**The contract:**

1. **Identical inputs give identical outputs.** The same input bits, the same tolerances and the
   same luce-cad version give a bit-identical model on every run, any thread count and every
   supported OS (macOS arm64, Linux x86-64, Windows x64).
2. **Order does not change the result.** Permuting faces, edges, operands or sketch entities
   (where the operation is symmetric) gives the same topology and geometry within R. Results are
   emitted in a canonical order, and each merged entity has a canonical representative.
3. **Rigid motion does not change the result.** Moving or turning the inputs gives the same
   topology, and geometry within R of the moved result, except where a decision lies within R of
   its threshold. The checker can report such near-threshold decisions.
4. **Units.** mm and inch display give bit-identical results, because the kernel always works in
   mm.

**How it is enforced:**

- **The harness.** It runs fingerprints in a row and concurrently. The cross-OS gate compares the
  printed fingerprints of the three runs: `robust-remote.sh` with `ROBUST_RUN=1`, diffed
  automatically in the gate script. Invariance and band checks ratchet.
- **Code rules,** checked in review and by a grep gate in `luc test` (a contract that scans
  `src/`):
  - no `.hash()`-keyed table may be iterated into output;
  - parallel work writes only into indexed slots and reduces serially in index order;
  - every sort comparator is a total order with an index tie-break;
  - no clock, random or address feeds geometry;
  - no tolerance literal appears outside `predicates/` and the named guard helpers.
- **Canonical representatives** (stage 4): a merged vertex is the input point with the smallest
  local tolerance, ties broken lexicographically by coordinates. A shared edge keeps the
  exact-carrier one (analytic before fitted), ties by a canonical key. A traced curve starts at its
  lexicographically smallest point and runs in a canonical direction.
- **Toolchain:** in-house transcendental functions in luce-std and `-ffp-contract=off` are
  preconditions, to be stated in luce-base's docs so they cannot regress silently. FMA is used
  only where its exactness is intended.

---

## 6. Staged plan

| stage | what | size | outcome |
|---|---|---|---|
| 0 ✓ | Harness and fingerprint (60a9332) | done | Baseline; cross-OS determinism measured |
| 1 ✓ | **Tolerance type and plumbing** (luce-cad 2256fc7, luced-3d 0b6745b). `CadTolerances` (P, F, T, K, derived R, size box) in luce-cad; every body-making call takes it; `BrepModel` stores and saves it; the size box is enforced. luced-3d: Network field, save and load, Project Settings rows, a cook-key part, pass-down. The working tolerances stay as they are: switching them to absolute values alone, without the predicate layer, was tried and made the tilt band worse (shallow plane crossings at 1e-7 to 1e-5 rad fail in more places), so the switch moved to stage 2. | L: 3–4 days | The setting reaches every body; results unchanged (fingerprints identical) |
| 2 ✓ | **Tolerant coincidence: one decision per fact at P** (4.4a). Faces of the second solid on a first-solid surface within P are put on it, and their edges and vertices become tolerant. Every later step honours that slack. A vertex merges into the nearest in reach. Planes are never tangent. The coaxial band rows were added to the harness. | done | Box band 0 of 372 off (was 169) and 0 failed (was 22); all tests green |
| 3 | **Classification and degeneracy.** Classification by local facts plus exact single-ray parity with Simulation of Simplicity; certified subdivision seeding for the march; fix the box/sphere-on-face case and the face-order tessellation case. | L: about 2k lines, 1 week | Invariance failures 0; band assertion becomes hard |
| 4 | **Canonical order and representatives** (vertex merge, final edges, march start, Stitch representatives, output ordering). | M: about 800 lines, 3–4 days | Operand and face permutation give identical topology; contract item 2 asserted |
| 5 | **Per-entity tolerances done right and the checker.** Remove `model_tolerance()`; keep B's tolerances; measured precision on import, stitch, intersection and fillet; tolerance tightening; a `check_body` (structure, geometry on topology, orientation, loops, sampled self-intersection) usable as a node and run on every result in tests. | L: about 2k lines, 1 week | Imports keep P; no model-wide widening |
| 6 | **The rest of the kernel by directory, and absolute working tolerances.** Meshing gets its own display tolerance, then `tolerance_for(size)` and the `size × 1e-7` work tolerances become the project's resolution. Fillet and blend, shell, surfacing, sketch, layout and meshing literals moved onto predicates and derived tolerances. Guards become helpers. Angular tests become the T or positional forms. About 800 literals, mechanical, one directory per commit, gated. | L: 1.5–2 weeks | No unprincipled literal left; the grep gate turns on |
| 7 | **Heavy sweep tool** (`dev/robustness`): randomised rotations, scales and near-degenerate placements in the thousands, run on demand, not in `luc test`. Per the owner's priority this is after all requests and is not fuzzing. | S: 2 days | Continued coverage |

Stages 1 and 2 hold most of the value. Each stage is pushed in green, gated pieces.

---

## 7. Owner decisions (2026-10-13)

The owner agreed with the whole design and approved stages 1–7, in order.

1. **Fit tolerance:** its own setting, default 0.001 mm.
2. **Changing the setting:** Apply rebuilds the CAD graph and flags what changed; undo is the
   safety net (4.5).
3. **The size box:** geometry outside it is refused, with a message to use a coarser P or move
   the part.
4. **Imports looser than P:** the import asks whether to adopt the scene's tolerances (heal and
   tighten to P) or keep the file's (tolerant entities). The answer is an option on the import
   node, so it replays without asking again; the prompt sets it the first time (4.6).
