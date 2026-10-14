# `align.mjs` — rigid floor alignment

## What it does

Aligns one floor onto a reference floor (usually the building's ground floor) by
matching their exterior perimeters. Pure geometry, unit-agnostic — points are
`[x, y]` pairs in any common frame. Exports:

- `convexHull(points)` (monotone chain), `meanPoint`, `ringSamples`,
  `hullSamples`.
- `bestFitRigid(current, reference, seed)` → a rigid transform.

## Why it's built this way

**Rigid, not scaled.** Both floors are drawn in inches, so the transform is
rotation + translation only (scale fixed at 1). Allowing scale would let a
mis-aligned floor "shrink to fit" and hide the error.

**Seed, then refine.** A rigid fit from scratch can fall into a local minimum.
So `bestFitRigid` first finds a good seed, then refines:

1. **Seed.** If both floors have two elevator anchors, use them — the shaft is
   vertically true, so it is the most trustworthy reference. Otherwise do a
   coarse rotation scan.
2. **Refine.** A few trimmed nearest-neighbour / Procrustes passes, rejecting the
   worst outliers each round.

**Hulls, not raw walls.** Matching the **convex hull** of each floor's wall
endpoints, sampled into points, ignores interior partition differences and keys
on the building outline.

## Implementation notes

- Used by `import.html` when stacking plans; the runtime router never calls it.
- Sampled points and hull are dependent on the input ordering only through the
  sort inside `convexHull`, so results are deterministic.
- The transform is returned in the caller's frame; `import.html` turns it into
  the floor `offset.rot/x/y` consumed by `geo.mjs`.

## See also

- [`geo.mjs`](geo.mjs) — the placement the alignment writes.
- [`apps.md`](apps.md) — `import.html`.
