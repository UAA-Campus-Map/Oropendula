# `walkable.mjs` — indoor walkability flood mask

## What it does

Computes **which cells of a floor are indoors** *before* the navGraph is built.
The navGraph raster treats every free cell inside the wall bbox as walkable, so
courtyards, breezeways, gaps and the ground around a building become part of the
indoor graph. `floodIndoor()` floods the outside from a seed in the ring around
the walls, seals the openings, and returns a mask that
`buildGraphState(..., { mask })` applies to the clearance raster.

Export: `floodIndoor({ wallsLL, extraWallsLL, doorways, seals, sealPoints,
outdoorSeeds, outdoorSeed, indoorSeed, cell, padM })` →

```
{
  mask: { x0, y0, cell, W, H, data },   // 1 = indoor/walkable, 0 = outdoor/wall
  blocked,                              // walls + seals (flood obstacles)
  wall,                                 // real walls only (preview)
  outdoor,                              // outdoor flood (preview/debug)
  outdoorDist,                          // BFS distance from the outdoor seed
  parent,                               // BFS parent cell (leak path)
  leakPath,                             // seed -> indoor label, when leak
  leakEntry,                            // first wall-bbox cell on the leak path
  bbox,                                 // expanded grid bbox (metres)
  wallBBox,                             // this floor's wall extents (metres)
  toLL,                                 // mask metre frame -> lon/lat
  outdoorSeeds, outdoorSeed, indoorSeed, // the points actually used
  leak, indoorCells, outdoorCells, ms,
}
```

or `{ error }` when the plan cannot be flooded.

## Algorithm

1. **Frame.** Use the same metre frame as `navgraph.mjs`: origin is the minimum
   wall lon/lat and the scale is `metresPerDegree(origin.lat)`. The mask
   therefore samples 1:1 into the navGraph raster. The grid covers the wall
   bbox plus **2.0 m** (a 0.5 m seed ring + 1.5 m outer margin), so the outdoor
   flood is a contiguous band that can travel around exit seals.
2. **Rasterise.** Paint this floor's walls, any **same-level neighbour walls**
   whose bbox overlaps this flood bbox, and manual seals. Interior doorways are
   sealed with a 0.5 m band around the barrier *and* a disc around the doorway
   centre (`width/2 + 0.3 m`), because the DXF leaf/barrier is often drawn beside
   the real opening. Exterior doors and skybridges are sealed with a 1.2 m disc
   (`sealPoints`). The 1.5 m outer margin leaves free outside cells beyond those
   discs.
3. **Outdoor seeds.** Without overrides, pick one seed per connected component
   of the outer margin (then the 0.5 m ring, then any free outside cell). This
   handles buildings whose outside band is split by a neighbour wall or exit
   seal. `outdoorSeeds` adds manual seeds (repeat `Seed outdoors`); `outdoorSeed`
   is the legacy single override. Every free outside cell (not just the ring) is
   marked outdoor regardless of reachability.
4. **Outdoor flood.** BFS over free cells from the outdoor seed.
5. **Indoor validation.** The auto indoor seed is a **room label** chosen by the
   import tool: a room-number label 0.6–6 m from the walls (farthest in that
   band), else the closest label. `indoorSeed` overrides it. `leak` is true when
   the outdoor flood reached that seed. If no seed can be found at all, the
   result is `{ error }`.
6. **Mask.** A cell inside the wall bbox that the outdoor flood did not reach
   is indoor (`data = 1`), except real walls. Artificial seal cells are indoor:
   they are the doorway threshold, and the navGraph wall raster still blocks the
   actual wall. Doorway thresholds are also kept walkable (0.4 m disc around
   each doorway centre). The outside band and outdoor cells are 0.

The mask is a **hard gate**: `buildGraphState` blocks every free cell whose
mask sample is 0 (and every cell outside the mask bbox). Walls still block
independently.

## Import-tool workflow

The **Walkable (flood)** panel in `import.html`:

- **Preview walkable** — flood the selected floor and draw the mask on the map
  (green = indoor; outdoor uses a blue-to-red distance gradient from the seed
  to the dead ends). The note shows the cell counts, timing and `LEAK`. When the
  flood reaches the indoor label, the tool draws the BFS leak path in blue and
  marks where it first enters the wall bbox.
- **Auto-run** — the preview runs when step 5 opens and re-runs after any
  connection edit (exits/skybridges change the seals).
- **Seed indoors** / **Seed outdoors** — tap the map to override the automatic
  seeds. `Seed outdoors` can be used repeatedly: add one seed for each
  disconnected outdoor area (SU is connected to other buildings on two sides).
  A manual indoor seed is also the leak check: if the flood reaches it, the
  panel reports a leak.
- **Seal gap** — tap two points across an opening that leaks. Seals are stored
  on the floor as `floodSeals` and persist in `buildings-config.json`.
- **Clear seeds/seals** — remove the overrides on this floor.

The auto seeds/seals are: a room-number label in the 0.6–6 m wall band (indoor;
farthest in the band, else closest), the outer margin / 0.5 m ring (outdoor),
DOOR-layer doorways, `floor.exits` (manual **and** the cached algorithm exits),
and manual `floor.skybridges`. Same-level buildings whose wall bbox overlaps the
2.0 m flood grid contribute their walls, so a shared wall is solid.

**Bundle generation refuses on a leak.** `generateBuildingBundle()` floods every
floor and throws before writing a bundle if `error` or `leak` is set. Fix it with
a seal or a seed and regenerate. The mask is baked into the `.nvg.gz`; the
bundle `sourceHash` stays the wall/door hash, so changing only seeds/seals does
not invalidate an already-deployed bundle — regenerate it explicitly (the
import tool does this when you download the bundle or the changed-buildings
ZIP).

## Invariants and gotchas

- **Flood before graph.** Never run the mask after the navGraph raster is built;
  it is an input to `buildGraphState`, not a post-filter.
- **Same frame.** `floodIndoor` must use the floor's wall lines as `wallsLL`; if
  the origin differs, mask sampling will be offset. `navGraphInputs()` supplies
  the lines; `buildGraphState` re-derives the same origin.
- **Ring is always outdoor.** Do not "fix" a leak by marking the ring indoor; it
  is the outside by definition.
- **Doorway seals are deliberately wide.** They are for the flood only. The
  navGraph keeps the doorway threshold walkable via the 0.4 m keep disc.
- **Leak means the room label was reached.** A leak confined to a corridor (not
  the chosen label) does not set `leak`, but the flooded corridor is grey in the
  preview and is removed from the graph. Seal it or move the indoor seed.
- **The app's fallback build has no mask.** If a bundle is missing or stale,
  `app.html` rebuilds without the flood (the fallback is a safety net). Generate
  and deploy the bundle to get the mask. The CLI `gen-navgraphs.mjs` does not
  apply the mask either; use the import tool for production bundles.
- **Cost.** ~0.1 s per floor at 0.1 m cells on the real plans; the import tool
  caches floor JSONs, so neighbour lookups are cheap.

## Tests

`node smoke.mjs walkable` builds synthetic plans: a C-shaped building with a
courtyard open to the south (outdoor inside the bbox), a room with an open door
(leak), the same room with a doorway seal (no leak), a manual seal, and the
navGraph mask gate.

See also [`navgraph.md`](navgraph.md), [`doorways.md`](doorways.md),
[`campus.md`](campus.md) and [`apps.md`](apps.md).
