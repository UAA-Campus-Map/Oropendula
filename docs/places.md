# `places.mjs` — place nicknames, favorites and student votes

## What it does

Lets the search box find a place by the names students actually use
("Subway", "Robotics Club", "AI club") as well as by `<building code> <room>`.
Three parts:

| Part | File | Who writes it |
|------|------|---------------|
| Default nicknames | `src/places.json` | admins, through `src/places.html` |
| Favorites | the phone's `localStorage` (`places-favorites`) | each student, from the route card |
| Votes | an external form (`PLACE_VOTE_FORM_URL` in `app.html`) | students, signed in with their UAA account |

Students vote for a nickname through a form the team runs outside this app
(students sign in with their university email there, which proves they are
UAA people). Admins read the responses and add the names they accept in
`places.html`, then download and upload `places.json`. Nothing reaches
everyone's search without that review, and the site has **no backend and no
`/api/` endpoint**.

`places.mjs` exports (no DOM, shared by `app.html`, `places.html` and
`smoke.mjs`):

- `splitRoomQuery(buildings, q)`, `findRoom(building, roomText)`,
  `doorwayFor(floor, labelLL)`, `findBuilding`, `normRoom` — the room search
  that used to live inline in `app.html`; `findRoom` returns the label `text` as
  drawn. `doorwayFor` prefers the floor's physical `doorways` (wall-aligned
  barriers) and falls back to DOOR-leaf midpoints when only those exist.
- `nameTokens`, `normName` — name normalisation (case, accents, punctuation and
  `and/the/of/a/an` ignored: "AI & Robotics Lab" = "ai robotics lab").
- `placeTarget(place)` / `parseTarget(target)` — the canonical location key,
  `"SU Subway"` or `"ll:-149.823123,61.190000@01"`. The vote form receives it
  as `{place}`.
- `buildPlaceIndex(places, favorites)`, `searchPlaces(index, q, limit)` — ranked
  matching (below).
- `resolvePlace(buildings, place)` → `{ll, labelLL, floorId, text?, building}` or
  null when the building is not registered or the room label is not drawn.
- `validatePlaces(places)` → a list of problems (the admin page and the smoke
  gate share it).

## The data

```json
{ "id": "su-subway", "names": ["Subway"], "category": "food",
  "building": "SU", "room": "Subway" }
{ "id": "statue", "names": ["Seawolf Statue"], "category": "landmark",
  "ll": [-149.82, 61.19], "floor": "01" }
```

- `building` is a search code: the floor JSON's `ID` (SU, ESH) or the
  registration id (AS119). `room` is a room label **exactly as the extractor
  drew it** (`text_lines`), so a place resolves through the same `findRoom` as
  "SU 101" and routes to the room's doorway.
- `ll` + `floor` is for places that are not a labelled room.
- Favorites have the same location fields with a single `name`.
- A place in a building that is not registered yet (EIB is AS162) is valid
  but does not resolve; search says it is "not on the map yet", and the admin
  page and smoke flag it.

## Why it's built this way

**JSON for the default list, no database.** The app is a static site with no
build step and no backend, and its other data (`buildings-config.json`) is
already "edit in an admin page, download, `scp`". `places.json` follows that:
gzipped by nginx, reviewable in a git diff, fixable by hand.

**Votes in an external form, not a vote server.** An earlier draft counted
favorites in a Node + SQLite service behind nginx `/api/`. It was removed so
the server exposes nothing but static files: no write endpoint to abuse, no
process to patch. The form also does what the anonymous device ids could not:
it checks that the voter is a UAA student, through their university sign-in.

**Admin approval.** Automatic promotion would put any prank or offensive name
in front of every student. The admin reads the votes and decides.

## Search order (`app.html` `runSearch`)

1. A suggestion picked from the dropdown (tap, or arrow keys + Enter).
2. A place name typed **in full** (score 0) — "Subway", "ai club".
3. `<building code> <room>`, unchanged — "SU 101", "BMH114A".
4. The best partial name match — "robotics", "sandw".
5. Otherwise the old error messages.

Room numbers come before partial names so "ESH 114" never turns into a
nickname that happens to contain "esh".

Ranking inside `searchPlaces`: 0 same name, 1 name starts with the query,
2 every query word starts a word of the name ("club ai" → "AI Club"), 3 the
query appears inside the name. Favorites win ties, then shorter names; one
result per (place, name).

## Favorites (`app.html`)

- The route card shows **☆ Save** for search destinations only (not the
  calibration walk or the dev tap-to-route); `searchDest` tracks that.
- Save asks "What do you call it?", pre-filled with the searched name, and
  stores `{name, building, room}` or `{name, ll, floor}`. One favorite per place
  per phone; tapping **★ Saved as …** removes it.
- Nothing is sent when saving or removing a favorite.
- When `PLACE_VOTE_FORM_URL` is set, the row also shows **Vote to add**, which
  opens the form in a new tab. `{name}` (the saved name, else the searched
  one) and `{place}` (the `placeTarget` key) in the URL are replaced,
  URL-encoded, so a Google Forms pre-filled link fills both answers:
  `https://docs.google.com/forms/d/e/<id>/viewform?usp=pp_url&entry.111={name}&entry.222={place}`.
  Empty hides the link.
- An empty search box lists the favorites in the dropdown.

## Admin page exposure

`places.html` is served as a normal static page, **without basic auth**. That is
deliberate: it has no write endpoint and cannot change the server. It only reads
the public `places.json`, `buildings-config.json` and floor JSONs; all edits stay
in the browser (and that admin's `localStorage`) until someone downloads
`places.json` and uploads it out of band. Restricting the page is a deployment
choice, not a requirement of the app, and the current deployment does not gate
it.

## Implementation notes

- The places index is built after the campus router (`style.load`), from
  `placesPromise` started right after the config fetch. A missing or broken
  `places.json` yields an empty list; it must never block boot (unlike
  `buildings-config.json`).
- `places`, `favorites`, `placeIndex` and `searchDest` are declared next to
  the search elements, before the boot `await`, because the search box is live
  while the floors load.
- Suggestion buttons `preventDefault` on `pointerdown` so the input keeps
  focus and its `blur` does not hide the list before the click lands.
- `#topbar:focus-within` raises the top bar above the route card so the
  dropdown overlaps it.
- `node smoke.mjs places` (fast, no navGraph build): matching and ranking,
  targets, validation, room lookup on a synthetic building, and every shipped
  `places.json` entry resolving on the registered floors.
- Publishing needs `places.json` plus, on the first deploy, `places.html` and
  `places.mjs` (and `app.html` when its search changes). Upload
  `places.json`, never edit it on the server.

## See also

- [apps.md](apps.md) — `app.html` and `places.html`.
- [tooling.md](tooling.md) — `smoke.mjs` categories.
