# Oropendula

Static, dependency-free indoor/outdoor campus navigation for UAA (Anchorage).
No build step, no npm packages: each page is one HTML file importing the ES
modules in this directory.

- **Production entry point:** `app.html` → https://uaa-campus-map.net
- **Demo / tools:** `index.html` (original demo), `import.html` (building
  authoring + deploy), `places.html` (place nicknames), `locate-test.html`
  (positioning testbed), `import-progress.html`.
- **Data in this directory:** `buildings-config.json` (registrations, portal
  cache, `graphUrl`/hashes), per-floor `*.json`, per-building `*.nvg.gz` navGraph
  bundles, `places.json`, `osm-paths.json`, `style.json`.
- **Tests:** `node smoke.mjs` (see [`../docs/tooling.md`](../docs/tooling.md));
  the gate must be green before deploying changed files.

Read [`agents.md`](agents.md) first — it is the single narrative of hard-won
context — then [`../docs/README.md`](../docs/README.md) for one page per module
(API, invariants, gotchas). If a doc and the code disagree, the code wins; fix
the doc.
