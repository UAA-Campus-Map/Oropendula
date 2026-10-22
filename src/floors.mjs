/**
 * Floor-id semantics shared by the app and the import tool.
 *
 * Floor ids are strings ("01", "B1", "R", ...). Basement levels must sort below
 * the ground floor and must not be numbered as upper floors:
 *
 *   B2 < B1 < 01 < 02 < 03 ...
 *
 * `floorLevelOf()` is the absolute level used for 3D stacking and route colour:
 * ground level 1 is 1, B1 is 0, B2 is -1.
 */

/** Numeric rank for sorting: basements negative, numeric floors positive. */
export function floorRank(id) {
  const s = String(id ?? '').trim().toUpperCase();
  const basement = /^B(\d+)$/.exec(s);
  if (basement) return -Number(basement[1]);
  const n = parseInt(s, 10);
  return Number.isFinite(n) ? n : 0;
}

/** True for B1/B2/... floor ids. */
export function isBasement(id) {
  return /^B\d+$/i.test(String(id ?? '').trim());
}

/** Human label: "Basement 1" for B1, "Floor 2" for 02, otherwise the id. */
export function floorLabelOf(id) {
  const s = String(id ?? '').trim();
  const basement = /^B(\d+)$/i.exec(s);
  if (basement) return `Basement ${Number(basement[1])}`;
  if (/^\d+$/.test(s)) return `Floor ${Number(s)}`;
  return s || 'Floor';
}

/**
 * Absolute level for 3D stacking / colours. Ground "01" is 1, B1 is 0,
 * B2 is -1; non-numeric ids fall back to ground level.
 */
export function floorLevelOf(id) {
  const s = String(id ?? '').trim().toUpperCase();
  const basement = /^B(\d+)$/.exec(s);
  if (basement) return 1 - Number(basement[1]);
  const n = parseInt(s, 10);
  return Number.isFinite(n) ? n : 1;
}
