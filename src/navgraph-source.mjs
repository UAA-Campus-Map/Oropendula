/**
 * Shared extraction of the inputs to a floor navGraph.
 *
 * The app, the import-tool bundle generator, and the test harness must all
 * derive the same wall/door geometry from a floor JSON. Keeping this in one
 * place prevents a generated bundle from silently describing a different graph
 * than the fallback on-device build.
 */
import { shapeSegments, isWallLayer, isDoorLayer } from './campus.mjs';

/**
 * @param {object} buildingConfig - buildings-config.json building entry
 * @param {object} floorConfig - that building's floor entry
 * @param {object} data - parsed floor JSON
 * @param {(x:number,y:number)=>number[]} toLL - DXF -> lon/lat for this floor
 * @returns {{lines: number[][][], doors: number[][]}}
 */
export function navGraphInputs(buildingConfig, floorConfig, data, toLL) {
  const shapes = data?.shapes ?? [];
  const walls = shapes.filter((s) => isWallLayer(s.layer));
  const lines = walls.flatMap((wall) =>
    shapeSegments(wall).map(([a, b]) => [toLL(a[0], a[1]), toLL(b[0], b[1])]));

  const doors = shapes
    .filter((s) => isDoorLayer(s.layer))
    .map((s) => {
      const leaf = (s.items ?? []).find((it) => it.op === 'line' && it.points?.length >= 2);
      if (leaf) {
        const a = leaf.points[0];
        const b = leaf.points[leaf.points.length - 1];
        return toLL((a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
      }
      if (s.bbox) return toLL((s.bbox[0] + s.bbox[2]) / 2, (s.bbox[1] + s.bbox[3]) / 2);
      return null;
    })
    .filter(Boolean);

  return { lines, doors };
}
