/**
 * Runtime helpers for cached navGraph bundles.
 *
 * The app still needs the floor JSON for map layers, portals, and room labels.
 * This module turns a decoded bundle floor into the same graph object that
 * `buildGraphState()` + `graphFromState()` would produce, but defers the
 * passage/heading-field build until the locator actually needs it.
 */
import { graphFromState } from './navgraph.mjs';
import { expandAdjacency, floorGraphState } from './navgraph-codec.mjs';
import { buildPassages, makeHeadingField } from './headingfield.mjs';

/**
 * Build a queryable graph from one decoded bundle floor.
 *
 * @param {object} floor - result of `decodeBundle().floors[i]`
 * @param {{lines: number[][][], doorPoints: number[][]}} inputs - the same
 *        wall/door geometry `buildGraphState()` used
 * @returns {object} graphFromState result, plus `ensureHeadingField()`
 */
export function makeBundleGraph(floor, { lines, doorPoints }) {
  const adj = expandAdjacency(floor);
  const graph = graphFromState(floorGraphState(floor, adj));

  const { origin, scale, nodes } = floor;
  const toM = ([lon, lat]) => [
    (lon - origin.lon) * scale.lon,
    (lat - origin.lat) * scale.lat,
  ];
  const doorsM = doorPoints.map(toM);
  const segments = lines.map(([a, b]) => [toM(a), toM(b)]);

  // The app's locator calls this when it first needs compass directions. It is
  // intentionally not invoked at boot: passages are ~5M segments across the
  // campus and only the active floor level needs them. Capture only the arrays
  // needed here so the decoded bundle (and its compact adjacency) can be GC'd.
  graph.ensureHeadingField = () => {
    if (!graph.headingField) {
      graph.headingField = makeHeadingField(
        buildPassages(nodes, adj, doorsM, segments),
        { toM });
    }
    return graph.headingField;
  };

  return graph;
}
