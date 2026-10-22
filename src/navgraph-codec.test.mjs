/**
 * Codec round-trip tests.
 *
 * Run: node --test navgraph-codec.test.mjs
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildGraphState, graphFromState } from './navgraph.mjs';
import {
  encodeFloorBlock, encodeBundle, encodeBundleFromBlocks,
  decodeBundle, decodeFloor, expandAdjacency, floorGraphState,
} from './navgraph-codec.mjs';
import { gzipBytes, gunzipBytes } from './navgraph-codec-io.mjs';
import { navGraphInputs } from './navgraph-source.mjs';
import { makeBundleGraph } from './navgraph-bundle-runtime.mjs';
import { makeToLL } from './geo.mjs';

const CONFIG = JSON.parse(readFileSync(new URL('./buildings-config.json', import.meta.url), 'utf8'));

function syntheticState() {
  const nodes = [[0, 0], [1, 0], [0.5, 1]];
  const adj = [
    [[1, 1]],
    [[0, 1], [2, Math.sqrt(1.25)]],
    [[1, Math.sqrt(1.25)]],
  ];
  const W = 4, H = 3;
  return {
    origin: { lon: -149.8, lat: 61.19 },
    scale: { lon: 111412.84, lat: 111132.92 },
    OX: -1, OY: -1, W, H,
    nodes,
    adj,
    blocked: Uint8Array.from([0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0]),
    stats: { nodes: 3, edges: 2, cells: 12 },
    passages: null,
  };
}

function realFloor(buildingId, floorId, opts = { passages: false }) {
  const building = CONFIG.find((b) => b.id === buildingId);
  assert.ok(building, `missing building ${buildingId}`);
  const floor = building.floors.find((f) => f.floorId === floorId);
  assert.ok(floor, `missing floor ${buildingId} ${floorId}`);
  const data = JSON.parse(readFileSync(new URL('.' + floor.url, import.meta.url), 'utf8'));
  const toLL = makeToLL(building, floor);
  const { lines, doors } = navGraphInputs(building, floor, data, toLL);
  return { building, floor, inputs: { lines, doors }, state: buildGraphState(lines, doors, opts) };
}

function compareState(source, decoded) {
  assert.equal(decoded.nodes.length, source.nodes.length);
  for (let i = 0; i < source.nodes.length; i++) {
    assert.deepEqual(decoded.nodes[i], source.nodes[i], `node ${i}`);
  }
  assert.deepEqual(decoded.blocked, source.blocked, 'blocked raster');

  const adj = expandAdjacency(decoded);
  assert.equal(adj.length, source.adj.length);
  for (let i = 0; i < source.adj.length; i++) {
    assert.equal(adj[i].length, source.adj[i].length, `degree ${i}`);
    for (let k = 0; k < source.adj[i].length; k++) {
      const a = source.adj[i][k], b = adj[i][k];
      assert.equal(b[0], a[0], `neighbour ${i}/${k}`);
      assert.equal(b[1], a[1], `weight ${i}/${k}`);
    }
  }
  return adj;
}

function llOf(state, node) {
  return [
    state.origin.lon + node[0] / state.scale.lon,
    state.origin.lat + node[1] / state.scale.lat,
  ];
}

function compareRoutes(source, decoded, points) {
  const a = graphFromState({ ...source, passages: null });
  const b = graphFromState({ ...decoded, passages: null });
  let checked = 0;
  for (const [start, end] of points) {
    const pa = a.findPath(start, end);
    const pb = b.findPath(start, end);
    assert.deepEqual(pb, pa, `route ${start} -> ${end}`);
    if (pa) checked++;
  }
  assert.ok(checked > 0, 'expected at least one reachable route');
}

test('synthetic floor block round-trip', () => {
  const state = syntheticState();
  const { block, meta } = encodeFloorBlock(state);
  const decoded = decodeFloor({ ...meta, floorId: '01' }, block);
  compareState(state, decoded);
});

test('buildGraphState passages option leaves graph data unchanged', () => {
  const withPassages = realFloor('AS102', '01', { passages: true }).state;
  const withoutPassages = realFloor('AS102', '01', { passages: false }).state;
  assert.ok(withPassages.passages?.length > 0);
  assert.equal(withoutPassages.passages, null);
  assert.deepEqual(withoutPassages.nodes, withPassages.nodes);
  assert.deepEqual(withoutPassages.blocked, withPassages.blocked);
  assert.deepEqual(withoutPassages.adj, withPassages.adj);
});

test('bundle runtime defers passages and matches source graph', () => {
  const cached = realFloor('AS102', '01', { passages: false });
  const source = realFloor('AS102', '01', { passages: true });
  const bytes = encodeBundle({ buildingId: 'AS102', floors: [{ floorId: '01', state: cached.state }] });
  const decoded = decodeBundle(bytes).floors[0];
  const graph = makeBundleGraph(decoded, {
    lines: cached.inputs.lines,
    doorPoints: cached.inputs.doors,
  });

  assert.equal(graph.headingField, null, 'heading field must be lazy');
  const sourceGraph = graphFromState(source.state);
  const n = cached.state.nodes.length;
  const start = llOf(cached.state, cached.state.nodes[0]);
  const end = llOf(cached.state, cached.state.nodes[n - 1]);
  assert.deepEqual(graph.findPath(start, end), sourceGraph.findPath(start, end));

  graph.ensureHeadingField();
  assert.ok(graph.headingField);
  assert.deepEqual(graph.headingField.query(start), sourceGraph.headingField.query(start));
});

test('multi-floor bundle round-trip and gzip', async () => {
  const as102 = realFloor('AS102', '01');
  const as154 = realFloor('AS154', '01');
  const bytes = encodeBundle({
    buildingId: 'TEST',
    floors: [
      { floorId: '01', state: as102.state },
      { floorId: '02', state: as154.state },
    ],
  });
  const decoded = decodeBundle(bytes);
  assert.equal(decoded.buildingId, 'TEST');
  assert.deepEqual(decoded.floors.map((f) => f.floorId), ['01', '02']);
  compareState(as102.state, decoded.floors[0]);
  compareState(as154.state, decoded.floors[1]);

  const gz = await gzipBytes(bytes);
  const raw = await gunzipBytes(gz);
  assert.deepEqual(Buffer.from(raw), Buffer.from(bytes));
  assert.deepEqual(decodeBundle(raw).floors.map((f) => f.floorId), ['01', '02']);
});

test('real floor AS127-01 round-trip and route parity', () => {
  const { state } = realFloor('AS127', '01');
  const bytes = encodeBundle({ buildingId: 'AS127', floors: [{ floorId: '01', state }] });
  const decoded = decodeBundle(bytes).floors[0];
  const adj = compareState(state, decoded);

  const routeState = { ...floorGraphState(decoded, adj) };
  const n = state.nodes.length;
  const pairs = [
    [llOf(state, state.nodes[0]), llOf(state, state.nodes[n - 1])],
    [llOf(state, state.nodes[1]), llOf(state, state.nodes[Math.floor(n / 2)])],
    [llOf(state, state.nodes[Math.floor(n / 3)]), llOf(state, state.nodes[Math.floor(2 * n / 3)])],
  ];
  compareRoutes(state, routeState, pairs);
});

test('rejects bad magic and truncated blocks', () => {
  const state = syntheticState();
  const bytes = encodeBundle({ buildingId: 'TEST', floors: [{ floorId: '01', state }] });
  const bad = bytes.slice();
  bad[0] = 0;
  assert.throws(() => decodeBundle(bad), /magic/);
  const { block, meta } = encodeFloorBlock(state);
  assert.throws(() => decodeFloor({ ...meta, floorId: '01' }, block.subarray(0, block.length - 8)), /truncated/);
});

test('rejects corrupt adjacency and blocked runs', () => {
  const state = syntheticState();
  const { block, meta } = encodeFloorBlock(state);
  const bad = block.slice();
  // First neighbour lives at nodes*16 + (nodes+1)*4.
  const neighbourOffset = meta.nodes * 16 + (meta.nodes + 1) * 4;
  new Uint32Array(bad.buffer, neighbourOffset, meta.edges)[0] = 0; // self-edge
  assert.throws(() => decodeFloor({ ...meta, floorId: '01' }, bad), /neighbour/);

  const badRuns = block.slice();
  const lengthsOffset = (meta.nodes * 16 + (meta.nodes + 1) * 4 + meta.edges * 4 + meta.runs + 3) & ~3;
  new Uint32Array(badRuns.buffer, lengthsOffset, meta.runs)[0] += 1;
  assert.throws(() => decodeFloor({ ...meta, floorId: '01' }, badRuns), /blocked/);
});
