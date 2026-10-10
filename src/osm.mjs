/**
 * osm.mjs - OpenStreetMap roads as a translated layer.
 *
 * The campus basemap is a Protomaps extract of OpenStreetMap served as a
 * single PMTiles archive (navigator.cheeseverse.net/scak.pmtiles, covering
 * South Central Alaska at max zoom 15). Instead of adding a new data source,
 * this module reads that archive's `roads` layer for the tiles covering a
 * lon/lat bounding box, keeps the walkable ways, and translates them into the
 * same shape format as the DXF files - a PATHS layer of
 * {bbox, stroke, fill, width, closed, layer, items:[{op:"line", points}]}
 * shapes. Unlike DXF shapes, points are [lon,lat] pairs, so the building
 * transform (DXF -> lon/lat) is the identity for this layer.
 *
 * campus.mjs turns those shapes into a path graph: building exits snap onto
 * it and every outdoor leg follows the real network instead of a straight
 * line. The router makes no assumptions about where the shapes came from, so
 * the OSM translation stays fully swappable.
 *
 * Tile fetching is injected: the browser passes a closure over the pmtiles
 * global the page already loads; Node callers (smoke tests) pass their own.
 */

/** MVT tiles use a 4096-square local coordinate grid. */
const TILE_EXTENT = 4096;

/** Walkable OSM highway kinds, as encoded in Protomaps' `kind` field. */
const WALKABLE_KINDS = new Set(['path', 'minor_road', 'major_road']);
/** `access` values that mean "keep out" (military bases, gated service roads). */
const BLOCKED_ACCESS = new Set(['private', 'no']);

// ---------------------------------------------------------------------------
// Tile math (slippy map)
// ---------------------------------------------------------------------------

/** Tile x/y for a lon/lat at a zoom level. */
export function lonLatToTile(lon, lat, zoom) {
  const n = 2 ** zoom;
  return {
    x: Math.floor((lon + 180) / 360 * n),
    y: Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) +
      1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * n),
  };
}

/** Inclusive tile x/y range covering a lon/lat bbox at a zoom level. */
export function tileRange(bbox, zoom) {
  const n = 2 ** zoom;
  const toY = lat => Math.floor((1 - Math.log(
    Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * n);
  const yA = toY(bbox.minLat), yB = toY(bbox.maxLat);
  return {
    zoom,
    minX: Math.floor((bbox.minLon + 180) / 360 * n),
    maxX: Math.floor((bbox.maxLon + 180) / 360 * n),
    minY: Math.min(yA, yB),
    maxY: Math.max(yA, yB),
  };
}

/**
 * MVT tile-local pixel -> lon/lat. Pixels may fall outside the tile (features
 * bleed into each other's buffers); the projection handles them unchanged.
 */
export function tilePixelToLL(zoom, tx, ty, px, py) {
  const n = 2 ** zoom;
  const lon = (tx + px / TILE_EXTENT) / n * 360 - 180;
  const lat = Math.atan(Math.sinh(
    Math.PI * (1 - 2 * (ty + py / TILE_EXTENT) / n))) * 180 / Math.PI;
  return [lon, lat];
}

// ---------------------------------------------------------------------------
// Minimal MVT decoder
//
// The Mapbox vector tile spec is a small protobuf profile: a Tile message
// holds Layer messages, a Layer holds packed keys/values plus Feature
// messages, and each feature carries a zig-zag delta-encoded command stream.
// This is the subset needed for roads - roughly 80 lines - so the project
// needs no extra runtime dependency.
// ---------------------------------------------------------------------------

/** Sequential protobuf reader over {bytes, pos, end}. */
class Reader {
  constructor(bytes, pos = 0, end = bytes.length) {
    this.b = bytes; this.p = pos; this.e = end;
  }
  get done() { return this.p >= this.e; }
  varint() {
    let v = 0, shift = 0;
    for (;;) {
      const byte = this.b[this.p++];
      v += (byte & 0x7f) * 2 ** shift;
      if (!(byte & 0x80)) break;
      shift += 7;
    }
    return v;
  }
  skip(wire) {
    if (wire === 0) this.varint();
    else if (wire === 1) this.p += 8;
    else if (wire === 2) this.p += this.varint();
    else if (wire === 5) this.p += 4;
    else throw new Error(`MVT: unsupported wire type ${wire}`);
  }
  /** Consume a length-delimited field, returning a reader over its bytes. */
  sub() {
    const len = this.varint();
    const r = new Reader(this.b, this.p, this.p + len);
    this.p += len;
    return r;
  }
  string() {
    const len = this.varint();
    const s = new TextDecoder().decode(this.b.subarray(this.p, this.p + len));
    this.p += len;
    return s;
  }
}

/** Zig-zag decode of MVT command deltas. */
function zigzag(n) {
  return n & 1 ? -((n + 1) >> 1) : n >> 1;
}

/**
 * MVT geometry command stream -> polylines in tile pixels.
 * Command ids: 1 MoveTo, 2 LineTo, 7 ClosePath (closed shapes are ignored).
 * @returns {[[[number,number],...],...]} one polyline per MoveTo-separated part
 */
function decodeGeometry(geom) {
  const parts = [];
  let part = null;
  let x = 0, y = 0, i = 0;
  while (i < geom.length) {
    const cmd = geom[i++];
    const op = cmd & 7, count = cmd >> 3;
    if (op === 1) {
      for (let k = 0; k < count; k++) {
        x += zigzag(geom[i++]); y += zigzag(geom[i++]);
        part = [[x, y]]; parts.push(part);
      }
    } else if (op === 2) {
      for (let k = 0; k < count; k++) {
        x += zigzag(geom[i++]); y += zigzag(geom[i++]);
        if (part) part.push([x, y]);
      }
    } else if (op === 7) {
      part = null;   // ClosePath: end this part
    } else break;    // unknown command - stop
  }
  return parts;
}

function decodeLayer(r) {
  const layer = {name: '', extent: TILE_EXTENT, keys: [], values: [], features: []};
  while (!r.done) {
    const tag = r.varint();
    const field = tag >> 3, wire = tag & 7;
    if (field === 1 && wire === 2) layer.name = r.string();
    else if (field === 2 && wire === 2) layer.features.push(decodeFeature(r.sub()));
    else if (field === 3 && wire === 2) layer.keys.push(r.string());
    else if (field === 4 && wire === 2) layer.values.push(decodeValue(r.sub()));
    else if (field === 5 && wire === 0) layer.extent = r.varint();
    else r.skip(wire);
  }
  return layer;
}

function decodeFeature(r) {
  const f = {tags: [], type: 0, geometry: []};
  while (!r.done) {
    const tag = r.varint();
    const field = tag >> 3, wire = tag & 7;
    if (field === 2 && wire === 2) {   // packed tags
      const sub = r.sub();
      while (!sub.done) f.tags.push(sub.varint());
    } else if (field === 3 && wire === 0) f.type = r.varint();
    else if (field === 4 && wire === 2) {   // packed geometry
      const sub = r.sub();
      while (!sub.done) f.geometry.push(sub.varint());
    } else r.skip(wire);
  }
  return f;
}

function decodeValue(r) {
  const v = {};
  while (!r.done) {
    const tag = r.varint();
    const field = tag >> 3, wire = tag & 7;
    if (field === 1 && wire === 2) v.string = r.string();
    else if (wire === 0) v.number = r.varint();
    else if (wire === 2) { v.bytes = true; r.skip(wire); }   // float/double - unused
    else r.skip(wire);
  }
  return v;
}

/** Decode one uncompressed MVT tile into per-feature road attributes.
 *  @returns {{kind,kindDetail,access,name,polylines}[]} polylines in tile pixels */
export function decodeMvtRoads(bytes) {
  const r = new Reader(bytes);
  let roads = null;
  while (!r.done) {
    const tag = r.varint();
    const field = tag >> 3, wire = tag & 7;
    if (field === 3 && wire === 2) {
      const layer = decodeLayer(r.sub());
      if (layer.name === 'roads') { roads = layer; break; }
    } else r.skip(wire);
  }
  if (!roads) return [];

  const features = [];
  for (const f of roads.features) {
    if (f.type !== 2) continue;   // LineString only
    const props = {};
    for (let i = 0; i + 1 < f.tags.length; i += 2) {
      const v = roads.values[f.tags[i + 1]];
      props[roads.keys[f.tags[i]]] = v.string !== undefined ? v.string : v.number;
    }
    features.push({
      kind: props.kind,
      kindDetail: props.kind_detail,
      access: props.access,
      name: props.name,
      polylines: decodeGeometry(f.geometry),
    });
  }
  return features;
}

// ---------------------------------------------------------------------------
// Compression
// ---------------------------------------------------------------------------

/** Gzip magic bytes; PMTiles stores every tile gzipped, but stay defensive. */
export function isGzip(bytes) {
  return bytes.length > 2 && bytes[0] === 0x1f && bytes[1] === 0x8b;
}

/**
 * Gzip-decompress tile bytes. Uses DecompressionStream where available
 * (browsers) and Node's zlib otherwise.
 * @param {Uint8Array|ArrayBuffer} bytes
 * @returns {Promise<Uint8Array>}
 */
export async function gunzip(bytes) {
  const buf = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
  if (typeof DecompressionStream !== 'undefined') {
    const stream = new Blob([buf]).stream()
      .pipeThrough(new DecompressionStream('gzip'));
    return new Uint8Array(await new Response(stream).arrayBuffer());
  }
  const {gunzipSync} = await import('node:zlib');
  return new Uint8Array(gunzipSync(Buffer.from(buf)));
}

/** Decompress only when needed. */
export async function maybeGunzip(bytes) {
  const buf = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes);
  return isGzip(buf) ? gunzip(buf) : buf;
}

// ---------------------------------------------------------------------------
// OSM -> our shape format
// ---------------------------------------------------------------------------

/**
 * Translate decoded MVT tiles into shapes on the PATHS layer.
 *
 * @param {{z:number,x:number,y:number,data:Uint8Array|ArrayBuffer}[]} tiles
 * @param {{walkableKinds?:Set<string>}} [opts]
 * @returns {{shapes:object[], stats:{tiles:number,ways:number,
 *            duplicateWays:number, blockedAccessWays:number}}}
 */
export function tilesToShapes(tiles, opts = {}) {
  const walkable = opts.walkableKinds ?? WALKABLE_KINDS;
  const shapes = [];
  const seen = new Set();
  let ways = 0, duplicateWays = 0, blockedAccessWays = 0;

  for (const tile of tiles) {
    const data = tile.data instanceof Uint8Array ? tile.data : new Uint8Array(tile.data);
    for (const f of decodeMvtRoads(data)) {
      if (!walkable.has(f.kind)) continue;
      if (BLOCKED_ACCESS.has(f.access)) { blockedAccessWays++; continue; }
      for (const line of f.polylines) {
        if (line.length < 2) continue;
        const points = line.map(([px, py]) =>
          tilePixelToLL(tile.z, tile.x, tile.y, px, py));
        // Ways that straddle a tile edge appear in both tiles' buffers with
        // identical geometry; keep one copy.
        const key = f.kind + '|' + points
          .map(p => p[0].toFixed(6) + ',' + p[1].toFixed(6)).join(';');
        if (seen.has(key)) { duplicateWays++; continue; }
        seen.add(key);
        let lon0 = Infinity, lat0 = Infinity, lon1 = -Infinity, lat1 = -Infinity;
        for (const [lon, lat] of points) {
          if (lon < lon0) lon0 = lon;
          if (lon > lon1) lon1 = lon;
          if (lat < lat0) lat0 = lat;
          if (lat > lat1) lat1 = lat;
        }
        shapes.push({
          bbox: [lon0, lat0, lon1, lat1],
          stroke: null, fill: null, width: null, closed: false,
          layer: 'PATHS',
          items: [{op: 'line', points}],
        });
        ways++;
      }
    }
  }
  return {
    shapes,
    stats: {tiles: tiles.length, ways, duplicateWays, blockedAccessWays},
  };
}

/**
 * Fetch the walkable OSM road network for a lon/lat bbox and translate it
 * into our shape format.
 *
 * @param {{url:string, bbox:{minLon:number,minLat:number,maxLon:number,maxLat:number},
 *          zoom?:number, fetchTile?:(z,x,y)=>Promise<ArrayBuffer|null>}} opts
 *        fetchTile defaults to a reader over the pmtiles global the app loads.
 * @returns {Promise<{shapes:object[], stats:object, range:object}>}
 */
export async function loadOsmShapes({url, bbox, zoom = 15, fetchTile}) {
  const get = fetchTile ?? pmtilesFetcher(url);
  const range = tileRange(bbox, zoom);
  const tiles = [];
  for (let x = range.minX; x <= range.maxX; x++) {
    for (let y = range.minY; y <= range.maxY; y++) {
      let raw = null;
      try {
        raw = await get(zoom, x, y);
      } catch {
        raw = null;   // tiles off the coast / outside the extract are fine
      }
      if (!raw) continue;
      tiles.push({z: zoom, x, y, data: await maybeGunzip(raw)});
    }
  }
  const {shapes, stats} = tilesToShapes(tiles);
  return {shapes, stats, range};
}

/**
 * A fetchTile backed by the pmtiles UMD bundle (window.pmtiles). The page
 * registers that bundle for the basemap already; PMTiles#getZxy does the
 * HTTP range requests into the archive for us.
 */
export function pmtilesFetcher(url) {
  const lib = globalThis.pmtiles;
  if (!lib || !lib.PMTiles) {
    throw new Error('pmtiles global not available - pass an explicit fetchTile');
  }
  const archive = new lib.PMTiles(url);
  return async (z, x, y) => {
    const r = await archive.getZxy(z, x, y);
    return r ? r.data : null;
  };
}
