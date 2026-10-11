/**
 * Minimal PMTiles v3 reader for Node smoke tests - no npm dependencies.
 * Implements the header parse + directory walk needed for getZxy, following
 * the v3 spec (the same code the pmtiles library runs). Browser code uses the
 * pmtiles UMD bundle instead; see osm.mjs.
 */
import {gunzipSync} from 'node:zlib';

function rangeFetch(url, start, end) {
  return fetch(url, {
    headers: {Range: `bytes=${start}-${end - 1}`},
  }).then(r => {
    if (!r.ok) throw new Error(`HTTP ${r.status} for ${url} [${start},${end})`);
    return r.arrayBuffer();
  });
}

function readVarint(buf, state) {
  let result = 0, shift = 0;
  for (;;) {
    const b = buf[state.pos++];
    result += (b & 0x7f) * 2 ** shift;
    if (!(b & 0x80)) break;
    shift += 7;
  }
  return result;
}

function rotate(a, x, y, rx, ry) {
  if (ry === 0) {
    if (rx !== 0) return [a - 1 - y, a - 1 - x];
    return [y, x];
  }
  return [x, y];
}

/** Hilbert-curve tile id (pmtiles v3 zxyToTileId). */
function zxyToTileId(z, x, y) {
  let acc = ((1 << z) * (1 << z) - 1) / 3;
  let n = z - 1;
  let [sx, sy] = [x, y];
  for (let a = 1 << n; a > 0; a >>= 1) {
    const rx = sx & a, ry = sy & a;
    acc += (3 * rx ^ ry) * (1 << n);
    [sx, sy] = rotate(a, sx, sy, rx, ry);
    n--;
  }
  return acc;
}

function deserializeDirectory(bytes) {
  const buf = new Uint8Array(bytes);
  const state = {pos: 0};
  const n = readVarint(buf, state);
  const entries = [];
  let lastId = 0;
  for (let i = 0; i < n; i++) {
    const v = readVarint(buf, state);
    entries.push({tileId: lastId + v, offset: 0, length: 0, runLength: 1});
    lastId += v;
  }
  for (let i = 0; i < n; i++) entries[i].runLength = readVarint(buf, state);
  for (let i = 0; i < n; i++) entries[i].length = readVarint(buf, state);
  for (let i = 0; i < n; i++) {
    const v = readVarint(buf, state);
    if (v === 0 && i > 0) entries[i].offset = entries[i - 1].offset + entries[i - 1].length;
    else entries[i].offset = v - 1;
  }
  return entries;
}

function findTile(entries, tileId) {
  // binary search for the last entry with tileId <= id, honouring runLength
  let m = 0, n = entries.length - 1;
  while (m <= n) {
    const k = (n + m) >> 1;
    const cmp = tileId - entries[k].tileId;
    if (cmp > 0) m = k + 1;
    else if (cmp < 0) n = k - 1;
    else return entries[k];
  }
  if (n >= 0 && (entries[n].runLength === 0 || tileId - entries[n].tileId < entries[n].runLength)) {
    return entries[n];
  }
  return null;
}

/**
 * getZxy(url, z, x, y) -> ArrayBuffer of decompressed tile bytes, or null
 * when the tile is outside the archive.
 */
export async function pmtilesGetZxy(url, z, x, y) {
  const headerBytes = await rangeFetch(url, 0, 16384);
  const hv = new DataView(headerBytes);
  const u64 = (off) => hv.getUint32(off + 4, true) * 2 ** 32 + hv.getUint32(off, true);
  const header = {
    rootDirectoryOffset: u64(8),
    rootDirectoryLength: u64(16),
    leafDirectoryOffset: u64(40),
    tileDataOffset: u64(56),
    internalCompression: hv.getUint8(97),
    tileCompression: hv.getUint8(98),
    minZoom: hv.getUint8(100),
    maxZoom: hv.getUint8(101),
  };
  const tileId = zxyToTileId(z, x, y);
  if (z < header.minZoom || z > header.maxZoom) return null;

  const getDir = async (offset, length) => {
    const raw = new Uint8Array(await rangeFetch(url, offset, offset + length));
    const bytes = header.internalCompression === 2 ? gunzipSync(raw) : raw;
    return deserializeDirectory(bytes);
  };

  // root dir first, then leaf directories (deep zooms live in leaves):
  // an entry with runLength 0 points at a leaf dir instead of tile data.
  let offset = header.rootDirectoryOffset;
  let length = header.rootDirectoryLength;
  for (let depth = 0; depth <= 3; depth++) {
    const entries = await getDir(offset, length);
    const entry = findTile(entries, tileId);
    if (!entry) return null;
    if (entry.runLength > 0) {
      const raw = await rangeFetch(
        url, header.tileDataOffset + entry.offset, header.tileDataOffset + entry.offset + entry.length);
      if (header.tileCompression !== 2) return raw;
      return new Uint8Array(gunzipSync(new Uint8Array(raw))).buffer;
    }
    offset = header.leafDirectoryOffset + entry.offset;
    length = entry.length;
  }
  throw new Error('Maximum directory depth exceeded');
}
