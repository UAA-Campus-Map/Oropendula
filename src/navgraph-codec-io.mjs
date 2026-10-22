/**
 * gzip helpers for navGraph bundles.
 *
 * The codec itself is synchronous and environment-neutral. These helpers use
 * the browser/Node `CompressionStream` API, available in modern browsers and
 * Node 18+.
 */

function asUint8(bytes) {
  if (bytes instanceof Uint8Array) return bytes;
  if (bytes instanceof ArrayBuffer) return new Uint8Array(bytes);
  if (ArrayBuffer.isView(bytes)) {
    return new Uint8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }
  throw new TypeError('navgraph-codec-io: expected binary data');
}

async function collect(stream) {
  const response = new Response(stream);
  return new Uint8Array(await response.arrayBuffer());
}

/** Gzip a bundle payload. */
export async function gzipBytes(bytes) {
  const input = asUint8(bytes);
  const stream = new Blob([input]).stream().pipeThrough(new CompressionStream('gzip'));
  return collect(stream);
}

/** Gunzip a bundle payload. */
export async function gunzipBytes(bytes) {
  const input = asUint8(bytes);
  if (input.byteLength < 2 || input[0] !== 0x1f || input[1] !== 0x8b) {
    throw new Error('navgraph-codec-io: not a gzip stream');
  }
  const stream = new Blob([input]).stream().pipeThrough(new DecompressionStream('gzip'));
  return collect(stream);
}

/** True when the bytes start with the gzip magic. */
export function isGzip(bytes) {
  const input = asUint8(bytes);
  return input.byteLength >= 2 && input[0] === 0x1f && input[1] === 0x8b;
}
