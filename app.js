'use strict';
/* ============================================================================
 * quilojaules — per-segment cycling energy calculator
 * Static, no build step. Sections:
 *   1. Physics            (ported from old-applet/energy-kj.html)
 *   2. FABDEM elevation   (ported from pedalhidrografico/web/app.js)
 *   3. cmocean.phase LUT
 *   4. Coloured topo GridLayer
 *   5. Map + layers
 *   6. Route input (draw + GPX)
 *   7. Clipping range slider
 *   8. Segments (splits + table + per-segment compute)
 *   9. Render + persistence
 * ========================================================================== */

// ============================================================================
// 1. Physics  (verbatim from the old applet — self-contained, profile-driven)
// ============================================================================
const G = 9.80665;
const R_EARTH = 6371008.8;

function haversine(lat1, lon1, lat2, lon2) {
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat / 2) ** 2
          + Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180)
          * Math.sin(dLon / 2) ** 2;
  return 2 * R_EARTH * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function segmentForces(distM, dEle, speed, p) {
  if (distM <= 0 || speed <= 0) return null;
  const m = p.riderKg + p.bikeKg;
  const dE = Math.max(Math.min(dEle, distM), -distM);
  const sin = dE / distM;
  const cos = Math.sqrt(Math.max(1 - sin * sin, 0));
  const fRoll = p.crr * m * G * cos;
  const fGrav = m * G * sin;
  const vEff  = speed + p.headwindMs;
  const fAero = 0.5 * p.rho * p.cda * vEff * Math.abs(vEff);
  const pWheel = (fRoll + fGrav + fAero) * speed;
  if (pWheel <= 0) return null;
  const dt = distM / speed;
  return {
    pPedal: pWheel / p.eff, dt,
    eRoll: Math.max(fRoll, 0) * speed * dt / p.eff,
    eAero: Math.max(fAero, 0) * speed * dt / p.eff,
    eGrav: Math.max(fGrav, 0) * speed * dt / p.eff,
  };
}

function targetPower(distM, dEle, p) {
  if (distM <= 0) return p.powerFlatW;
  const gradePct = (dEle / distM) * 100;
  if (gradePct >  p.gradeThPct) return p.powerClimbW;
  if (gradePct < -p.gradeThPct) return p.powerDescW;
  return p.powerFlatW;
}

function pedalPowerAtSpeed(distM, dEle, v, p) {
  if (v <= 0) return 0;
  const m = p.riderKg + p.bikeKg;
  const dE = Math.max(Math.min(dEle, distM), -distM);
  const sin = dE / distM;
  const cos = Math.sqrt(Math.max(1 - sin * sin, 0));
  const fRoll = p.crr * m * G * cos;
  const fGrav = m * G * sin;
  const vEff  = v + p.headwindMs;
  const fAero = 0.5 * p.rho * p.cda * vEff * Math.abs(vEff);
  const pWheel = (fRoll + fGrav + fAero) * v;
  return pWheel > 0 ? pWheel / p.eff : 0;
}

function solveSpeedForPower(distM, dEle, targetW, p) {
  const V_MAX = 25, V_MIN = 0.5;
  const V_CAP = p.maxDescentMs > 0 ? p.maxDescentMs : V_MAX;
  if (targetW <= 0) return Math.min(coastingSpeed(distM, dEle, p), V_CAP);
  let lo = V_MIN, hi = V_MAX;
  if (pedalPowerAtSpeed(distM, dEle, hi, p) < targetW) return Math.min(hi, V_CAP);
  if (pedalPowerAtSpeed(distM, dEle, lo, p) > targetW) return lo;
  for (let i = 0; i < 40; i++) {
    const mid = 0.5 * (lo + hi);
    const pw = pedalPowerAtSpeed(distM, dEle, mid, p);
    if (pw < targetW) lo = mid; else hi = mid;
    if (hi - lo < 1e-3) break;
  }
  return Math.min(0.5 * (lo + hi), V_CAP);
}

function coastingSpeed(distM, dEle, p) {
  const m = p.riderKg + p.bikeKg;
  const dE = Math.max(Math.min(dEle, distM), -distM);
  const sin = dE / distM;
  const cos = Math.sqrt(Math.max(1 - sin * sin, 0));
  const fGravDown = -m * G * sin;
  const fRoll     =  p.crr * m * G * cos;
  if (fGravDown <= fRoll) return 1.0;
  let lo = 0.1, hi = 30;
  for (let i = 0; i < 40; i++) {
    const mid = 0.5 * (lo + hi);
    const vEff = mid + p.headwindMs;
    const fAero = 0.5 * p.rho * p.cda * vEff * Math.abs(vEff);
    if (fAero < fGravDown - fRoll) lo = mid; else hi = mid;
    if (hi - lo < 1e-3) break;
  }
  return 0.5 * (lo + hi);
}

function analyze(points, p, opts) {
  const ignoreTimestamps = opts && opts.ignoreTimestamps === true;
  const r = {
    energyKJ: 0, distanceKm: 0, ascentM: 0, descentM: 0, durationS: 0,
    avgPowerW: 0, kcal: 0, samples: [], energyJ: 0,
    eRoll: 0, eAero: 0, eGrav: 0,
  };
  if (!points || points.length < 2) return r;
  let energyJ = 0, totDt = 0, dist = 0, asc = 0, desc = 0, cumDist = 0;
  // `t` is cumulative MOVING time (s) — used by the time-binned power profile.
  r.samples.push({ d: 0, t: 0, ele: points[0].ele || 0, p: 0, lat: points[0].lat, lon: points[0].lon });

  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    const horiz = haversine(a.lat, a.lon, b.lat, b.lon);
    const dEle  = (b.ele - a.ele) || 0;
    const seg   = Math.sqrt(horiz * horiz + dEle * dEle);
    if (seg < 0.1) continue;

    let speed, dt;
    if (!ignoreTimestamps && a.t > 0 && b.t > a.t) {
      dt    = (b.t - a.t) / 1000;
      speed = seg / Math.max(dt, 0.1);
      if (speed < 0.5) continue;
    } else {
      const tgtW = targetPower(seg, dEle, p);
      speed = solveSpeedForPower(seg, dEle, tgtW, p);
      dt    = seg / speed;
    }

    const f = segmentForces(seg, dEle, speed, p);
    let segPower = 0;
    if (f) {
      energyJ += f.pPedal * f.dt;
      segPower = f.pPedal;
      r.eRoll += f.eRoll; r.eAero += f.eAero; r.eGrav += f.eGrav;
    }
    dist += seg; totDt += dt; cumDist += seg;
    if (dEle > 0) asc += dEle; else desc -= dEle;
    r.samples.push({ d: cumDist, t: totDt, ele: b.ele, p: segPower, lat: b.lat, lon: b.lon });
  }
  r.energyJ    = energyJ;
  r.energyKJ   = energyJ / 1000;
  r.distanceKm = dist / 1000;
  r.ascentM    = asc;
  r.descentM   = desc;
  r.durationS  = totDt;
  r.avgPowerW  = totDt > 0 ? energyJ / totDt : 0;
  r.kcal       = (r.energyKJ / 0.24) / 4.184;
  return r;
}

// ============================================================================
// 2. FABDEM elevation  (ported from pedalhidrografico/web/app.js)
// ============================================================================
const FABDEM_BASE_URL = 'https://telhas.pedalhidrografi.co/fabdem/';
const FABDEM_ARCSEC   = 1 / 3600;   // ~30 m at the equator

let _geoTiffPromise = null;
function ensureGeoTIFF() {
  if (!_geoTiffPromise) {
    _geoTiffPromise = (async () => {
      if (!window.GeoTIFF) throw new Error('geotiff.js não carregou');
      return window.GeoTIFF;
    })();
  }
  return _geoTiffPromise;
}

// Bucket convention: SW corner, hemisphere before the digits.
//   lat=-24, lon=-47  ->  S24W047_FABDEM_V1-2.tif
function fabdemTileName(lat, lon) {
  const ns = lat >= 0 ? 'N' : 'S';
  const ew = lon >= 0 ? 'E' : 'W';
  const la = String(Math.abs(lat)).padStart(2, '0');
  const lo = String(Math.abs(lon)).padStart(3, '0');
  return `${ns}${la}${ew}${lo}_FABDEM_V1-2.tif`;
}

const _fabdemTileCache = new Map();   // "latLo_lonLo" -> entry | null
async function openFabdemTile(latLo, lonLo) {
  const key = `${latLo}_${lonLo}`;
  if (_fabdemTileCache.has(key)) return _fabdemTileCache.get(key);
  const url = FABDEM_BASE_URL + fabdemTileName(latLo, lonLo);
  try {
    const GeoTIFF = await ensureGeoTIFF();
    const tiff   = await GeoTIFF.fromUrl(url);
    const image  = await tiff.getImage();
    const origin = image.getOrigin();
    const resolution = image.getResolution();
    const nodataRaw = image.fileDirectory.getValue
      ? image.fileDirectory.getValue('GDAL_NODATA')
      : image.fileDirectory.GDAL_NODATA;
    const nodata = nodataRaw ? parseFloat(nodataRaw) : null;
    const entry = { tiff, image, origin, resolution, nodata,
      width: image.getWidth(), height: image.getHeight() };
    _fabdemTileCache.set(key, entry);
    return entry;
  } catch (e) {
    console.info(`[fabdem] tile (${latLo},${lonLo}) indisponível: ${e.message}`);
    _fabdemTileCache.set(key, null);
    return null;
  }
}

// Sample many points: groups by 1° tile, one windowed read per tile.
async function sampleFabdemBatch(points /* [[lat, lng], …] */) {
  if (!points.length) return [];
  const groups = new Map();
  points.forEach(([lat, lng], i) => {
    const latLo = Math.floor(lat), lonLo = Math.floor(lng);
    const k = `${latLo}_${lonLo}`;
    if (!groups.has(k)) groups.set(k, { latLo, lonLo, idxs: [] });
    groups.get(k).idxs.push(i);
  });
  const out = new Array(points.length).fill(null);
  for (const { latLo, lonLo, idxs } of groups.values()) {
    const t = await openFabdemTile(latLo, lonLo);
    if (!t) continue;
    const [oX, oY] = t.origin;
    const [rX, rY] = t.resolution;
    let cMin = Infinity, cMax = -Infinity, rMin = Infinity, rMax = -Infinity;
    const cells = idxs.map((i) => {
      const [lat, lng] = points[i];
      const c = Math.round((lng - oX) / rX);
      const r = Math.round((lat - oY) / rY);
      if (c < cMin) cMin = c; if (c > cMax) cMax = c;
      if (r < rMin) rMin = r; if (r > rMax) rMax = r;
      return [i, r, c];
    });
    try {
      const wndW = cMax - cMin + 1;
      const ras = await t.image.readRasters({
        window: [cMin, rMin, cMax + 1, rMax + 1], interleave: true,
      });
      for (const [i, r, c] of cells) {
        const v = ras[(r - rMin) * wndW + (c - cMin)];
        if (Number.isFinite(v) && (t.nodata == null || v !== t.nodata)) out[i] = v;
      }
    } catch (e) {
      console.warn(`[fabdem] read window (${latLo},${lonLo}) falhou: ${e.message}`);
    }
  }
  return out;
}

// Open-Meteo fallback for points FABDEM couldn't resolve.
async function sampleOpenMeteoBatch(points) {
  const out = new Array(points.length).fill(null);
  const BATCH = 100;
  for (let i = 0; i < points.length; i += BATCH) {
    const batch = points.slice(i, i + BATCH);
    const lats = batch.map(([la]) => la.toFixed(5)).join(',');
    const lons = batch.map(([, lo]) => lo.toFixed(5)).join(',');
    const url = `https://api.open-meteo.com/v1/elevation?latitude=${lats}&longitude=${lons}`;
    try {
      const res = await fetch(url);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      const elevs = Array.isArray(data.elevation) ? data.elevation : [];
      batch.forEach((_, j) => { if (Number.isFinite(elevs[j])) out[i + j] = elevs[j]; });
    } catch (e) {
      console.warn('Open-Meteo falhou:', e.message);
      break;
    }
  }
  return out;
}

// Fill `ele` on a points array from FABDEM, falling back to Open-Meteo.
async function fillElevations(points) {
  const coords = points.map((p) => [p.lat, p.lon]);
  let elevs = await sampleFabdemBatch(coords);
  const missing = [];
  elevs.forEach((e, i) => { if (!Number.isFinite(e)) missing.push(i); });
  if (missing.length) {
    const fb = await sampleOpenMeteoBatch(missing.map((i) => coords[i]));
    missing.forEach((idx, j) => { if (Number.isFinite(fb[j])) elevs[idx] = fb[j]; });
  }
  points.forEach((p, i) => { if (Number.isFinite(elevs[i])) p.ele = elevs[i]; else if (p.ele == null) p.ele = 0; });
  return points;
}

// ============================================================================
// 3. cmocean.phase colormap (embedded cyclic anchors, interpolated to 256)
// ============================================================================
// Anchors approximating cmocean's cyclic "phase" palette (hue cycle, last == first).
const PHASE_ANCHORS = [
  [167, 121,  16], [192,  90,  55], [197,  66, 123], [160,  86, 196],
  [ 90, 123, 214], [ 52, 160, 184], [ 78, 172,  75], [140, 156,  20],
  [167, 121,  16],
];
const PHASE_LUT = (() => {
  const N = 256, lut = new Uint8Array(N * 3);
  const segs = PHASE_ANCHORS.length - 1;
  for (let i = 0; i < N; i++) {
    const x = (i / (N - 1)) * segs;
    const k = Math.min(Math.floor(x), segs - 1);
    const f = x - k;
    const a = PHASE_ANCHORS[k], b = PHASE_ANCHORS[k + 1];
    lut[i * 3]     = Math.round(a[0] + (b[0] - a[0]) * f);
    lut[i * 3 + 1] = Math.round(a[1] + (b[1] - a[1]) * f);
    lut[i * 3 + 2] = Math.round(a[2] + (b[2] - a[2]) * f);
  }
  return lut;
})();
function phaseColor(t) {
  const i = Math.max(0, Math.min(255, Math.round(t * 255)));
  return [PHASE_LUT[i * 3], PHASE_LUT[i * 3 + 1], PHASE_LUT[i * 3 + 2]];
}
function phaseGradientCss(stops = 8) {
  const parts = [];
  for (let i = 0; i <= stops; i++) {
    const [r, g, b] = phaseColor(i / stops);
    parts.push(`rgb(${r},${g},${b}) ${(i / stops * 100).toFixed(0)}%`);
  }
  return parts.join(', ');
}

// ============================================================================
// 4. Coloured topographical GridLayer
//    bottom = elevation via cmocean.phase; top = slope% grey, multiply blend.
// ============================================================================
const topoSettings = { minElevation: 720, maxElevation: 1150, maxSlope: 15, maxSampleMB: 5 };

// Render-scoped cache: per (FABDEM tile, overview level) the elevation raster
// for the CURRENT viewport region, read ONCE and shared by every map tile in
// the render. Without this, each 256px map tile re-fetches the same coarse COG
// tile (≈ N× the data). Reset whenever the viewport key changes.
let _topoRegion = { key: null, map: new Map() };

// NOTE: returns a PROMISE stored synchronously, so the ~dozen map tiles that
// all request the same region in the same frame share ONE read (no stampede).
function getTopoRegion(tLat, tLon, level, vb, vpKey) {
  if (_topoRegion.key !== vpKey) _topoRegion = { key: vpKey, map: new Map() };
  const ck = `${tLat}_${tLon}_${level}`;
  if (_topoRegion.map.has(ck)) return _topoRegion.map.get(ck);
  const p = (async () => {
    const t = await openFabdemTile(tLat, tLon);
    if (!t) return null;
    const img = await getFabdemOverview(t, level);
    if (!img) return null;
    const ovW = img.getWidth(), ovH = img.getHeight();
    const sx = ovW / t.width, sy = ovH / t.height;
    const [oX, oY] = t.origin, [rX, rY] = t.resolution;
    const west  = Math.max(vb.west,  tLon),     east  = Math.min(vb.east,  tLon + 1);
    const north = Math.min(vb.north, tLat + 1), south = Math.max(vb.south, tLat);
    if (east <= west || north <= south) return null;
    const left   = Math.max(0,   Math.floor(((west  - oX) / rX) * sx));
    const right  = Math.min(ovW, Math.ceil (((east  - oX) / rX) * sx));
    const top    = Math.max(0,   Math.floor(((north - oY) / rY) * sy));
    const bottom = Math.min(ovH, Math.ceil (((south - oY) / rY) * sy));
    if (right - left < 1 || bottom - top < 1) return null;
    let ras;
    try { ras = await img.readRasters({ window: [left, top, right, bottom], interleave: true }); }
    catch (e) { console.warn('[topo] readRasters falhou:', e.message); return null; }
    return {
      ras, rW: right - left, rH: bottom - top, nodata: t.nodata,
      west: oX + (left / sx) * rX, north: oY + (top / sy) * rY,
      dLon: rX / sx, dLat: rY / sy,
    };
  })();
  _topoRegion.map.set(ck, p);
  return p;
}

// Build the elevation grid for one map tile (`bounds`) by SAMPLING the cached
// per-viewport overview regions. `dec` selects the overview (download bound).
// Returns { grid:Float32Array, gW, gH }, NaN where uncovered. Slope is computed
// later on this grid (a real DEM at the read resolution → no staircase).
async function readTopoGrid(bounds, dec, vb, vpKey) {
  await ensureGeoTIFF();
  const A = FABDEM_ARCSEC;
  const READ_MAX = 512;
  const nativeW = Math.max(2, Math.round((bounds.east - bounds.west) / A));
  const nativeH = Math.max(2, Math.round((bounds.north - bounds.south) / A));
  const gW = Math.max(2, Math.min(READ_MAX, Math.round(nativeW / dec)));
  const gH = Math.max(2, Math.min(READ_MAX, Math.round(nativeH / dec)));
  const level = Math.round(Math.log2(dec));
  const grid = new Float32Array(gW * gH);
  grid.fill(NaN);
  const latLo = Math.floor(bounds.south), latHi = Math.floor(bounds.north - 1e-9);
  const lonLo = Math.floor(bounds.west),  lonHi = Math.floor(bounds.east  - 1e-9);

  for (let tLat = latLo; tLat <= latHi; tLat++) {
    for (let tLon = lonLo; tLon <= lonHi; tLon++) {
      const reg = await getTopoRegion(tLat, tLon, level, vb, vpKey);
      if (!reg) continue;
      // Sample each grid cell of this map tile from the region raster.
      for (let gy = 0; gy < gH; gy++) {
        const lat = bounds.north - ((gy + 0.5) / gH) * (bounds.north - bounds.south);
        const rr = Math.floor((lat - reg.north) / reg.dLat);
        if (rr < 0 || rr >= reg.rH) continue;
        for (let gx = 0; gx < gW; gx++) {
          const lon = bounds.west + ((gx + 0.5) / gW) * (bounds.east - bounds.west);
          const rc = Math.floor((lon - reg.west) / reg.dLon);
          if (rc < 0 || rc >= reg.rW) continue;
          const v = reg.ras[rr * reg.rW + rc];
          if (Number.isFinite(v) && (reg.nodata == null || v !== reg.nodata)) {
            grid[gy * gW + gx] = v;
          }
        }
      }
    }
  }
  return { grid, gW, gH };
}

// Lazily open + cache an overview IFD (0 = full res). Clamps to the coarsest.
async function getFabdemOverview(t, level) {
  if (!t._ovCache) t._ovCache = {};
  const L = Math.max(0, level);
  if (t._ovCache[L] !== undefined) return t._ovCache[L];
  try {
    const count = await t.tiff.getImageCount();
    const idx = Math.min(L, count - 1);
    const img = idx === 0 ? t.image : await t.tiff.getImage(idx);
    t._ovCache[L] = img;
    return img;
  } catch (e) {
    t._ovCache[L] = null;
    return null;
  }
}

const FabdemTopoLayer = L.GridLayer.extend({
  createTile(coords, done) {
    const tile = document.createElement('canvas');
    const size = this.getTileSize();
    tile.width = size.x; tile.height = size.y;
    this._draw(coords, size, tile, done);
    return tile;
  },
  _bounds(coords, size) {
    const map = this._map;
    const nwPt = coords.scaleBy(size);
    const sePt = nwPt.add(size);
    const nw = map.unproject(nwPt, coords.z);
    const se = map.unproject(sePt, coords.z);
    return { north: nw.lat, west: nw.lng, south: se.lat, east: se.lng };
  },
  // Choose ONE overview decimation (1,2,4,8) for the whole current viewport so a
  // full map render stays under the byte cap. Download scales with the overview
  // geotiff fetches, so we estimate COG-tile fetches across the viewport at each
  // level and pick the finest that fits. ~per-tile fetch cost is measured-ish.
  _viewportDec(capBytes) {
    if (!this._map || capBytes <= 0) return 1;   // cap 0 = sem limite
    // Download is dominated by whole 512² COG-tile fetches, not sample count, so
    // a per-overview cost-per-deg² table (calibrated from measured fetches over
    // São Paulo FABDEM) predicts it far better than a smooth model. Index = level
    // L (dec = 2^L). Conservative; real (compressed) fetch ≤ these.
    const PER_DEG2 = [210e6, 168e6, 78e6, 35e6];   // bytes/deg² at L0..L3
    const vb = this._map.getBounds();
    const W = vb.getWest(), E = vb.getEast(), S = vb.getSouth(), N = vb.getNorth();
    const latLo = Math.floor(S), latHi = Math.floor(N - 1e-9);
    const lonLo = Math.floor(W), lonHi = Math.floor(E - 1e-9);
    let area = 0;   // deg² of viewport ∩ covered FABDEM tiles
    for (let la = latLo; la <= latHi; la++) {
      for (let lo = lonLo; lo <= lonHi; lo++) {
        const w = Math.min(E, lo + 1) - Math.max(W, lo);
        const h = Math.min(N, la + 1) - Math.max(S, la);
        if (w > 0 && h > 0) area += w * h;
      }
    }
    for (let L = 0; L <= 3; L++) {
      if (PER_DEG2[L] * area <= capBytes) return 1 << L;
    }
    return 8;
  },
  async _draw(coords, size, tile, done) {
    const W = size.x, H = size.y;
    try {
      const b = this._bounds(coords, size);
      // Per-MAP byte cap → one overview level for the whole view (0 = sem limite).
      const capBytes = (topoSettings.maxSampleMB || 0) * 1e6;
      const dec = this._viewportDec(capBytes);
      const vb = this._map.getBounds();
      const viewBounds = { west: vb.getWest(), east: vb.getEast(), south: vb.getSouth(), north: vb.getNorth() };
      const vpKey = `${coords.z}_${viewBounds.west.toFixed(3)}_${viewBounds.south.toFixed(3)}_${viewBounds.east.toFixed(3)}_${viewBounds.north.toFixed(3)}_${dec}`;
      const { grid, gW, gH } = await readTopoGrid(b, dec, viewBounds, vpKey);
      const { minElevation: minE, maxElevation: maxE, maxSlope } = topoSettings;
      const eRange = Math.max(maxE - minE, 1e-6);
      // Ground spacing of the READ grid (slope is computed at this resolution,
      // i.e. on a real DEM — not on a nearest-neighbour upsample).
      const latMid = (b.north + b.south) / 2;
      const metersY = ((b.north - b.south) / gH) * (Math.PI / 180) * R_EARTH;
      const metersX = ((b.east - b.west) / gW) * (Math.PI / 180) * R_EARTH
                    * Math.cos(latMid * Math.PI / 180);
      const SLOPE_FLOOR = 0.18;   // steepest cells keep ~18% brightness (multiply)

      // Colour the read grid into a gW×gH ImageData.
      const lo = document.createElement('canvas');
      lo.width = gW; lo.height = gH;
      const loCtx = lo.getContext('2d');
      const img = loCtx.createImageData(gW, gH);
      const data = img.data;
      for (let r = 0; r < gH; r++) {
        for (let c = 0; c < gW; c++) {
          const idx = r * gW + c;
          const e = grid[idx];
          const o = idx * 4;
          if (!Number.isFinite(e)) { data[o + 3] = 0; continue; }
          const t = Math.max(0, Math.min(1, (e - minE) / eRange));
          const [br, bg, bb] = phaseColor(t);
          const cl = Math.max(0, c - 1), cr = Math.min(gW - 1, c + 1);
          const ru = Math.max(0, r - 1), rd = Math.min(gH - 1, r + 1);
          const eL = grid[r * gW + cl], eR = grid[r * gW + cr];
          const eU = grid[ru * gW + c], eD = grid[rd * gW + c];
          let slopePct = 0;
          if (Number.isFinite(eL) && Number.isFinite(eR) &&
              Number.isFinite(eU) && Number.isFinite(eD)) {
            const dzdx = (eR - eL) / (((cr - cl) || 1) * metersX);
            const dzdy = (eD - eU) / (((rd - ru) || 1) * metersY);
            slopePct = Math.sqrt(dzdx * dzdx + dzdy * dzdy) * 100;
          }
          const sNorm = Math.max(0, Math.min(1, slopePct / Math.max(maxSlope, 1e-6)));
          const grey = 1 - sNorm * (1 - SLOPE_FLOOR);
          data[o]     = Math.round(br * grey);
          data[o + 1] = Math.round(bg * grey);
          data[o + 2] = Math.round(bb * grey);
          data[o + 3] = 255;
        }
      }
      loCtx.putImageData(img, 0, 0);
      // Smoothly scale the read grid up to the 256px tile (bilinear) — no blocks.
      const ctx = tile.getContext('2d');
      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = 'high';
      ctx.clearRect(0, 0, W, H);
      ctx.drawImage(lo, 0, 0, gW, gH, 0, 0, W, H);
      done(null, tile);
    } catch (e) {
      console.warn('[topo] tile falhou:', e.message);
      done(e, tile);
    }
  },
});

// ============================================================================
// 5. Map + layers
// ============================================================================
const SP_CENTER = [-23.55, -46.63];
const map = L.map('map', { zoomControl: true, scrollWheelZoom: false }).setView(SP_CENTER, 12);

const osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19, attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);
const satellite = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19, attribution: 'Imagery © Esri, Maxar, Earthstar Geographics' },
);
const topo = new FabdemTopoLayer({ maxZoom: 19, minZoom: 10, attribution: 'Topografia: FABDEM · Pedal Hidrográfico' });

const BASE_LAYERS = { 'OpenStreetMap': osm, 'Satélite': satellite, 'Topográfica colorida': topo };
// POIs from the loaded GPX — a toggleable overlay (checkbox in the control).
const poiLayer = L.layerGroup().addTo(map);
L.control.layers(BASE_LAYERS, { 'POIs (GPX)': poiLayer }, { position: 'topright' }).addTo(map);

// ============================================================================
// 6. Route input (draw on map + GPX upload)
// ============================================================================
const OSRM_URL = 'https://router.project-osrm.org/route/v1/cycling/';

let points = null;            // [{lat, lon, ele, t}] — the active route
let cumDist = null;           // cumulative distance (m) along points
let totalDist = 0;
let mapNeedsFit = true;

// Draw-mode state
let drawing = false;
let waypoints = [];           // [{marker, pathFromPrev:[[lat,lon]…]}]

function parseGPX(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  if (doc.querySelector('parsererror')) throw new Error('XML inválido');
  let pts = Array.from(doc.getElementsByTagName('trkpt'));
  if (pts.length === 0) pts = Array.from(doc.getElementsByTagName('rtept'));
  if (pts.length === 0) throw new Error('Nenhum <trkpt> ou <rtept> encontrado');
  return pts.map((node) => {
    const eleNode = node.getElementsByTagName('ele')[0];
    const timeNode = node.getElementsByTagName('time')[0];
    return {
      lat: parseFloat(node.getAttribute('lat')),
      lon: parseFloat(node.getAttribute('lon')),
      ele: eleNode ? parseFloat(eleNode.textContent) : null,
      t:   timeNode ? Date.parse(timeNode.textContent) : 0,
    };
  });
}

// Standalone <wpt> POIs (not track points). Returns [{lat,lon,name,desc,type,sym}].
function parsePois(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  if (doc.querySelector('parsererror')) return [];
  const txt = (n, tag) => { const e = n.getElementsByTagName(tag)[0]; return e ? e.textContent.trim() : ''; };
  return Array.from(doc.getElementsByTagName('wpt')).map((n) => ({
    lat: parseFloat(n.getAttribute('lat')), lon: parseFloat(n.getAttribute('lon')),
    name: txt(n, 'name'), desc: txt(n, 'desc') || txt(n, 'cmt'),
    type: txt(n, 'type'), sym: txt(n, 'sym'),
  })).filter((p) => Number.isFinite(p.lat) && Number.isFinite(p.lon));
}

// Map a GPX waypoint type/sym to a standard symbol glyph.
const POI_GLYPHS = {
  danger: '⚠', caution: '⚠', food: '🍴', restaurant: '🍴', water: '💧',
  overlook: '🔭', viewpoint: '🔭', summit: '⛰', campsite: '⛺', camp: '⛺',
  shelter: '🏠', lodging: '🏠', shower: '🚿', rest_area: '☕', checkpoint: '🚩',
  parking: '🅿', gas: '⛽', fuel: '⛽', generic: '📍', dot: '📍',
};
function poiGlyph(p) {
  const k = (p.type || p.sym || '').toLowerCase();
  return POI_GLYPHS[k] || '📍';
}

function renderPois(pois) {
  poiLayer.clearLayers();
  pois.forEach((p) => {
    const icon = L.divIcon({ className: 'poi-icon', html: poiGlyph(p), iconSize: [24, 24], iconAnchor: [12, 12] });
    const m = L.marker([p.lat, p.lon], { icon });
    const title = p.name || '(sem nome)';
    m.bindTooltip(title, { direction: 'top' });
    const body = `<strong>${escapeHtml(title)}</strong>`
      + (p.desc ? `<br>${escapeHtml(p.desc)}` : '')
      + (p.type ? `<br><em>${escapeHtml(p.type)}</em>` : '');
    m.bindPopup(body);
    poiLayer.addLayer(m);
  });
}

async function osrmRoute(a, b) {
  const url = `${OSRM_URL}${a.lng},${a.lat};${b.lng},${b.lat}?overview=full&geometries=geojson`;
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  const data = await res.json();
  if (data.code !== 'Ok' || !data.routes?.[0]) throw new Error(data.code || 'sem rota');
  return data.routes[0].geometry.coordinates.map(([lng, lat]) => [lat, lng]);
}

// Straight line densified to ~one point per 80 m.
function densifyStraight(a, b) {
  const d = a.distanceTo(b);
  const n = Math.max(1, Math.round(d / 80));
  const out = [];
  for (let i = 0; i <= n; i++) {
    const f = i / n;
    out.push([a.lat + (b.lat - a.lat) * f, a.lng + (b.lng - a.lng) * f]);
  }
  return out;
}

async function routeBetween(a, b) {
  const snap = document.getElementById('snap-roads').checked;
  if (snap) {
    try { return await osrmRoute(a, b); }
    catch (e) { console.warn('OSRM falhou, usando reta:', e.message); }
  }
  return densifyStraight(a, b);
}

// Build the dense points array from drawn waypoints + their connecting paths.
function pointsFromWaypoints() {
  const out = [];
  for (let i = 0; i < waypoints.length; i++) {
    const wp = waypoints[i].marker.getLatLng();
    if (i === 0) { out.push([wp.lat, wp.lng]); continue; }
    const path = waypoints[i].pathFromPrev;
    if (path && path.length >= 2) {
      for (let j = 1; j < path.length; j++) out.push([path[j][0], path[j][1]]);
    } else {
      out.push([wp.lat, wp.lng]);
    }
  }
  return out.map(([lat, lon]) => ({ lat, lon, ele: null, t: 0 }));
}

let drawMarkers = L.layerGroup().addTo(map);

async function rebuildFromWaypoints() {
  setDrawStatus('roteando…');
  for (let i = 1; i < waypoints.length; i++) {
    if (!waypoints[i].pathFromPrev) {
      const a = waypoints[i - 1].marker.getLatLng();
      const b = waypoints[i].marker.getLatLng();
      waypoints[i].pathFromPrev = await routeBetween(a, b);
    }
  }
  points = pointsFromWaypoints();
  if (points.length >= 1) await fillElevations(points);
  onRouteChanged(false);
  setDrawStatus(`${waypoints.length} ponto(s). Clique para adicionar, arraste para mover.`);
}

function startDrawing() {
  drawing = true;
  document.getElementById('draw-toolbar').classList.remove('hidden');
  document.getElementById('map').classList.add('drawing');
  map.invalidateSize();
  setDrawStatus('Modo desenho: clique no mapa para adicionar pontos.');
}
function stopDrawing() {
  drawing = false;
  document.getElementById('draw-toolbar').classList.add('hidden');
  document.getElementById('map').classList.remove('drawing');
}
function setDrawStatus(msg) { document.getElementById('draw-status').textContent = msg; }

map.on('click', (e) => {
  if (!drawing) return;
  addDraggableWaypoint(e.latlng);
});

// Draggable waypoint via L.marker + divIcon (circleMarker can't drag).
function addDraggableWaypoint(latlng) {
  const icon = L.divIcon({ className: 'wp-icon', html: '', iconSize: [14, 14] });
  const marker = L.marker(latlng, { icon, draggable: true });
  marker.addTo(drawMarkers);
  const wp = { marker, pathFromPrev: null };
  const idx = waypoints.length;
  waypoints.push(wp);
  marker.on('dragend', () => {
    // invalidate paths touching this waypoint
    if (waypoints[idx]) waypoints[idx].pathFromPrev = null;
    if (waypoints[idx + 1]) waypoints[idx + 1].pathFromPrev = null;
    rebuildFromWaypoints();
  });
  rebuildFromWaypoints();
}

document.getElementById('draw-btn').addEventListener('click', (e) => {
  e.stopPropagation();
  if (drawing) return;
  waypoints.forEach((w) => drawMarkers.removeLayer(w.marker));
  waypoints = [];
  startDrawing();
});
document.getElementById('draw-undo').addEventListener('click', () => {
  const wp = waypoints.pop();
  if (wp) drawMarkers.removeLayer(wp.marker);
  rebuildFromWaypoints();
});
document.getElementById('draw-clear').addEventListener('click', () => {
  waypoints.forEach((w) => drawMarkers.removeLayer(w.marker));
  waypoints = [];
  points = null;
  rebuildFromWaypoints();
});
document.getElementById('draw-done').addEventListener('click', () => {
  stopDrawing();
  if (points && points.length >= 2) { mapNeedsFit = true; recompute(); fitToRoute(); }
});
document.getElementById('snap-roads').addEventListener('change', () => {
  waypoints.forEach((w) => { w.pathFromPrev = null; });
  rebuildFromWaypoints();
});

// ── GPX upload ──
function loadGPXText(text, name) {
  try {
    const pts = parseGPX(text);
    const hasEle = pts.some((p) => Number.isFinite(p.ele) && p.ele !== 0);
    points = pts.map((p) => ({ ...p, ele: Number.isFinite(p.ele) ? p.ele : null }));
    const pois = parsePois(text);
    renderPois(pois);
    const poiNote = pois.length ? ` · ${pois.length} POIs` : '';
    document.getElementById('dz-title').textContent = `${name} · ${pts.length} pontos${poiNote}`;
    if (!hasEle) {
      setDrawStatus('');
      fillElevations(points).then(() => { mapNeedsFit = true; onRouteChanged(true); fitToRoute(); });
    } else {
      points.forEach((p) => { if (p.ele == null) p.ele = 0; });
      mapNeedsFit = true; onRouteChanged(true); fitToRoute();
    }
  } catch (e) {
    alert('Não consegui ler o GPX: ' + e.message);
  }
}

const dz = document.getElementById('dropzone');
const fileInput = document.getElementById('file-input');
document.getElementById('browse-btn').addEventListener('click', (e) => { e.stopPropagation(); fileInput.click(); });
dz.addEventListener('click', (e) => { if (e.target === dz || e.target.classList.contains('dropzone-title')) fileInput.click(); });
dz.addEventListener('dragover',  (e) => { e.preventDefault(); dz.classList.add('drag'); });
dz.addEventListener('dragleave', () => dz.classList.remove('drag'));
dz.addEventListener('drop', (e) => {
  e.preventDefault(); dz.classList.remove('drag');
  const f = e.dataTransfer.files[0]; if (f) readFile(f);
});
fileInput.addEventListener('change', () => { if (fileInput.files[0]) readFile(fileInput.files[0]); });
function readFile(f) {
  const reader = new FileReader();
  reader.onload = () => { if (drawing) stopDrawing(); loadGPXText(reader.result, f.name); };
  reader.readAsText(f);
}
document.getElementById('sample-btn').addEventListener('click', (e) => {
  e.stopPropagation();
  if (drawing) stopDrawing();
  loadGPXText(SAMPLE_GPX, 'sp-jaragua.gpx');
});

// ============================================================================
// 7. Clipping range slider
// ============================================================================
function rebuildCumDist() {
  if (!points || points.length < 2) { cumDist = null; totalDist = 0; return; }
  const cum = [0];
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1], b = points[i];
    const horiz = haversine(a.lat, a.lon, b.lat, b.lon);
    const dEle = (b.ele - a.ele) || 0;
    cum.push(cum[i - 1] + Math.sqrt(horiz * horiz + dEle * dEle));
  }
  cumDist = cum;
  totalDist = cum[cum.length - 1];
}

function readRangeFractions() {
  const a = parseFloat(document.getElementById('range-start').value) / 1000;
  const b = parseFloat(document.getElementById('range-end').value)   / 1000;
  return { lo: Math.min(a, b), hi: Math.max(a, b) };
}
function clipBounds() {
  const { lo, hi } = readRangeFractions();
  return { dLo: totalDist * lo, dHi: totalDist * hi };
}
// Indices of points whose cumDist lies within [dStart, dEnd].
function sliceByDistance(dStart, dEnd) {
  if (!cumDist) return null;
  let iStart = 0, iEnd = points.length - 1;
  for (let i = 0; i < cumDist.length; i++) { if (cumDist[i] >= dStart) { iStart = i; break; } }
  for (let i = cumDist.length - 1; i >= 0; i--) { if (cumDist[i] <= dEnd) { iEnd = i; break; } }
  if (iEnd <= iStart) iEnd = Math.min(iStart + 1, points.length - 1);
  return points.slice(iStart, iEnd + 1);
}

function updateRangeUI() {
  const { lo, hi } = readRangeFractions();
  const fill = document.getElementById('range-fill');
  fill.style.left  = (lo * 100) + '%';
  fill.style.right = ((1 - hi) * 100) + '%';
  const totalKm = totalDist / 1000;
  document.getElementById('range-label-start').textContent = (totalKm * lo).toFixed(1) + ' km';
  document.getElementById('range-label-end').textContent   = (totalKm * hi).toFixed(1) + ' km';
  document.getElementById('range-label-total').textContent = `· de ${totalKm.toFixed(1)} km`;
}

let rafToken = null;
function scheduleRecompute() {
  if (rafToken) return;
  rafToken = requestAnimationFrame(() => { rafToken = null; updateRangeUI(); recompute(); });
}
document.getElementById('range-start').addEventListener('input', scheduleRecompute);
document.getElementById('range-end').addEventListener('input', scheduleRecompute);
document.getElementById('range-reset').addEventListener('click', () => {
  document.getElementById('range-start').value = 0;
  document.getElementById('range-end').value = 1000;
  mapNeedsFit = true; updateRangeUI(); recompute();
});

// ============================================================================
// 8. Segments (split points + per-segment overrides)
// ============================================================================
// Base profile fields (the global sliders).
const FIELDS = [
  { key: 'riderKg',     label: 'Massa ciclista',  short: 'massa',  unit: 'kg',    min: 40,    max: 130, step: 1,     val: 75   },
  { key: 'bikeKg',      label: 'Bike + bagagem',  short: 'bike',   unit: 'kg',    min: 6,     max: 25,  step: 0.1,   val: 10   },
  { key: 'cda',         label: 'Área CdA',        short: 'CdA',    unit: 'm²',    min: 0.20,  max: 0.50, step: 0.01, val: 0.32 },
  { key: 'crr',         label: 'Rolamento Crr',   short: 'Crr',    unit: '',      min: 0.003, max: 0.012, step: 0.0005, val: 0.005 },
  { key: 'eff',         label: 'Eficiência trans.', short: 'η',    unit: '',      min: 0.92,  max: 0.99, step: 0.01, val: 0.97 },
  { key: 'rho',         label: 'Densidade do ar', short: 'ρ',     unit: 'kg/m³', min: 1.00,  max: 1.30, step: 0.01, val: 1.16 },
  { key: 'headwindKmh', label: 'Vento contra',    short: 'vento', unit: 'km/h',  min: -25,   max: 25,  step: 1,     val: 0    },
  { key: 'powerClimbW', label: 'Potência · subida', short: 'P↑',  unit: 'W',     min: 50,    max: 400, step: 5,     val: 200  },
  { key: 'powerFlatW',  label: 'Potência · plano',  short: 'P=',  unit: 'W',     min: 30,    max: 300, step: 5,     val: 100  },
  { key: 'powerDescW',  label: 'Potência · descida',short: 'P↓',  unit: 'W',     min: 0,     max: 100, step: 1,     val: 10   },
  { key: 'gradeThPct',  label: 'Limiar plano ±',  short: 'grau±', unit: '%',     min: 0.5,   max: 3,   step: 0.1,   val: 1.0  },
  { key: 'maxDescentKmh', label: 'Limite freio',  short: 'freio', unit: 'km/h',  min: 25,    max: 80,  step: 1,     val: 40   },
];
const profile = {};
FIELDS.forEach((f) => { profile[f.key] = f.val; });

// Convert a UI profile (km/h units) to the physics profile analyze() expects.
function physicsFromUi(ui) {
  return {
    riderKg: ui.riderKg, bikeKg: ui.bikeKg, cda: ui.cda, crr: ui.crr,
    eff: ui.eff, rho: ui.rho, headwindMs: ui.headwindKmh / 3.6,
    powerClimbW: ui.powerClimbW, powerFlatW: ui.powerFlatW, powerDescW: ui.powerDescW,
    gradeThPct: ui.gradeThPct, maxDescentMs: ui.maxDescentKmh / 3.6,
  };
}

function formatVal(f, v) {
  let s;
  if      (f.step >= 1)    s = v.toFixed(0);
  else if (f.step >= 0.1)  s = v.toFixed(1);
  else if (f.step >= 0.01) s = v.toFixed(2);
  else                     s = v.toFixed(4);
  return s + (f.unit ? ' ' + f.unit : '');
}

// (No global profile sidebar — parameters are edited per segment in the table.
//  `profile` holds the inherited defaults shown as placeholders.)

// Splits: sorted distances (m) along the full route. Segments: stable objects.
let splits = [];                       // [dist, …] sorted
let segments = [{ overrides: {} }];    // length === splits.length + 1

function boundaries() { return [0, ...splits, totalDist]; }

function addSplit(dist) {
  if (!Number.isFinite(dist) || dist <= 0 || dist >= totalDist) return;
  // find segment containing dist
  const B = boundaries();
  let segIdx = -1;
  for (let i = 0; i < B.length - 1; i++) {
    if (dist > B[i] && dist < B[i + 1]) { segIdx = i; break; }
  }
  if (segIdx < 0) return;
  // avoid duplicate (too close to an existing split)
  if (splits.some((s) => Math.abs(s - dist) < totalDist * 0.005)) return;
  splits.push(dist); splits.sort((a, b) => a - b);
  segments.splice(segIdx + 1, 0, { overrides: { ...segments[segIdx].overrides } });
  recompute(); saveState();
}

function removeSplit(splitIdx) {
  splits.splice(splitIdx, 1);
  segments.splice(splitIdx + 1, 1);   // merge into the left segment
  recompute(); saveState();
}

// Delete a whole segment, merging its span into a neighbour (keeps invariant
// segments.length === splits.length + 1).
function removeSegment(i) {
  if (segments.length <= 1) { segments[0].overrides = {}; recompute(); saveState(); return; }
  if (i === 0) { splits.splice(0, 1); segments.splice(0, 1); }       // merge into next
  else { splits.splice(i - 1, 1); segments.splice(i, 1); }           // merge into previous
  recompute(); saveState();
}

function clearSplits() { splits = []; segments = [{ overrides: {} }]; recompute(); saveState(); }
document.getElementById('seg-clear').addEventListener('click', clearSplits);

// Compute every segment (clipped) and the grand total.
function computeSegments() {
  const { dLo, dHi } = clipBounds();
  const B = boundaries();
  const rows = [];
  let allSamples = [{ d: 0, t: 0, ele: points ? (points[0].ele || 0) : 0, p: 0,
                      lat: points ? points[0].lat : 0, lon: points ? points[0].lon : 0 }];
  const total = { energyJ: 0, distanceKm: 0, ascentM: 0, descentM: 0, durationS: 0,
                  eRoll: 0, eAero: 0, eGrav: 0 };
  let tOffset = 0;   // running moving-time offset across segments
  for (let i = 0; i < B.length - 1; i++) {
    const segStart = Math.max(B[i], dLo);
    const segEnd   = Math.min(B[i + 1], dHi);
    if (segEnd <= segStart + 1) continue;   // outside clip / negligible
    const slice = sliceByDistance(segStart, segEnd);
    if (!slice || slice.length < 2) continue;
    const ui = { ...profile, ...segments[i].overrides };
    const res = analyze(slice, physicsFromUi(ui));
    total.energyJ += res.energyJ; total.distanceKm += res.distanceKm;
    total.ascentM += res.ascentM; total.descentM += res.descentM;
    total.durationS += res.durationS;
    total.eRoll += res.eRoll; total.eAero += res.eAero; total.eGrav += res.eGrav;
    // splice the segment samples onto the global chart, offset by distance + time
    res.samples.forEach((s, j) => {
      if (j === 0) return;   // skip duplicate boundary sample
      allSamples.push({ ...s, d: segStart + s.d, t: tOffset + s.t });
    });
    tOffset += res.durationS;
    const lbl = (segments[i].overrides.__label || '').trim();
    rows.push({ segIdx: i, startKm: segStart / 1000, endKm: segEnd / 1000,
                startM: segStart, endM: segEnd, energyKJ: res.energyKJ, durationS: res.durationS,
                num: rows.length + 1, label: lbl || `trecho ${rows.length + 1}` });
  }
  total.energyKJ = total.energyJ / 1000;
  total.kcal = (total.energyKJ / 0.24) / 4.184;
  total.avgPowerW = total.durationS > 0 ? total.energyJ / total.durationS : 0;
  return { rows, total, allSamples };
}

// ── Segment table ──
function renderSegTable(rows) {
  const table = document.getElementById('seg-table');
  if (!rows.length) { table.innerHTML = '<tbody><tr><td>—</td></tr></tbody>'; return; }
  const head = '<thead><tr>'
    + '<th class="lbl">trecho</th><th>km</th><th>tempo</th>'
    + FIELDS.map((f) => `<th title="${f.label}">${f.short}</th>`).join('')
    + '<th>kJ</th><th></th></tr></thead>';
  const body = rows.map((row, rIdx) => {
    const seg = segments[row.segIdx];
    const ov = seg.overrides;
    const cells = FIELDS.map((f) => {
      const v = ov[f.key];
      return `<td><input data-seg="${row.segIdx}" data-key="${f.key}" type="number"
                step="${f.step}" placeholder="${profile[f.key]}"
                value="${v == null ? '' : v}"></td>`;
    }).join('');
    return `<tr>
      <td class="lbl"><input class="lbl-input" data-seg="${row.segIdx}" data-key="__label"
            placeholder="trecho ${rIdx + 1}" value="${ov.__label || ''}"></td>
      <td class="seg-range">${row.startKm.toFixed(1)}–${row.endKm.toFixed(1)}</td>
      <td class="seg-range">${fmtDuration(row.durationS)}</td>
      ${cells}
      <td class="seg-energy">${row.energyKJ.toFixed(0)}</td>
      <td><button class="seg-del" data-seg="${row.segIdx}" title="excluir trecho">✕</button></td>
    </tr>`;
  }).join('');
  table.innerHTML = head + '<tbody>' + body + '</tbody>';
  table.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('change', () => {
      const si = parseInt(inp.dataset.seg, 10);
      const key = inp.dataset.key;
      if (!segments[si]) return;
      if (key === '__label') {
        if (inp.value.trim()) segments[si].overrides.__label = inp.value.trim();
        else delete segments[si].overrides.__label;
      } else if (inp.value === '') {
        delete segments[si].overrides[key];
      } else {
        segments[si].overrides[key] = parseFloat(inp.value);
      }
      recompute(); saveState();
    });
  });
  table.querySelectorAll('.seg-del').forEach((btn) => {
    btn.addEventListener('click', () => removeSegment(parseInt(btn.dataset.seg, 10)));
  });
}

// ============================================================================
// 9. Render + hover + persistence
// ============================================================================
let routeLayer = null, splitLayer = null, segLabelLayer = null, hoverMarker = null;
let currentSamples = null, currentDMax = 0, currentRows = [];
let chartMode = 'split';        // 'split' (click adds divider) | 'zoom' (drag to zoom)
let chartView = null;           // { lo, hi } distance window into the profile
let chartD0 = 0, chartD1 = 0;   // domain currently drawn (set by drawChart)

function fitToRoute() {
  if (!points || points.length < 2) return;
  const line = L.polyline(points.map((p) => [p.lat, p.lon]));
  map.invalidateSize();
  map.fitBounds(line.getBounds(), { padding: [28, 28] });
  mapNeedsFit = false;
}

function onRouteChanged(resetSplits) {
  rebuildCumDist();
  chartView = null;
  if (resetSplits) { splits = []; segments = [{ overrides: {} }]; }
  document.getElementById('range-start').value = 0;
  document.getElementById('range-end').value = 1000;
  document.getElementById('empty-state').classList.add('hidden');
  document.getElementById('results-area').classList.remove('hidden');
  updateRangeUI();
  recompute();
  saveState();
}

function recompute() {
  if (!points || points.length < 2) return;
  rebuildCumDist();
  const { dLo, dHi } = clipBounds();
  const { rows, total, allSamples } = computeSegments();
  currentSamples = allSamples;
  currentRows = rows;
  currentDMax = allSamples.length ? allSamples[allSamples.length - 1].d : 0;

  document.getElementById('r-energy').textContent = total.energyKJ.toFixed(0);
  document.getElementById('r-energy-sub').textContent =
    `${total.distanceKm.toFixed(1)} km · ${total.ascentM.toFixed(0)} m subida · ${fmtDuration(total.durationS)}`;
  document.getElementById('r-kcal').textContent = total.kcal.toFixed(0);
  document.getElementById('r-power').textContent = total.avgPowerW.toFixed(0);
  document.getElementById('r-power-sub').textContent = (total.avgPowerW / profile.riderKg).toFixed(2) + ' W/kg';

  drawMap(dLo, dHi, rows);
  drawChart(allSamples, rows);
  drawBreakdown(total);
  renderSegTable(rows);
}

function fmtDuration(s) {
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h > 0 ? `${h}h ${m}m` : `${m}m`;
}

function latlonAtDistance(d) {
  if (!cumDist) return null;
  let lo = 0, hi = cumDist.length - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (cumDist[mid] < d) lo = mid + 1; else hi = mid; }
  return points[lo];
}

// Brutalist square map markers (béton-brut: right angles, not dots).
function sqIcon(cls, size) {
  size = size || 12;
  return L.divIcon({ className: 'mk ' + cls, iconSize: [size, size], iconAnchor: [size / 2, size / 2] });
}

function drawMap(dLo, dHi, rows) {
  if (routeLayer) { map.removeLayer(routeLayer); routeLayer = null; }
  if (splitLayer) { map.removeLayer(splitLayer); splitLayer = null; }
  if (segLabelLayer) { map.removeLayer(segLabelLayer); segLabelLayer = null; }
  const full = points.map((p) => [p.lat, p.lon]);
  const clip = sliceByDistance(dLo, dHi).map((p) => [p.lat, p.lon]);
  const isSubset = clip.length < full.length;
  const layers = [];
  if (isSubset) {
    layers.push(L.polyline(full, { color: '#16140f', weight: 2, opacity: 0.30, interactive: false }));
  }
  const bright = L.polyline(clip, { color: '#ad3815', weight: 4, opacity: 0.95, lineCap: 'butt', lineJoin: 'miter' });
  bright.on('click', (e) => {
    L.DomEvent.stop(e);
    const near = nearestCumDistTo(e.latlng);
    if (near != null) addSplit(near);
  });
  layers.push(bright);
  layers.push(L.marker(clip[0], { icon: sqIcon('mk-ink') }).bindTooltip('início', { direction: 'top' }));
  layers.push(L.marker(clip[clip.length - 1], { icon: sqIcon('mk-oxide') }).bindTooltip('fim', { direction: 'top' }));
  routeLayer = L.layerGroup(layers).addTo(map);

  // split markers
  const sl = [];
  splits.forEach((d, k) => {
    if (d <= dLo || d >= dHi) return;
    const ll = latlonAtDistance(d);
    if (!ll) return;
    const m = L.marker([ll.lat, ll.lon], { icon: sqIcon('mk-hollow', 13) })
      .bindTooltip(`divisão · ${(d / 1000).toFixed(1)} km (clique p/ remover)`, { direction: 'top' });
    m.on('click', (e) => { L.DomEvent.stop(e); removeSplit(k); });
    sl.push(m);
  });
  splitLayer = L.layerGroup(sl).addTo(map);

  // segment labels at each segment midpoint (only when there's more than one)
  const ll2 = [];
  if (rows && rows.length > 1) {
    rows.forEach((row) => {
      const ll = latlonAtDistance((row.startM + row.endM) / 2);
      if (!ll) return;
      const icon = L.divIcon({
        className: 'seg-label',
        html: escapeHtml(row.label),
        iconSize: null,   // size to content
      });
      ll2.push(L.marker([ll.lat, ll.lon], { icon, interactive: false, keyboard: false }));
    });
  }
  segLabelLayer = L.layerGroup(ll2).addTo(map);

  map.invalidateSize();
  if (mapNeedsFit) fitToRoute();
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

function nearestCumDistTo(latlng) {
  if (!points) return null;
  let best = null, bestSq = Infinity;
  for (let i = 0; i < points.length; i++) {
    const dLat = points[i].lat - latlng.lat, dLon = points[i].lon - latlng.lng;
    const sq = dLat * dLat + dLon * dLon;
    if (sq < bestSq) { bestSq = sq; best = i; }
  }
  return best != null ? cumDist[best] : null;
}

// Distance covered at moving-time `t` (linear interp between samples).
function distAtTime(samples, t) {
  if (t <= samples[0].t) return samples[0].d;
  for (let j = 1; j < samples.length; j++) {
    if (samples[j].t >= t) {
      const a = samples[j - 1], c = samples[j];
      const f = (t - a.t) / ((c.t - a.t) || 1);
      return a.d + (c.d - a.d) * f;
    }
  }
  return samples[samples.length - 1].d;
}

// Resample the power series into ≤`nBins` points evenly spaced in MOVING TIME;
// each point is the AVERAGE power over its time slice (energy in slice ÷ slice
// duration). Plotted at the distance reached at the slice midpoint. Power is
// piecewise-constant per source interval, so a time-weighted mean is exact.
function resamplePowerByTime(samples, nBins) {
  const n = samples.length;
  if (n < 2) return samples.map((s) => ({ d: s.d, p: s.p }));
  const t0 = samples[0].t, t1 = samples[n - 1].t;
  const totalT = t1 - t0;
  if (totalT <= 0) return samples.map((s) => ({ d: s.d, p: s.p }));
  const bins = Math.max(2, Math.min(nBins, Math.round(totalT)));   // ≤1 pt/s, ≤nBins
  const binDur = totalT / bins;
  const out = [];
  let j = 1;   // current source interval [j-1, j]
  for (let b = 0; b < bins; b++) {
    const tLo = t0 + b * binDur, tHi = tLo + binDur;
    let e = 0;
    let k = j;
    // advance k back if needed (cheap; intervals are monotonic)
    while (k > 1 && samples[k - 1].t > tLo) k--;
    for (; k < n; k++) {
      const a = samples[k - 1], c = samples[k];
      if (a.t >= tHi) break;
      const lo = Math.max(tLo, a.t), hi = Math.min(tHi, c.t);
      if (hi > lo) e += c.p * (hi - lo);
    }
    j = Math.max(1, k - 1);
    out.push({ d: distAtTime(samples, tLo + binDur / 2), p: e / binDur });
  }
  return out;
}

function drawChart(samples, rows) {
  const svg = document.getElementById('chart');
  if (!samples || samples.length < 2) { svg.innerHTML = ''; chartD0 = chartD1 = 0; return; }
  const W = 800, H = 220, padT = 24, padB = 22, padL = 6, padR = 6;
  const fullLo = samples[0].d, fullHi = samples[samples.length - 1].d;
  // Optional zoom window into the profile (independent of the energy clip).
  let d0 = fullLo, d1 = fullHi;
  if (chartView) {
    d0 = Math.max(fullLo, chartView.lo);
    d1 = Math.min(fullHi, chartView.hi);
    if (d1 - d0 < (fullHi - fullLo) * 0.005) { chartView = null; d0 = fullLo; d1 = fullHi; }
  }
  chartD0 = d0; chartD1 = d1;
  document.getElementById('chart-zoom-reset').classList.toggle('hidden', !chartView);
  const span = Math.max(d1 - d0, 1);
  // Scale vertically to the visible window only.
  const vis = samples.filter((s) => s.d >= d0 - 1 && s.d <= d1 + 1);
  const scope = vis.length >= 2 ? vis : samples;
  const eleVals = scope.map((s) => s.ele);
  const eleMin = Math.min(...eleVals), eleMax = Math.max(...eleVals);
  const eleRange = Math.max(eleMax - eleMin, 1);
  // Power: ≤200 points, each the AVERAGE power over its (total time / 200) slice.
  const powerPts = resamplePowerByTime(scope, 200);
  const pMax = Math.max(...powerPts.map((q) => q.p), 1);

  const xOf = (d) => padL + ((d - d0) / span) * (W - padL - padR);
  const yE  = (e) => padT + (1 - (e - eleMin) / eleRange) * (H - padT - padB);
  const yP  = (p) => padT + (1 - p / pMax) * (H - padT - padB);

  let elePath = `M ${xOf(d0)} ${H - padB}`;
  scope.forEach((s) => { elePath += ` L ${xOf(s.d).toFixed(2)} ${yE(s.ele).toFixed(2)}`; });
  elePath += ` L ${xOf(d1)} ${H - padB} Z`;
  let powPath = '';
  powerPts.forEach((q, i) => { powPath += (i === 0 ? 'M ' : 'L ') + xOf(q.d).toFixed(2) + ' ' + yP(q.p).toFixed(2) + ' '; });

  // split ticks within the visible window
  const ticks = splits.filter((d) => d > d0 && d < d1).map((d) =>
    `<line x1="${xOf(d).toFixed(2)}" x2="${xOf(d).toFixed(2)}" y1="${padT}" y2="${H - padB}"
       stroke="#ad3815" stroke-width="1.5" stroke-dasharray="4 3" vector-effect="non-scaling-stroke"/>`).join('');

  // Segment labels in the clear top band (above the plot), centred per segment.
  let segLabels = '';
  if (rows && rows.length > 1) {
    segLabels = rows.map((row) => {
      const cx = xOf((row.startM + row.endM) / 2);
      const txt = row.label.length > 16 ? row.label.slice(0, 15) + '…' : row.label;
      return `<text x="${cx.toFixed(1)}" y="11" text-anchor="middle" font-family="JetBrains Mono" font-size="9" font-weight="700" fill="#16140f">${escapeHtml(txt)}</text>`;
    }).join('');
  }

  svg.innerHTML = `
    <defs><linearGradient id="eleGrad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="rgba(22,20,15,0.32)"/>
      <stop offset="100%" stop-color="rgba(22,20,15,0.04)"/>
    </linearGradient></defs>
    <path d="${elePath}" fill="url(#eleGrad)" stroke="rgba(22,20,15,0.75)" stroke-width="1" vector-effect="non-scaling-stroke"/>
    <path d="${powPath}" fill="none" stroke="#ad3815" stroke-width="2" stroke-opacity="0.45" stroke-linejoin="miter" vector-effect="non-scaling-stroke"/>
    ${ticks}
    ${segLabels}
    <text x="${padL + 2}" y="${H - 6}" font-family="JetBrains Mono" font-size="10" fill="#4c483f">${(d0 / 1000).toFixed(1)} km</text>
    <text x="${W - padR}" y="${H - 6}" text-anchor="end" font-family="JetBrains Mono" font-size="10" fill="#4c483f">${(d1 / 1000).toFixed(1)} km</text>
    <text x="${W / 2}" y="${H - 6}" text-anchor="middle" font-family="JetBrains Mono" font-size="10" fill="#4c483f">↑${eleMax.toFixed(0)} m · ${pMax.toFixed(0)} W pico</text>`;
}

function drawBreakdown(r) {
  const totalE = r.eRoll + r.eAero + r.eGrav;
  const barEl = document.getElementById('breakdown-bar');
  const legEl = document.getElementById('breakdown-legend');
  if (totalE <= 0) {
    barEl.innerHTML = '';
    legEl.innerHTML = '<div class="breakdown-legend-item"><span class="label">Só descida — sem trabalho positivo.</span></div>';
    return;
  }
  const segs = [
    { label: 'Rolamento', val: r.eRoll, color: 'var(--rolling)' },
    { label: 'Aero',      val: r.eAero, color: 'var(--aero)' },
    { label: 'Subida',    val: r.eGrav, color: 'var(--climb)' },
  ];
  barEl.innerHTML = segs.map((s) => {
    const pct = s.val / totalE * 100;
    return `<div class="breakdown-segment" style="flex:${s.val}; background:${s.color}">${pct >= 8 ? pct.toFixed(0) + '%' : ''}</div>`;
  }).join('');
  legEl.innerHTML = segs.map((s) => `
    <div class="breakdown-legend-item">
      <span class="dot" style="background:${s.color}"></span>
      <span class="label">${s.label}</span>
      <span class="value">${(s.val / 1000).toFixed(0)} kJ</span>
      <span class="label">(${(s.val / totalE * 100).toFixed(0)}%)</span>
    </div>`).join('');
}

// ── Chart interaction: hover, click-to-split, drag-to-zoom ──
const chartEl = document.getElementById('chart');
const chartPanel = document.getElementById('chart-panel');
const chartSelect = document.getElementById('chart-select');
let zoomStartD = null;          // active zoom-drag anchor (distance) or null

// Map a clientX to a distance using the domain currently drawn (respects zoom).
function chartDistAt(clientX) {
  const rect = chartEl.getBoundingClientRect();
  if (rect.width <= 0 || chartD1 <= chartD0) return null;
  const frac = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width));
  return chartD0 + frac * (chartD1 - chartD0);
}

function setChartMode(mode) {
  chartMode = mode;
  chartPanel.classList.toggle('mode-zoom', mode === 'zoom');
  document.querySelectorAll('#chart-mode button').forEach((b) =>
    b.classList.toggle('active', b.dataset.mode === mode));
}
document.querySelectorAll('#chart-mode button').forEach((b) =>
  b.addEventListener('click', () => setChartMode(b.dataset.mode)));
document.getElementById('chart-zoom-reset').addEventListener('click', () => {
  chartView = null; drawChart(currentSamples, currentRows);
});

chartPanel.addEventListener('mousemove', (e) => {
  if (!currentSamples || currentDMax <= 0) return;
  if (zoomStartD != null) {       // dragging a zoom selection — show the band
    const wrap = chartSelect.parentElement.getBoundingClientRect();
    const x1 = chartEl.getBoundingClientRect().left
      + ((zoomStartD - chartD0) / (chartD1 - chartD0)) * chartEl.getBoundingClientRect().width;
    const x2 = e.clientX;
    chartSelect.style.display = 'block';
    chartSelect.style.left = (Math.min(x1, x2) - wrap.left) + 'px';
    chartSelect.style.width = Math.abs(x2 - x1) + 'px';
    return;
  }
  const target = chartDistAt(e.clientX);
  if (target == null) return;
  let best = currentSamples[0], bestDiff = Infinity;
  for (const s of currentSamples) { const diff = Math.abs(s.d - target); if (diff < bestDiff) { bestDiff = diff; best = s; } }
  highlightAt(best);
});
chartPanel.addEventListener('mouseleave', clearHighlight);

chartPanel.addEventListener('mousedown', (e) => {
  if (chartMode !== 'zoom' || !currentSamples || currentDMax <= 0) return;
  e.preventDefault();
  zoomStartD = chartDistAt(e.clientX);
});
window.addEventListener('mouseup', (e) => {
  if (zoomStartD == null) return;
  const endD = chartDistAt(e.clientX);
  chartSelect.style.display = 'none';
  const a = zoomStartD; zoomStartD = null;
  if (endD == null) return;
  const lo = Math.min(a, endD), hi = Math.max(a, endD);
  const fullSpan = (currentSamples[currentSamples.length - 1].d - currentSamples[0].d) || 1;
  if (hi - lo > fullSpan * 0.01) { chartView = { lo, hi }; drawChart(currentSamples, currentRows); }
});

chartPanel.addEventListener('click', (e) => {
  if (chartMode !== 'split' || !currentSamples || currentDMax <= 0) return;
  const target = chartDistAt(e.clientX);
  if (target != null) addSplit(target);
});

function highlightAt(sample) {
  if (!sample) return clearHighlight();
  if (!hoverMarker) {
    hoverMarker = L.marker([sample.lat, sample.lon], { icon: sqIcon('mk-hl', 15), interactive: false }).addTo(map);
  } else hoverMarker.setLatLng([sample.lat, sample.lon]);
  document.getElementById('hr-d').textContent = (sample.d / 1000).toFixed(2) + ' km';
  document.getElementById('hr-e').textContent = sample.ele.toFixed(0) + ' m';
  document.getElementById('hr-p').textContent = sample.p.toFixed(0) + ' W';
  document.getElementById('hover-readout').classList.add('show');
}
function clearHighlight() {
  if (hoverMarker) { map.removeLayer(hoverMarker); hoverMarker = null; }
  document.getElementById('hover-readout').classList.remove('show');
}

// ============================================================================
// Layer panel UI (topo controls + legend)
// ============================================================================
function renderLayerPanel() {
  const root = document.getElementById('layer-panel');
  root.innerHTML = `
    <div class="topo-controls" id="topo-controls">
      <div class="topo-grid">
        <div class="num-field"><label>Elevação mín. (m)</label>
          <input type="number" id="topo-min" value="${topoSettings.minElevation}" step="10"></div>
        <div class="num-field"><label>Elevação máx. (m)</label>
          <input type="number" id="topo-max" value="${topoSettings.maxElevation}" step="10"></div>
        <div class="num-field"><label>Declividade máx. (%)</label>
          <input type="number" id="topo-slope" value="${topoSettings.maxSlope}" step="1" min="1"></div>
        <div class="num-field"><label>Amostragem máx. (MB / mapa · 0 = sem limite)</label>
          <input type="number" id="topo-cap" value="${topoSettings.maxSampleMB}" step="1" min="0" max="128"></div>
      </div>
      <div class="topo-legend">
        <div class="ramp" style="background: linear-gradient(to right, ${phaseGradientCss()})"></div>
        <div class="ramp-labels"><span id="topo-lbl-min">${topoSettings.minElevation} m</span><span id="topo-lbl-max">${topoSettings.maxElevation} m</span></div>
      </div>
      <div class="seg-help" style="margin-top:8px">Camada “Topográfica colorida”: fundo = elevação (cmocean.phase), topo = declividade (multiply). Cada visualização do mapa amostra do FABDEM no máximo o limite acima (dividido entre os tiles à vista) — nunca baixa os tiles inteiros.</div>
    </div>`;
  const rerender = () => {
    topoSettings.minElevation = parseFloat(document.getElementById('topo-min').value);
    topoSettings.maxElevation = parseFloat(document.getElementById('topo-max').value);
    topoSettings.maxSlope     = parseFloat(document.getElementById('topo-slope').value);
    topoSettings.maxSampleMB  = Math.max(0, parseFloat(document.getElementById('topo-cap').value) || 0);
    document.getElementById('topo-lbl-min').textContent = topoSettings.minElevation + ' m';
    document.getElementById('topo-lbl-max').textContent = topoSettings.maxElevation + ' m';
    if (map.hasLayer(topo)) topo.redraw();
    saveState();
  };
  ['topo-min', 'topo-max', 'topo-slope', 'topo-cap'].forEach((id) =>
    document.getElementById(id).addEventListener('change', rerender));
}

// ============================================================================
// Persistence (localStorage autosave/restore)
// ============================================================================
const STORE_KEY = 'quilojaules.state.v1';
function saveState() {
  try {
    const state = {
      profile, topoSettings, splits,
      overrides: segments.map((s) => s.overrides),
      route: points ? points.map((p) => [+p.lat.toFixed(6), +p.lon.toFixed(6), p.ele == null ? null : +p.ele.toFixed(1), p.t || 0]) : null,
      clip: [document.getElementById('range-start').value, document.getElementById('range-end').value],
    };
    localStorage.setItem(STORE_KEY, JSON.stringify(state));
  } catch (e) { /* storage may be unavailable */ }
}
function restoreState() {
  let state;
  try { state = JSON.parse(localStorage.getItem(STORE_KEY)); } catch (e) { return false; }
  if (!state) return false;
  if (state.profile) Object.assign(profile, state.profile);
  if (state.topoSettings) Object.assign(topoSettings, state.topoSettings);
  if (Array.isArray(state.route) && state.route.length >= 2) {
    points = state.route.map(([lat, lon, ele, t]) => ({ lat, lon, ele: ele == null ? 0 : ele, t: t || 0 }));
    splits = Array.isArray(state.splits) ? state.splits : [];
    segments = (state.overrides && state.overrides.length) ? state.overrides.map((o) => ({ overrides: o || {} })) : [{ overrides: {} }];
    if (segments.length !== splits.length + 1) { splits = []; segments = [{ overrides: {} }]; }
    rebuildCumDist();
    if (state.clip) {
      document.getElementById('range-start').value = state.clip[0];
      document.getElementById('range-end').value = state.clip[1];
    }
    document.getElementById('empty-state').classList.add('hidden');
    document.getElementById('results-area').classList.remove('hidden');
    document.getElementById('dz-title').textContent = `rota restaurada · ${points.length} pontos`;
    mapNeedsFit = true;
    updateRangeUI();
    return true;
  }
  return false;
}

// ============================================================================
// RDF / Turtle state datagraph (conforms to shapes.ttl). Export = .ttl.gz.
// ============================================================================
const QJ = 'https://pedalhidrografi.co/quilojaules#';
const RDF_TYPE = 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type';
const OVERRIDE_KEYS = FIELDS.map((f) => f.key);   // the 12 physical parameters

let _n3Promise = null;
function loadScript(src) {
  return new Promise((res, rej) => {
    const s = document.createElement('script');
    s.src = src; s.onload = res; s.onerror = () => rej(new Error('falha ao carregar ' + src));
    document.head.appendChild(s);
  });
}
function ensureN3() {
  if (!_n3Promise) _n3Promise = (async () => {
    if (!window.N3) await loadScript('https://cdn.jsdelivr.net/npm/n3@1.17.4/browser/n3.min.js');
    return window.N3;
  })();
  return _n3Promise;
}

// Decimal literal (forces a '.' so it's xsd:decimal, not xsd:integer).
function dec(n) { const v = +n; return Number.isInteger(v) ? v + '.0' : String(v); }
function ttlStr(s) { return '"' + String(s).replace(/\\/g, '\\\\').replace(/"/g, '\\"') + '"'; }

function stateToTurtle() {
  const cs = document.getElementById('range-start').value;
  const ce = document.getElementById('range-end').value;
  const out = [];
  out.push('@prefix qj: <' + QJ + '> .');
  out.push('@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .');
  out.push('');
  out.push('<urn:quilojaules:state> a qj:AppState ;');
  out.push(`  qj:created "${new Date().toISOString()}"^^xsd:dateTime ;`);
  out.push(`  qj:clipStart ${dec(cs)} ;`);
  out.push(`  qj:clipEnd ${dec(ce)} ;`);
  out.push(`  qj:topography [ a qj:TopographyConfig ; qj:minElevation ${dec(topoSettings.minElevation)} ; qj:maxElevation ${dec(topoSettings.maxElevation)} ; qj:maxSlope ${dec(topoSettings.maxSlope)} ; qj:maxSampleMB ${dec(topoSettings.maxSampleMB)} ] ;`);
  out.push(`  qj:baseProfile [ a qj:Profile ; ${OVERRIDE_KEYS.map((k) => `qj:${k} ${dec(profile[k])}`).join(' ; ')} ] ;`);
  splits.forEach((s) => out.push(`  qj:split ${dec(s)} ;`));
  segments.forEach((seg, i) => {
    const ov = seg.overrides;
    const parts = ['a qj:Segment', `qj:index ${i}`];
    if (ov.__label) parts.push(`qj:label ${ttlStr(ov.__label)}`);
    OVERRIDE_KEYS.forEach((k) => { if (ov[k] != null) parts.push(`qj:${k} ${dec(ov[k])}`); });
    out.push(`  qj:segment [ ${parts.join(' ; ')} ] ;`);
  });
  const coords = (points || []).map((p) =>
    `${p.lat},${p.lon},${p.ele == null ? '' : +(+p.ele).toFixed(1)},${p.t || 0}`).join(';');
  out.push(`  qj:route [ a qj:Route ; qj:pointCount ${(points || []).length} ; qj:coordinates """${coords}""" ] .`);
  return out.join('\n') + '\n';
}

async function turtleToState(ttl) {
  const N3 = await ensureN3();
  const { namedNode } = N3.DataFactory;
  const store = new N3.Store();
  await new Promise((res, rej) => {
    new N3.Parser().parse(ttl, (err, quad) => {
      if (err) return rej(err);
      if (quad) store.addQuad(quad); else res();
    });
  });
  const stateNodes = store.getSubjects(namedNode(RDF_TYPE), namedNode(QJ + 'AppState'));
  if (!stateNodes.length) throw new Error('Nenhum qj:AppState no grafo.');
  const st = stateNodes[0];
  const obj1 = (s, p) => { const o = store.getObjects(s, namedNode(QJ + p)); return o.length ? o[0] : null; };
  const num1 = (s, p) => { const o = obj1(s, p); return o ? parseFloat(o.value) : null; };

  // Route (required)
  const routeNode = obj1(st, 'route');
  const coordsObj = routeNode && obj1(routeNode, 'coordinates');
  if (!coordsObj) throw new Error('qj:route/qj:coordinates ausente.');
  const newPoints = coordsObj.value.split(';').filter(Boolean).map((tok) => {
    const [lat, lon, ele, t] = tok.split(',');
    return { lat: +lat, lon: +lon, ele: ele === '' ? 0 : +ele, t: +t || 0 };
  }).filter((p) => Number.isFinite(p.lat) && Number.isFinite(p.lon));
  if (newPoints.length < 2) throw new Error('Rota com menos de 2 pontos.');

  // Topography
  const topo = obj1(st, 'topography');
  if (topo) {
    ['minElevation', 'maxElevation', 'maxSlope', 'maxSampleMB'].forEach((k) => {
      const v = num1(topo, k); if (v != null) topoSettings[k] = v;
    });
  }
  // Base profile
  const bp = obj1(st, 'baseProfile');
  if (bp) OVERRIDE_KEYS.forEach((k) => { const v = num1(bp, k); if (v != null) profile[k] = v; });

  // Splits
  const newSplits = store.getObjects(st, namedNode(QJ + 'split'))
    .map((o) => parseFloat(o.value)).filter(Number.isFinite).sort((a, b) => a - b);

  // Segments (ordered by index)
  const segNodes = store.getObjects(st, namedNode(QJ + 'segment'));
  const segArr = segNodes.map((sn) => {
    const idx = num1(sn, 'index') || 0;
    const overrides = {};
    const lbl = obj1(sn, 'label'); if (lbl) overrides.__label = lbl.value;
    OVERRIDE_KEYS.forEach((k) => { const v = num1(sn, k); if (v != null) overrides[k] = v; });
    return { idx, overrides };
  }).sort((a, b) => a.idx - b.idx).map((s) => ({ overrides: s.overrides }));

  // Commit
  points = newPoints;
  splits = newSplits;
  segments = segArr.length ? segArr : [{ overrides: {} }];
  if (segments.length !== splits.length + 1) { splits = []; segments = [{ overrides: {} }]; }
  rebuildCumDist();
  const cs = num1(st, 'clipStart'), ce = num1(st, 'clipEnd');
  document.getElementById('range-start').value = cs == null ? 0 : cs;
  document.getElementById('range-end').value = ce == null ? 1000 : ce;
  renderLayerPanel();   // reflect restored topography values
  document.getElementById('empty-state').classList.add('hidden');
  document.getElementById('results-area').classList.remove('hidden');
  document.getElementById('dz-title').textContent = `estado importado · ${points.length} pontos`;
  mapNeedsFit = true;
  if (map.hasLayer(topo)) topo.redraw();
  updateRangeUI();
  recompute();
  fitToRoute();
  saveState();
}

// gzip / gunzip via the platform CompressionStream.
async function gzipStr(str) {
  const stream = new Blob([new TextEncoder().encode(str)]).stream().pipeThrough(new CompressionStream('gzip'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}
async function gunzipBuf(buf) {
  const stream = new Blob([buf]).stream().pipeThrough(new DecompressionStream('gzip'));
  return await new Response(stream).text();
}
function downloadBlob(blob, filename) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = filename;
  document.body.appendChild(a); a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 0);
}

async function exportStateTtl() {
  if (!points || points.length < 2) { alert('Carregue ou desenhe uma rota antes de exportar.'); return; }
  try {
    const gz = await gzipStr(stateToTurtle());
    downloadBlob(new Blob([gz], { type: 'application/gzip' }), 'quilojaules-estado.ttl.gz');
  } catch (e) { alert('Falha ao exportar: ' + e.message); }
}
async function importStateTtl(file) {
  try {
    const buf = await file.arrayBuffer();
    const bytes = new Uint8Array(buf);
    const ttl = (bytes[0] === 0x1f && bytes[1] === 0x8b) ? await gunzipBuf(buf) : new TextDecoder().decode(buf);
    if (drawing) stopDrawing();
    poiLayer.clearLayers();
    await turtleToState(ttl);
  } catch (e) { alert('Falha ao importar estado: ' + e.message); }
}

document.getElementById('export-ttl').addEventListener('click', (e) => { e.stopPropagation(); exportStateTtl(); });
const ttlInput = document.getElementById('ttl-input');
document.getElementById('import-ttl').addEventListener('click', (e) => { e.stopPropagation(); ttlInput.click(); });
ttlInput.addEventListener('change', () => { if (ttlInput.files[0]) importStateTtl(ttlInput.files[0]); ttlInput.value = ''; });

// ============================================================================
// Sample GPX (synthetic Pinheiros → Pico do Jaraguá out-and-back)
// ============================================================================
const SAMPLE_GPX = `<?xml version="1.0"?>
<gpx version="1.1" creator="quilojaules" xmlns="http://www.topografix.com/GPX/1/1">
  <trk><name>SP — Pinheiros a Jaraguá</name><trkseg>
    <trkpt lat="-23.5870" lon="-46.6940"><ele>728</ele></trkpt>
    <trkpt lat="-23.5800" lon="-46.7000"><ele>732</ele></trkpt>
    <trkpt lat="-23.5700" lon="-46.7100"><ele>740</ele></trkpt>
    <trkpt lat="-23.5600" lon="-46.7200"><ele>748</ele></trkpt>
    <trkpt lat="-23.5500" lon="-46.7300"><ele>756</ele></trkpt>
    <trkpt lat="-23.5400" lon="-46.7400"><ele>762</ele></trkpt>
    <trkpt lat="-23.5300" lon="-46.7500"><ele>770</ele></trkpt>
    <trkpt lat="-23.5200" lon="-46.7550"><ele>780</ele></trkpt>
    <trkpt lat="-23.5100" lon="-46.7600"><ele>800</ele></trkpt>
    <trkpt lat="-23.5000" lon="-46.7650"><ele>830</ele></trkpt>
    <trkpt lat="-23.4900" lon="-46.7680"><ele>880</ele></trkpt>
    <trkpt lat="-23.4800" lon="-46.7700"><ele>950</ele></trkpt>
    <trkpt lat="-23.4720" lon="-46.7710"><ele>1020</ele></trkpt>
    <trkpt lat="-23.4660" lon="-46.7720"><ele>1100</ele></trkpt>
    <trkpt lat="-23.4620" lon="-46.7700"><ele>1135</ele></trkpt>
    <trkpt lat="-23.4660" lon="-46.7720"><ele>1100</ele></trkpt>
    <trkpt lat="-23.4720" lon="-46.7710"><ele>1020</ele></trkpt>
    <trkpt lat="-23.4800" lon="-46.7700"><ele>950</ele></trkpt>
    <trkpt lat="-23.4900" lon="-46.7680"><ele>880</ele></trkpt>
    <trkpt lat="-23.5000" lon="-46.7650"><ele>830</ele></trkpt>
    <trkpt lat="-23.5100" lon="-46.7600"><ele>800</ele></trkpt>
    <trkpt lat="-23.5200" lon="-46.7550"><ele>780</ele></trkpt>
    <trkpt lat="-23.5300" lon="-46.7500"><ele>770</ele></trkpt>
    <trkpt lat="-23.5400" lon="-46.7400"><ele>762</ele></trkpt>
    <trkpt lat="-23.5500" lon="-46.7300"><ele>756</ele></trkpt>
    <trkpt lat="-23.5600" lon="-46.7200"><ele>748</ele></trkpt>
    <trkpt lat="-23.5700" lon="-46.7100"><ele>740</ele></trkpt>
    <trkpt lat="-23.5800" lon="-46.7000"><ele>732</ele></trkpt>
    <trkpt lat="-23.5870" lon="-46.6940"><ele>728</ele></trkpt>
  </trkseg></trk>
</gpx>`;

// ============================================================================
// Boot
// ============================================================================
renderLayerPanel();
if (restoreState()) { recompute(); }
// Map container is visible from boot; make sure Leaflet has correct dimensions.
window.addEventListener('load', () => map.invalidateSize());
setTimeout(() => map.invalidateSize(), 200);
