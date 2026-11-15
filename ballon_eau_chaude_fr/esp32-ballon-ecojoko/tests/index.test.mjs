// Tests des fonctions de calcul et de rendu de index.html (ballon d'eau chaude).
// Lancer depuis le dossier du projet :  node --test tests/
// Sans navigateur : la page est chargee dans un contexte JS avec un faux DOM.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const pagePath = process.env.PAGE || path.join(here, '..', 'index.html');
const html = readFileSync(pagePath, 'utf8');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]);

// Faux element DOM : accepte n'importe quelle propriete ou methode.
function makeEl() {
  const f = function () { return makeEl(); };
  return new Proxy(f, {
    get(t, p) {
      if (p === Symbol.toPrimitive) return () => '';
      if (p === 'then') return undefined;
      if (p in t) return t[p];
      return makeEl();
    },
    set(t, p, v) { t[p] = v; return true; },
    apply() { return makeEl(); },
  });
}

const ctx = {
  document: {
    hidden: true, // pas de sondage pendant le chargement
    querySelector: () => makeEl(),
    querySelectorAll: () => makeEl(),
    getElementById: () => makeEl(),
    createElement: () => makeEl(),
    addEventListener() {},
    body: makeEl(), documentElement: makeEl(),
  },
  window: null, self: null,
  addEventListener() {},
  matchMedia: () => ({ matches: false, addEventListener() {} }),
  // reseau simule : requete jamais terminee (aucune erreur asynchrone parasite)
  fetch: () => new Promise(() => {}),
  setInterval: () => 0, clearInterval() {}, setTimeout: () => 0, clearTimeout() {},
  requestAnimationFrame: () => 0,
  location: { hash: '', pathname: '/', href: '/' },
  history: { replaceState() {}, pushState() {} },
  alert() {}, confirm: () => false,
  navigator: {}, console, Intl, Date, Math, JSON, Promise, URL,
  Blob: function () {}, Number, String, Array, Object, Error, RegExp,
  AbortController: class { constructor() { this.signal = {}; } abort() {} },
};
ctx.window = ctx; ctx.self = ctx;
vm.createContext(ctx);
for (const s of scripts) vm.runInContext(s, ctx, { filename: 'index.html' });

const call = expr => vm.runInContext(expr, ctx);

// ---------- chargement ----------
test('la page se charge sans erreur', () => {
  assert.equal(call('typeof render'), 'function');
  assert.equal(call('typeof poll'), 'function');
});

test('la version de la page a le bon format', () => {
  assert.match(call('UI_VERSION'), /^\d{4}\.\d{2}\.\d{2}-\d+$/);
});

// ---------- comparaison de versions (7.3) ----------
test('verLt compare les numeros de revision comme des nombres', () => {
  assert.equal(call("verLt('2026.11.04-9','2026.11.04-10')"), true);
  assert.equal(call("verLt('2026.11.04-10','2026.11.04-9')"), false);
  assert.equal(call("verLt('2026.11.04-1','2026.11.04-1')"), false);
  assert.equal(call("verLt('2026.10.30-3','2026.11.04-1')"), true);
  assert.equal(call("verLt('2026.11.05-5','2026.11.04-1')"), false);
});

// ---------- echappement (3.b) ----------
test('sosEsc neutralise le HTML venant de l\'ESP', () => {
  const out = call(`sosEsc('<img src=x onerror=alert(1)>"&')`);
  assert.ok(!out.includes('<'));
  assert.ok(!out.includes('>'));
  assert.equal(out, '&lt;img src=x onerror=alert(1)&gt;&quot;&amp;');
});

// ---------- dates et cycles ----------
// Identifiant de cycle = annee * 1000 + jour de l'annee a partir de 0 (comme tm_yday en C).
test('isoDay convertit un identifiant de cycle en date', () => {
  assert.equal(call('isoDay(2026000)'), '2026-01-01');
  assert.equal(call('isoDay(2026364)'), '2026-12-31');
});

test('dayOff compte les jours entre deux instants', () => {
  const noonUTC = Date.UTC(2026, 10, 11, 12) / 1000;
  assert.equal(call(`dayOff(${noonUTC + 86400}, ${noonUTC})`), 1);
  assert.equal(call(`dayOff(${noonUTC}, ${noonUTC})`), 0);
});

// ---------- heures creuses (fin HC = config.hcEnd, 420 = 07:00) ----------
test('jHM affiche l\'heure de Paris', () => {
  // 11 nov 2026 10:00 a Paris = 09:00 UTC
  assert.equal(call(`jHM(${Date.UTC(2026, 10, 11, 9) / 1000})`), '10:00');
});

test('jPos donne les heures depuis la fin des heures creuses', () => {
  call('config.hcEnd = 420');
  assert.equal(call(`jPos(${Date.UTC(2026, 10, 11, 6) / 1000})`), 0); // 07:00 Paris
  assert.equal(call(`jPos(${Date.UTC(2026, 10, 11, 9) / 1000})`), 3); // 10:00 Paris
});

test('jKey rattache les heures de nuit au jour precedent', () => {
  call('config.hcEnd = 420');
  // 03:00 Paris : encore dans la journee de la veille
  assert.equal(call(`jKey(${Date.UTC(2026, 10, 11, 2) / 1000})`), '2026-11-10');
  // 08:00 Paris : nouvelle journee
  assert.equal(call(`jKey(${Date.UTC(2026, 10, 11, 7) / 1000})`), '2026-11-11');
});

// ---------- prevision solaire ----------
test('solModel renvoie null sans donnees meteo', () => {
  assert.equal(call('solModel({})'), null);
  assert.equal(call('solModel(null)'), null);
});

test('solModel regroupe les heures ensoleillees en fenetres', () => {
  const t0 = Date.UTC(2026, 10, 11, 12) / 1000;
  const r = [100, 300, 300, 100];
  const wx = r.map((v, i) => ({ e: t0 + i * 3600, r: v, t: 10 }));
  const m = call(`solModel(${JSON.stringify({ nowEpoch: t0, wx, antKValid: false })})`);
  assert.equal(m.wins.length, 1);
  assert.equal(m.wins[0].n, 2);
  assert.equal(m.wins[0].s, t0 + 3600);
  assert.equal(m.wins[0].end, t0 + 3 * 3600);
});

// ---------- reglages (export / import) ----------
test('cfgPick garde les valeurs valides et ecarte les mots de passe', () => {
  const out = call(`cfgPick(${JSON.stringify({
    hcStart: 600, heaterW: 99999, wifiPassword: 'secret', ecoPassword: 'secret2', hp: '0.25',
  })}, [])`);
  assert.equal(out.hcStart, 600);
  assert.equal('heaterW' in out, false);
  assert.equal('wifiPassword' in out, false);
  assert.equal('ecoPassword' in out, false);
  assert.equal('hp' in out, false); // mauvais type
});

test('cfgPick signale les valeurs hors limites', () => {
  const bad = [];
  const out = vm.runInContext(`(() => { const b = []; cfgPick({heaterW: 99999}, b); return b; })()`, ctx);
  assert.deepEqual([...out], ['heaterW']);
});

test('cfgPick ignore une entree qui n\'est pas un objet', () => {
  // objets crees dans le contexte de la page : on compare les cles, pas le prototype
  assert.deepEqual(Object.keys(call('cfgPick(null)')), []);
  assert.deepEqual(Object.keys(call('cfgPick("texte")')), []);
});

// ---------- rendu sans reecriture inutile (4.c) ----------
test('setH ne reecrit pas le DOM si le contenu est identique', () => {
  let writes = 0, value = '';
  const el = { set innerHTML(v) { writes++; value = v; }, get innerHTML() { return value; } };
  vm.runInContext('globalThis.__el = null', ctx);
  ctx.__el = el;
  call('setH(__el, "<b>a</b>")');
  call('setH(__el, "<b>a</b>")');
  call('setH(__el, "<b>b</b>")');
  assert.equal(writes, 2);
  assert.equal(value, '<b>b</b>');
});

// ---------- fraicheur des donnees (5.c) ----------
test('freshTick affiche "en direct" juste apres une reponse', () => {
  let text = '', cls = '';
  const fake = { set textContent(v) { text = v; }, get textContent() { return text; }, set className(v) { cls = v; }, get className() { return cls; } };
  const prev = ctx.document.querySelector;
  ctx.document.querySelector = s => (s === '#fresh' ? fake : makeEl());
  call('lastOk = Date.now()');
  call('freshTick()');
  assert.equal(text, '● en direct');
  assert.equal(cls, 'muted');
  ctx.document.querySelector = prev;
});

test('freshTick signale les donnees anciennes', () => {
  let text = '', cls = '';
  const fake = { set textContent(v) { text = v; }, get textContent() { return text; }, set className(v) { cls = v; }, get className() { return cls; } };
  const prev = ctx.document.querySelector;
  ctx.document.querySelector = s => (s === '#fresh' ? fake : makeEl());
  call('lastOk = Date.now() - 30000');
  call('freshTick()');
  assert.match(text, /il y a 30 s/);
  assert.equal(cls, 'bad');
  ctx.document.querySelector = prev;
});
