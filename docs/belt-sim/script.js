/* DOMに依存しないモデル。今後の制御器追加はcommandを拡張する。 */
(function (root) {
  'use strict';
  const defaults = { M: 2, b: 1, r: .5, Fmax: 40, kp: 128, kd: 11.8, kx: 128, kv: 11.8 };
  const limits = { M: [.2, 20], b: [0, 20], r: [0, 1], Fmax: [1, 100], kp: [0, 500], kd: [0, 100], kx: [0, 500], kv: [0, 100] };
  const dt = .001, duration = 5;
  const clamp = (n, a, b) => Math.max(a, Math.min(b, n));
  function validate(p) {
    for (const [key, [min, max]] of Object.entries(limits)) {
      if (!Number.isFinite(p[key]) || p[key] < min || p[key] > max) throw new Error(`${key} は ${min}〜${max} の有限な数値にしてください。`);
    }
  }
  function command(p, mode, x, v) {
    return (mode === 'pd' ? p.kp : p.kx) * (p.r - x) - (mode === 'pd' ? p.kd : p.kv) * v;
  }
  function simulate(p, mode) {
    validate(p);
    if (!['pd', 'sf'].includes(mode)) throw new Error('未知の制御方法です。');
    let x = 0, v = 0, boundaryCount = 0, saturated = false;
    const samples = [];
    const derivative = (xx, vv) => [vv, (clamp(command(p, mode, xx, vv), -p.Fmax, p.Fmax) - p.b * vv) / p.M];
    for (let i = 0; i <= duration / dt; i++) {
      const cmd = command(p, mode, x, v), f = clamp(cmd, -p.Fmax, p.Fmax);
      saturated ||= Math.abs(cmd) > p.Fmax + 1e-9;
      samples.push({ t: i * dt, x, v, f, cmd });
      if (i === duration / dt) break;
      // 閉ループの力はRK4の各中間段階で再評価する。
      const a = derivative(x, v), b = derivative(x + dt*a[0]/2, v + dt*a[1]/2);
      const c = derivative(x + dt*b[0]/2, v + dt*b[1]/2), d = derivative(x + dt*c[0], v + dt*c[1]);
      x += dt/6*(a[0]+2*b[0]+2*c[0]+d[0]); v += dt/6*(a[1]+2*b[1]+2*c[1]+d[1]);
      if (!Number.isFinite(x) || !Number.isFinite(v) || Math.abs(v) > 1e6) throw new Error('計算が発散しました。ゲインや質量を見直してください。');
      // 数値的な端点処理。衝突・反発を表す物理モデルではない。
      if (x < 0 || x > 1) { x = clamp(x, 0, 1); if ((x === 0 && v < 0) || (x === 1 && v > 0)) v = 0; boundaryCount++; }
    }
    return { samples, saturated, boundaryCount };
  }
  function difference(a, b) {
    return a.samples.reduce((m, s, i) => { for (const k of ['x','v','f']) m[k] = Math.max(m[k], Math.abs(s[k]-b.samples[i][k])); return m; }, { x:0,v:0,f:0 });
  }
  const api = { defaults, limits, dt, duration, simulate, difference, validate };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.BeltModel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this);

// ブラウザの操作処理。Node.jsでは計算モデルのみを読み込む。
if (typeof document !== "undefined") {
'use strict';
const $ = id => document.getElementById(id);
const model = window.BeltModel;
let params = { ...model.defaults }, mode = 'pd', results, index = 0, playing = false, lastFrame = null;
const specs = {
  M: ['等価質量 M', 'kg', .1], b: ['粘性抵抗 b', 'N s/m', .1], r: ['目標位置 r', 'm', .01], Fmax: ['駆動力上限', 'N', 1],
  kp: ['位置ゲイン Kp', 'N/m', 1], kd: ['速度ゲイン Kd', 'N s/m', .1], kx: ['位置ゲイン Kx', 'N/m', 1], kv: ['速度ゲイン Kv', 'N s/m', .1]
};
function controls(container, keys) {
  $(container).replaceChildren();
  keys.forEach(key => {
    const [label, unit, step] = specs[key], [min, max] = model.limits[key];
    const el = document.createElement('div'); el.className = 'parameter';
    el.innerHTML = `<div class="parameter-top"><label for="${key}-number">${label} <span>[${unit}]</span></label><input id="${key}-number" type="number" min="${min}" max="${max}" step="${step}" value="${params[key]}"></div><input id="${key}-range" aria-label="${label} スライダー" type="range" min="${min}" max="${max}" step="${step}" value="${params[key]}"><div class="range-labels"><span>${min}</span><span>${max}</span></div>`;
    $(container).append(el);
    const number = $(`${key}-number`), range = $(`${key}-range`);
    function change(source) {
      const value = source.valueAsNumber;
      try { model.validate({ ...params, [key]: value }); }
      catch (error) { $('input-error').textContent = error.message; source.setAttribute('aria-invalid', 'true'); return; }
      params[key] = value; number.value = range.value = value;
      number.removeAttribute('aria-invalid'); range.removeAttribute('aria-invalid'); $('input-error').textContent = '';
      recalculate();
    }
    number.addEventListener('input', () => change(number)); range.addEventListener('input', () => change(range));
  });
}
function gainControls() {
  controls('gain-controls', mode === 'pd' ? ['kp','kd'] : ['kx','kv']);
  $('gain-title').textContent = mode === 'pd' ? 'PD制御のゲイン' : '状態フィードバックのゲイン';
}
function pause() { playing = false; lastFrame = null; $('play').textContent = '▶ 再生'; }
function recalculate() {
  pause(); index = 0;
  try {
    results = { pd: model.simulate(params, 'pd'), sf: model.simulate(params, 'sf') };
    $('gain-summary').textContent = `PD: (${params.kp}, ${params.kd}) / 状態FB: (${params.kx}, ${params.kv})`;
    render();
  } catch (e) { $('input-error').textContent = e.message; }
}
function drawChart(id, key, title) {
  const canvas = $(id), rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  const w = rect.width, h = rect.height;
  if (canvas.width !== Math.round(w*ratio) || canvas.height !== Math.round(h*ratio)) { canvas.width = Math.round(w*ratio); canvas.height = Math.round(h*ratio); }
  const ctx = canvas.getContext('2d'); ctx.setTransform(ratio,0,0,ratio,0,0); ctx.clearRect(0,0,w,h);
  const left=49, right=w-14, top=25, bottom=h-25;
  const modes = $('compare').checked ? ['pd','sf'] : [mode];
  let min=0, max=key === 'x' ? params.r : 0;
  for (const m of modes) for (const s of results[m].samples) { min=Math.min(min,s[key]); max=Math.max(max,s[key]); }
  if (key === 'f') for (const s of results[mode].samples) { min=Math.min(min,s.cmd); max=Math.max(max,s.cmd); }
  const pad = Math.max((max-min)*.12, key === 'x' ? .03 : .1); min-=pad; max+=pad;
  const px=t=>left+t/5*(right-left), py=y=>bottom-(y-min)/(max-min)*(bottom-top);
  ctx.font='12px system-ui'; ctx.fillStyle='#172e3d'; ctx.fillText(title,0,12);
  ctx.font='10px system-ui';
  for(let i=0;i<=3;i++) { const y=min+(max-min)*i/3; ctx.strokeStyle='#e4e0d7';ctx.beginPath();ctx.moveTo(left,py(y));ctx.lineTo(right,py(y));ctx.stroke();ctx.fillStyle='#64717a';ctx.textAlign='right';ctx.fillText(y.toFixed(key==='f'?0:2),left-7,py(y)+3); }
  for(let t=0;t<=5;t++){ctx.textAlign='center';ctx.fillText(String(t),px(t),h-8);}
  ctx.textAlign='right';ctx.fillText('t [s]',right,12);ctx.textAlign='left';
  function line(samples, prop, color, dash) {ctx.strokeStyle=color;ctx.lineWidth=1.8;ctx.setLineDash(dash);ctx.beginPath();for(let i=0;i<samples.length;i+=5){const s=samples[i];if(i===0)ctx.moveTo(px(s.t),py(s[prop]));else ctx.lineTo(px(s.t),py(s[prop]));}ctx.stroke();ctx.setLineDash([]);}
  if(key==='x'){ctx.strokeStyle='#a38b63';ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(left,py(params.r));ctx.lineTo(right,py(params.r));ctx.stroke();ctx.setLineDash([]);}
  if(key==='f')line(results[mode].samples,'cmd','#98a1a9',[2,4]);
  for(const m of modes) line(results[m].samples,key,m==='pd'?'#19394b':'#d8652a',m==='pd'?[]:[7,5]);
  ctx.strokeStyle='#a4afb6';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(px(index*model.dt),top);ctx.lineTo(px(index*model.dt),bottom);ctx.stroke();
}
function render() {
  if (!results) return;
  const result = results[mode], s = result.samples[Math.round(index)];
  $('position').innerHTML = `${s.x.toFixed(3)} <small>m</small>`;
  $('velocity').innerHTML = `${s.v.toFixed(3)} <small>m/s</small>`;
  $('force').innerHTML = `${s.f.toFixed(2)} <small>N</small>`;
  $('carriage').style.left = `${s.x*100}%`; $('target').style.left = `${params.r*100}%`;
  $('time').textContent = `${s.t.toFixed(2)} / 5.00 s`; $('scrub').value = Math.round(index);
  $('status').className = 'status' + (result.saturated || result.boundaryCount ? ' warn' : '');
  $('status').textContent = `指令力 ${s.cmd.toFixed(2)} N / 実入力 ${s.f.toFixed(2)} N　·　${result.saturated?'5秒間に入力飽和あり':'5秒間に入力飽和なし'}${result.boundaryCount ? ' · 端点処理あり' : ''}`;
  const d = model.difference(results.pd, results.sf);
  const fmt=n=>n===0?'0':n.toExponential(2);
  $('difference').textContent = $('compare').checked ? `両方式の最大絶対差　位置 ${fmt(d.x)} m / 速度 ${fmt(d.v)} m/s / 力 ${fmt(d.f)} N` : '比較表示はオフです。選択中の方式のみ表示しています。';
  drawChart('x-chart','x','位置 x [m]');drawChart('v-chart','v','速度 v [m/s]');drawChart('f-chart','f','実入力 F [N]');
}
$('mode').addEventListener('change',()=>{ mode=$('mode').value;gainControls();recalculate(); });
$('defaults').addEventListener('click',()=>{params={...model.defaults};mode='pd';$('mode').value=mode;$('speed').value='1';$('compare').checked=true;$('input-error').textContent='';controls('plant-controls',['M','b','r','Fmax']);gainControls();recalculate();});
$('equalize').addEventListener('click',()=>{if(mode==='pd'){params.kx=params.kp;params.kv=params.kd;}else{params.kp=params.kx;params.kd=params.kv;}gainControls();recalculate();});
$('compare').addEventListener('change',render);
$('play').addEventListener('click',()=>{if(playing)pause();else{if(index>=5000)index=0;playing=true;lastFrame=null;$('play').textContent='Ⅱ 一時停止';}});
$('reset').addEventListener('click',()=>{pause();index=0;render();});
$('scrub').addEventListener('input',()=>{pause();index=Number($('scrub').value);render();});
// 積分は事前に固定刻みで計算。描画頻度や倍速によって物理結果は変化しない。
function frame(now){if(playing){if(lastFrame!==null)index=Math.min(5000,index+Math.min((now-lastFrame),100)*Number($('speed').value));lastFrame=now;if(index>=5000)pause();render();}requestAnimationFrame(frame);}
window.addEventListener('resize',render);
controls('plant-controls',['M','b','r','Fmax']);gainControls();recalculate();requestAnimationFrame(frame);
}
