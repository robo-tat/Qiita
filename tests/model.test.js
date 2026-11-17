const { test } = require('node:test');
const assert = require('node:assert/strict');
const m = require('../docs/control-intro/script.js');
test('等しいゲインでは全5001サンプルの位置・速度・力が一致', () => {
  for (const p of [m.defaults, {...m.defaults,M:.2,b:0,Fmax:1,kp:500,kx:500,kd:0,kv:0}]) {
    const a=m.simulate(p,'pd'), b=m.simulate(p,'sf');
    assert.equal(a.samples.length,5001);assert.deepEqual(m.difference(a,b),{x:0,v:0,f:0});
  }
});
test('初期値は目標へ収束し、初期指令力38.4 N',()=>{const a=m.simulate(m.defaults,'pd');assert.ok(Math.abs(a.samples[0].f-38.4)<1e-10);assert.ok(Math.abs(a.samples.at(-1).x-.3)<1e-6);});
test('飽和と端点処理、全サンプルの有限性',()=>{const p={...m.defaults,M:.2,b:0,r:1,Fmax:1,kd:0};const a=m.simulate(p,'pd');assert.ok(a.saturated);assert.ok(a.boundaryCount>0);for(const s of a.samples){assert.ok(s.x>=0&&s.x<=1);assert.ok(Math.abs(s.f)<=1);for(const v of Object.values(s))assert.ok(Number.isFinite(v));}});
test('無入力・静止目標は静止し、無効入力は拒否',()=>{for(const p of [{...m.defaults,r:0},{...m.defaults,kp:0,kd:0}])assert.ok(m.simulate(p,'pd').samples.every(s=>s.x===0&&s.v===0&&s.f===0));for(const p of [{M:0},{b:-1},{r:2},{Fmax:NaN},{kd:Infinity}])assert.throws(()=>m.simulate({...m.defaults,...p},'pd'));});
test('未飽和・端点なしの解析解とRK4が一致',()=>{const p={...m.defaults,Fmax:100};const a=m.simulate(p,'pd');const alpha=(p.b+p.kd)/(2*p.M),omega=Math.sqrt(p.kp/p.M-alpha*alpha);for(const s of a.samples){const exact=p.r*(1-Math.exp(-alpha*s.t)*(Math.cos(omega*s.t)+alpha/omega*Math.sin(omega*s.t)));assert.ok(Math.abs(s.x-exact)<1e-8);}});
