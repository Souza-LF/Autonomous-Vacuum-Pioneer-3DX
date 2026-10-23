/*
 * Testes de cinematica.js (Node, sem dependências).
 *
 * 1. Os dois programas dos slides (gerados por referencia_professor.py) são refeitos com
 *    cinematica.js e comparados ponto a ponto: mesmo número de passos e mesma trajetória.
 * 2. Propriedades das fórmulas: ida e volta direta -> inversa, sinal de ω com rodas de
 *    diâmetros diferentes, saturação que preserva o raio de curva, ICC.
 * 3. Ordem de convergência contra o arco exato: Euler (diferencial) cai com Δt,
 *    o incremental cai com Δt².
 *
 * Uso (na pasta do simulador):
 *   python testes/referencia_professor.py
 *   node testes/teste_cinematica.js
 */
'use strict';
const path = require('path');
const C = require(path.join(__dirname, '..', 'cinematica.js'));
const ref = require(path.join(__dirname, 'referencia.json'));

let falhas = 0;
function confere(cond, msg) {
  console.log((cond ? 'ok    ' : 'FALHA ') + msg);
  if (!cond) falhas++;
}
const deg = (x) => (x * Math.PI) / 180;

// 1. Programas dos slides
for (const modelo of ['diferencial', 'incremental']) {
  const d = ref[modelo];
  const passos = C.passosDoLaco(d.tmax, d.dt);
  confere(passos === d.passos, `${modelo}: passos do laço = ${passos} (Python: ${d.passos})`);

  const sim = C.simular(modelo, d.R[0], d.wl, d.wr, d.r, d.r, d.l, d.dt, d.passos);
  let maxDif = 0, iguais = 0;
  for (let k = 0; k < d.R.length; k++) {
    for (let i = 0; i < 3; i++) {
      const e = Math.abs(sim.R[k][i] - d.R[k][i]);
      maxDif = Math.max(maxDif, e);
      if (e === 0) iguais++;
    }
  }
  const total = d.R.length * 3;
  confere(maxDif < 1e-12, `${modelo}: maior diferença na trajetória = ${maxDif.toExponential(2)} ` +
    `(${iguais} de ${total} coordenadas idênticas bit a bit)`);
  const [x, y, th] = sim.P;
  console.log(`       pose final = (${x.toFixed(6)} m, ${y.toFixed(6)} m, ${(th * 180 / Math.PI).toFixed(4)}°)`);
}

// 2. Propriedades das fórmulas
const r = 0.0975, l = 0.1905;
let maxIda = 0;
for (let i = 0; i < 1000; i++) {
  const rl = r * (0.8 + 0.4 * Math.random()), rr = r * (0.8 + 0.4 * Math.random());
  const wl = deg(720 * (Math.random() - 0.5)), wr = deg(720 * (Math.random() - 0.5));
  const { v, w } = C.direta(wl, wr, rl, rr, l);
  const volta = C.inversa(v, w, rl, rr, l);
  maxIda = Math.max(maxIda, Math.abs(volta.wl - wl), Math.abs(volta.wr - wr));
}
confere(maxIda < 1e-12, `direta -> inversa devolve as rodas (erro máx. ${maxIda.toExponential(2)} rad/s)`);

{
  const reta = C.direta(deg(90), deg(90), r, r, l);
  confere(reta.w === 0 && reta.v > 0, 'rodas iguais e φ̇_l = φ̇_r: ω = 0 (linha reta)');
  const girar = C.direta(deg(-90), deg(90), r, r, l);
  confere(girar.v === 0 && girar.w > 0, 'φ̇_l = −φ̇_r: v = 0 e ω > 0 (gira no lugar, anti-horário)');
  const esqMaior = C.direta(deg(90), deg(90), r * 1.05, r, l);
  confere(esqMaior.w < 0, 'roda esquerda maior com φ̇ iguais: ω < 0 (o robô puxa para a direita)');
  const dirMaior = C.direta(deg(90), deg(90), r, r * 1.05, l);
  confere(dirMaior.w > 0, 'roda direita maior com φ̇ iguais: ω > 0 (o robô puxa para a esquerda)');
}

{
  const s = C.saturar(deg(500), deg(250), deg(360));
  const a = C.direta(deg(500), deg(250), r, r, l), b = C.direta(s.wl, s.wr, r, r, l);
  confere(s.saturou && Math.abs(Math.max(Math.abs(s.wl), Math.abs(s.wr)) - deg(360)) < 1e-12,
    'saturação leva a roda mais rápida ao limite');
  confere(Math.abs(a.v / a.w - b.v / b.w) < 1e-12, 'saturação preserva o raio de curva R = v/ω');
  confere(!C.saturar(deg(100), deg(-200), deg(360)).saturou, 'dentro do limite não satura');
}

{
  const P = [0.3, -0.2, deg(30)];
  const { v, w } = C.direta(deg(40), deg(80), r, r, l);
  const c = C.icc(P, v, w);
  let Q = P, maxR = 0;
  for (let k = 0; k < 200; k++) {
    Q = C.passoExato(Q, v, w, 0.37);
    maxR = Math.max(maxR, Math.abs(Math.hypot(Q[0] - c.x, Q[1] - c.y) - Math.abs(c.R)));
  }
  confere(maxR < 1e-12, `passo exato mantém a distância ao ICC = |R| (desvio máx. ${maxR.toExponential(2)} m)`);
}

// 3. Convergência contra o arco exato (parâmetros do slide 27, 30 s de simulação)
{
  const wl = deg(60), wr = deg(70), T = 30;
  const erroFinal = (modelo, dt) => {
    const n = Math.round(T / dt);
    const a = C.simular(modelo, [0, 0, 0], wl, wr, r, r, l, dt, n).P;
    const b = C.simular('exato', [0, 0, 0], wl, wr, r, r, l, dt, n).P;
    return Math.hypot(a[0] - b[0], a[1] - b[1]);
  };
  for (const [modelo, ordem] of [['diferencial', 1], ['incremental', 2]]) {
    const e1 = erroFinal(modelo, 0.2), e2 = erroFinal(modelo, 0.1);
    const razao = e1 / e2;
    console.log(`       ${modelo}: erro final ${(e1 * 1000).toFixed(4)} mm (Δt = 0,2 s) e ` +
      `${(e2 * 1000).toFixed(4)} mm (Δt = 0,1 s), razão ${razao.toFixed(3)}`);
    confere(Math.abs(razao - 2 ** ordem) < 0.15 * 2 ** ordem,
      `${modelo}: erro cai como Δt^${ordem} (razão esperada ≈ ${2 ** ordem})`);
  }
  confere(erroFinal('incremental', 0.5) < erroFinal('diferencial', 0.5),
    'com o mesmo Δt, o incremental fica mais perto do arco exato que o diferencial (Euler)');
}

console.log(falhas ? `\n${falhas} falha(s)` : '\ntodos os testes passaram');
process.exit(falhas ? 1 : 0);
