/*
 * Testes de controladores.js (Node, sem dependências).
 *
 * 1. Coordenadas polares (Fig. 4.13 do Corke): casos feitos à mão, o β do livro com o alvo na
 *    origem e a invariância a uma rotação da cena inteira.
 * 2. Condições de estabilidade e escolha do sentido (ré quando o alvo está atrás).
 * 3. Convergência com os ganhos da interface, de 96 poses iniciais (3 distâncias × 8 direções
 *    × 4 orientações), nos três modelos; o que a ré muda quando o alvo está atrás; e o exemplo
 *    do Corke, de (9, 5, 0) a (5, 5, 90°).
 * 4. Cada condição violada faz a malha falhar; e um caso que cumpre as três condições mas
 *    deixa α sair de (−π/2, π/2]. A condição forte de Siegwart e Nourbakhsh separa os dois casos,
 *    também nos ganhos mais perto da fronteira dela.
 * 5. Saturação das rodas dentro da malha: o limite é respeitado e o raio de curva do comando, não.
 * 6. Δt: distância entre a trajetória de cada modelo e a do arco exato com o mesmo Δt
 *    (Euler cai com Δt, o incremental com Δt²).
 * 7. Malha pela odometria: sem erros ela chega como a pose verdadeira; a estimativa não depende
 *    do robô real; com uma roda maior ou escorregando, a estimativa chega e o robô real, não.
 * 8. Ir a um ponto (4.1.1.1): o exemplo do Corke; as partidas da Fig. 4.8 chegam com orientações
 *    diferentes; a grade chega; com Kh < Kv o robô chega em espiral, com α parado no equilíbrio,
 *    e abaixo de Kh/Kv = 2/π a espiral é para fora.
 * 9. Seguir uma reta (4.1.1.2): a reta (1, −2, 4) e as partidas da Fig. 4.10, dos dois lados;
 *    a reta marcada por uma pose; a grade converge; o sobressinal perto da reta segue o do
 *    segundo grau com ζ = Kh / (2√(v·Kd)); com Kd, Kh ≤ 0 não converge; com uma roda maior, sobra
 *    o erro estacionário d = ω/Kd previsto.
 * 10. Seguir um caminho (4.1.1.3, pure pursuit): amostragem e ponto perseguido; os caminhos prontos
 *    chegam ao fim; o corte da quina só é proporcional a d quando d ≫ v/Kh; no 8, a busca do Corke
 *    no caminho inteiro faz j saltar de ramo no cruzamento; Kh ≤ 0 não segue; odometria.
 * 11. O exemplo do Corke de cada lei (EXEMPLOS_CORKE, o da página de escolha) chega nos três modelos.
 *
 * Uso (na pasta do simulador):
 *   node testes/teste_controladores.js
 */
'use strict';
const path = require('path');
const K = require(path.join(__dirname, '..', 'controladores.js'));

let falhas = 0;
function confere(cond, msg) {
  console.log((cond ? 'ok    ' : 'FALHA ') + msg);
  if (!cond) falhas++;
}
const deg = (x) => (x * Math.PI) / 180;
const quase = (a, b, tol = 1e-12) => Math.abs(a - b) < tol;

// 1. Coordenadas polares
{
  const c = (P, alvo, rho, alpha, beta) => {
    const p = K.polares(P, alvo);
    return quase(p.rho, rho) && quase(p.alpha, alpha) && quase(p.beta, beta);
  };
  confere(c([0, 0, 0], [1, 0, 0], 1, 0, 0), 'alvo 1 m à frente e alinhado: (ρ, α, β) = (1, 0, 0)');
  confere(c([0, 0, 0], [0, 1, 0], 1, Math.PI / 2, -Math.PI / 2), 'alvo à esquerda: α = 90°, β = −90°');
  confere(c([0, 0, 0], [-1, 0, Math.PI], 1, Math.PI, 0), 'alvo atrás e virado: α = 180°, β = 0');
  confere(c([9, 5, 0], [5, 5, Math.PI / 2], 4, Math.PI, -Math.PI / 2), 'exemplo do Corke (9, 5, 0) → (5, 5, 90°): ρ = 4, α = 180°, β = −90°');

  let maxLivro = 0, maxRot = 0;
  for (let i = 0; i < 1000; i++) {
    const P = [10 * (Math.random() - 0.5), 10 * (Math.random() - 0.5), 2 * Math.PI * (Math.random() - 0.5)];
    // com o alvo na origem e θ* = 0, o livro dá β = −θ − α
    const p = K.polares(P, [0, 0, 0]);
    maxLivro = Math.max(maxLivro, Math.abs(K.angulo(p.beta - (-P[2] - p.alpha))));
    // girar robô e alvo pelo mesmo φ em torno da origem não muda (ρ, α, β)
    const alvo = [10 * (Math.random() - 0.5), 10 * (Math.random() - 0.5), 2 * Math.PI * (Math.random() - 0.5)];
    const phi = 2 * Math.PI * Math.random(), cs = Math.cos(phi), sn = Math.sin(phi);
    const gira = (Q) => [cs * Q[0] - sn * Q[1], sn * Q[0] + cs * Q[1], Q[2] + phi];
    const a = K.polares(P, alvo), b = K.polares(gira(P), gira(alvo));
    maxRot = Math.max(maxRot, Math.abs(a.rho - b.rho), Math.abs(K.angulo(a.alpha - b.alpha)), Math.abs(K.angulo(a.beta - b.beta)));
  }
  confere(maxLivro < 1e-12, `alvo na origem: β = −θ − α como no livro (erro máx. ${maxLivro.toExponential(2)} rad)`);
  confere(maxRot < 1e-9, `(ρ, α, β) não mudam ao girar a cena inteira (erro máx. ${maxRot.toExponential(2)})`);
}

// 2. Estabilidade e sentido
{
  const ok = (k) => K.condicoesPose(k).every((c) => c.ok);
  confere(ok(K.GANHOS_POSE), `ganhos da interface (kρ, kα, kβ) = (${K.GANHOS_POSE.rho}, ${K.GANHOS_POSE.alpha}, ${K.GANHOS_POSE.beta}) cumprem as três condições`);
  confere(!ok({ rho: 0.5, alpha: 1.5, beta: 0.5 }) && !ok({ rho: 1, alpha: 0.5, beta: -0.5 }) && !ok({ rho: -0.5, alpha: 1.5, beta: -0.5 }),
    'kβ > 0, kα < kρ e kρ < 0 são reprovados');
  confere(K.sentidoPose([0, 0, 0], [1, 0.5, 0]) === 1 && K.sentidoPose([0, 0, 0], [-1, 0.5, 0]) === -1,
    'alvo à frente: segue em frente; alvo atrás: ré');
  confere(K.sentidoPose([0, 0, 0], [0, 1, 0]) === 1 && K.sentidoPose([0, 0, 0], [0, -1, 0]) === -1,
    'bordas do intervalo (−π/2, π/2]: α = +90° segue em frente, α = −90° dá ré');
}

// Grade de poses iniciais em volta do alvo (0, 0, 90°): metade com o alvo atrás
const ALVO = [0, 0, deg(90)];
const GRADE = [];
for (const R of [1, 2, 4])
  for (let b = 0; b < 8; b++)
    for (let h = 0; h < 4; h++) {
      const ang = (b * Math.PI) / 4;
      GRADE.push([R * Math.cos(ang), R * Math.sin(ang), K.angulo((h * Math.PI) / 2 + 0.3)]);
    }

function rodarGrade(k, extra) {
  const r = { chegaram: 0, tMax: 0, re: 0, saiuI1: 0, saturou: 0, maxRoda: 0, maxCurva: 0, maxTh: 0, maxRho: 0 };
  for (const P0 of GRADE) {
    const s = K.simular(Object.assign({ lei: 'pose', alvo: ALVO, k, P0, tmax: 120 }, extra));
    if (s.chegou) {
      r.chegaram++;
      r.tMax = Math.max(r.tMax, s.tChegada);
      r.maxRho = Math.max(r.maxRho, Math.hypot(s.P[0] - ALVO[0], s.P[1] - ALVO[1]));
      r.maxTh = Math.max(r.maxTh, Math.abs(K.angulo(ALVO[2] - s.P[2])));
    }
    if (s.estado.sentido < 0) r.re++;
    if (s.amostras.some((a) => Math.abs(a.alpha) > Math.PI / 2)) r.saiuI1++;
    if (s.amostras.some((a) => a.saturou)) r.saturou++;
    for (const a of s.amostras) {
      r.maxRoda = Math.max(r.maxRoda, Math.abs(a.wl), Math.abs(a.wr));
      // curvatura ω/v do que o robô fez contra a do comando (só onde v não é quase zero)
      if (Math.abs(a.v) > 1e-3) r.maxCurva = Math.max(r.maxCurva, Math.abs(a.wReal / a.vReal - a.w / a.v));
    }
  }
  return r;
}

// 3. Convergência
for (const modelo of ['diferencial', 'incremental', 'exato']) {
  const r = rodarGrade(K.GANHOS_POSE, { modelo });
  confere(r.chegaram === GRADE.length, `${modelo}: ${r.chegaram} de ${GRADE.length} poses chegam (ρ < 1 cm e |θ* ⊖ θ| < 1°); ` +
    `a mais lenta em ${r.tMax.toFixed(1)} s`);
  confere(r.re === GRADE.length / 2 && r.saiuI1 === 0, `${modelo}: ${r.re} começam de ré, e α nunca sai de (−π/2, π/2]`);
  if (modelo === 'diferencial') {
    // 5. Saturação dentro da malha
    confere(r.saturou > 0 && r.maxRoda <= K.PADRAO.wmax + 1e-12,
      `rodas saturam em ${r.saturou} de ${GRADE.length} poses, e nunca passam de ${(K.PADRAO.wmax * 180 / Math.PI).toFixed(0)}°/s ` +
      `(máx. ${(r.maxRoda * 180 / Math.PI).toFixed(3)}°/s)`);
    confere(r.maxCurva < 1e-9, `saturar não muda a curvatura ω/v do comando (erro máx. ${r.maxCurva.toExponential(2)} rad/m)`);
  }
}
{
  // a ré não é o que faz chegar: proibida, as 48 poses com o alvo atrás chegam em frente, por um caminho mais longo
  const comprimento = (R) => R.slice(1).reduce((L, Q, i) => L + Math.hypot(Q[0] - R[i][0], Q[1] - R[i][1]), 0);
  const media = { re: [0, 0, 0], frente: [0, 0, 0] };
  for (const P0 of GRADE) {
    if (K.sentidoPose(P0, ALVO) > 0) continue;
    for (const [nome, re] of [['re', true], ['frente', false]]) {
      const s = K.simular({ lei: 'pose', alvo: ALVO, k: K.GANHOS_POSE, P0, re, tmax: 120 });
      if (s.chegou) { media[nome][0]++; media[nome][1] += s.tChegada; media[nome][2] += comprimento(s.R); }
    }
  }
  const [nR, tR, LR] = media.re, [nF, tF, LF] = media.frente;
  confere(nR === GRADE.length / 2 && nF === GRADE.length / 2 && LR < LF,
    `alvo atrás, ${GRADE.length / 2} poses: de ré chegam ${nR} (média ${(tR / nR).toFixed(1)} s, ${(LR / nR).toFixed(2)} m); ` +
    `sem ré chegam ${nF} (média ${(tF / nF).toFixed(1)} s, ${(LF / nF).toFixed(2)} m)`);
}
{
  const s = K.simular({ lei: 'pose', alvo: [5, 5, deg(90)], k: K.GANHOS_POSE, P0: [9, 5, 0] });
  const [x, y, th] = s.P;
  confere(s.chegou && s.estado.sentido === -1, `exemplo do Corke: chega de ré em ${s.tChegada.toFixed(2)} s`);
  console.log(`       pose final = (${x.toFixed(4)} m, ${y.toFixed(4)} m, ${(th * 180 / Math.PI).toFixed(2)}°)`);
}

// 4. Condições violadas
for (const [nome, k, maxChegam] of [
  ['kβ > 0', { rho: 0.5, alpha: 1.5, beta: 0.5 }, 0],
  ['kρ < 0', { rho: -0.5, alpha: 1.5, beta: -0.5 }, 0],
  ['kα < kρ', { rho: 1, alpha: 0.5, beta: -0.5 }, GRADE.length / 10],
]) {
  const r = rodarGrade(k, {});
  confere(r.chegaram <= maxChegam, `${nome} (${k.rho}, ${k.alpha}, ${k.beta}): só ${r.chegaram} de ${GRADE.length} chegam em 120 s`);
}
{
  // cumpre as três condições, chega sempre, mas α sai do intervalo em que as equações polares valem
  const k = { rho: 1, alpha: 2, beta: -1 };
  const r = rodarGrade(k, {});
  confere(r.chegaram === GRADE.length && r.saiuI1 > 0,
    `(1, 2, −1) cumpre as três condições: chegam ${r.chegaram} de ${GRADE.length}, mas α sai de (−π/2, π/2] em ${r.saiuI1}`);
}
{
  // a condição forte de Siegwart e Nourbakhsh (eq. 3.65), kα + 5/3·kβ − 2/π·kρ > 0, é a que esses ganhos
  // violam. Numa varredura de 144 conjuntos que cumprem as três condições, ela separou todos; aqui, os
  // seis mais perto da fronteira de cada lado.
  const f = K.condicaoFortePose;
  const padrao = f(K.GANHOS_POSE), um = f({ rho: 1, alpha: 2, beta: -1 });
  confere(padrao.ok && Math.abs(padrao.margem - 0.3484) < 1e-4 && !um.ok && Math.abs(um.margem + 0.3033) < 1e-4,
    `condição forte: os ganhos da interface a cumprem (margem ${padrao.margem.toFixed(4)}), (1, 2, −1) não (${um.margem.toFixed(4)})`);
  const perto = [
    [0.25, 0.375, -0.125], [0.5, 0.75, -0.25], [1, 1.5, -0.5], [0.25, 0.3, -0.0625], [2, 3, -1], [0.5, 0.6, -0.125],
    [0.25, 0.75, -0.375], [0.25, 0.3, -0.125], [0.5, 1.5, -0.75], [0.25, 0.5, -0.25], [0.5, 0.6, -0.25], [1, 3, -1.5],
  ];
  let certos = 0;
  const lista = [];
  for (const [rho, alpha, beta] of perto) {
    const k = { rho, alpha, beta };
    const r = rodarGrade(k, {});
    if (f(k).ok === (r.saiuI1 === 0)) certos++;
    lista.push(`${f(k).margem.toFixed(3)} → ${r.saiuI1}`);
  }
  confere(certos === perto.length,
    `perto da fronteira, a forte vale ⇔ α nunca sai de (−π/2, π/2] nas 96 poses, em ${certos} de ${perto.length} conjuntos ` +
    `(margem → poses em que α sai: ${lista.join('; ')})`);
}

// 6. Δt
{
  const caso = { lei: 'pose', alvo: [5, 5, deg(90)], k: K.GANHOS_POSE, P0: [9, 5, 0] };
  const desvio = {};
  for (const dt of [0.025, 0.05, 0.1, 0.2]) {
    const ex = K.simular(Object.assign({ modelo: 'exato', dt }, caso));
    for (const modelo of ['diferencial', 'incremental']) {
      const s = K.simular(Object.assign({ modelo, dt }, caso));
      const n = Math.min(s.R.length, ex.R.length);
      let m = 0;
      for (let i = 0; i < n; i++) m = Math.max(m, Math.hypot(s.R[i][0] - ex.R[i][0], s.R[i][1] - ex.R[i][1]));
      (desvio[modelo] = desvio[modelo] || []).push(m);
    }
    console.log(`       Δt = ${dt.toFixed(3)} s: diferencial ${(desvio.diferencial.at(-1) * 1000).toFixed(3)} mm, ` +
      `incremental ${(desvio.incremental.at(-1) * 1000).toFixed(4)} mm do arco exato`);
  }
  const razoes = (v) => v.slice(1).map((x, i) => x / v[i]);
  const rd = razoes(desvio.diferencial), ri = razoes(desvio.incremental);
  confere(rd.every((q) => q > 1.7 && q < 2.5), `diferencial: dobrar Δt multiplica o desvio por ${rd.map((q) => q.toFixed(2)).join(', ')} (≈ 2, ordem 1)`);
  confere(ri.every((q) => q > 3.2 && q < 5), `incremental: dobrar Δt multiplica o desvio por ${ri.map((q) => q.toFixed(2)).join(', ')} (≈ 4, ordem 2)`);
}

// 7. Malha pela odometria
{
  const r = K.PADRAO.r;
  const corke = { lei: 'pose', alvo: [5, 5, deg(90)], k: K.GANHOS_POSE, P0: [9, 5, 0] };
  const mm = (e) => `${(e.d * 1000).toFixed(1)} mm e ${(e.th * 180 / Math.PI).toFixed(2)}°`;
  const odo = (extra) => K.simularOdometria(Object.assign({}, corke, extra));
  const verdadeira = (extra) => K.simular(Object.assign({ modelo: 'exato' }, corke, extra));

  // sem erros: com encoder fino, a odometria chega onde chega a malha com a pose verdadeira
  const fino = odo({ pulsos: 1e6 }), ver = verdadeira({});
  const eF = K.erroAoAlvo(fino.real, corke.alvo), eV = K.erroAoAlvo(ver.P, corke.alvo);
  confere(fino.chegou && Math.abs(eF.d - eV.d) < 5e-4 && Math.abs(eF.th - eV.th) < deg(0.01),
    `sem erros, 10⁶ pulsos: o robô real termina a ${mm(eF)} do alvo (pose verdadeira: ${mm(eV)})`);
  const p500 = odo({});
  const e500 = K.erroAoAlvo(p500.real, corke.alvo);
  confere(p500.chegou && e500.d < 0.02, `sem erros, 500 pulsos: chega em ${p500.tChegada.toFixed(2)} s, o robô real a ${mm(e500)} do alvo`);

  // a estimativa só depende dos pulsos, que só dependem dos comandos, que só dependem da estimativa:
  // o robô real não entra na malha
  const casos = [{}, { rr: r + 0.001 }, { rl: r + 0.002 }, { escL: 0.05 }, { escL: 0.05, escR: 0.05 }];
  const estimativas = casos.map((c) => JSON.stringify(odo(c).O));
  confere(estimativas.every((e) => e === estimativas[0]),
    `a estimativa é a mesma, bit a bit, sem erro, com roda maior e com escorregamento (${casos.length} casos)`);

  // roda direita 2 mm maior: a estimativa chega, o robô real não; com a pose verdadeira, chega
  const d2 = odo({ rr: r + 0.001 }), v2 = verdadeira({ rr: r + 0.001 });
  const eR = K.erroAoAlvo(d2.real, corke.alvo), eE = K.erroAoAlvo(d2.odo, corke.alvo), eV2 = K.erroAoAlvo(v2.P, corke.alvo);
  confere(d2.chegou && eE.d < 0.01 && eR.d > 0.1 && v2.chegou && eV2.d < 0.01,
    `direita 2 mm maior: a estimativa chega (${mm(eE)}), o robô real fica a ${mm(eR)}; com a pose verdadeira, ${mm(eV2)}`);

  // o erro final cresce na proporção do erro de diâmetro
  const erros = [0.5, 1, 2, 4].map((dD) => K.erroAoAlvo(odo({ rr: r + dD / 2000 }).real, corke.alvo).d);
  const razoes = erros.slice(1).map((x, i) => x / erros[i]);
  confere(razoes.every((q) => q > 1.7 && q < 2.3),
    `direita 0,5 / 1 / 2 / 4 mm maior: ${erros.map((e) => (e * 1000).toFixed(0)).join(' / ')} mm do alvo ` +
    `(dobrar o erro dobra a distância: ${razoes.map((q) => q.toFixed(2)).join(', ')})`);

  const esc = odo({ escL: 0.05 });
  const eEsc = K.erroAoAlvo(esc.real, corke.alvo);
  confere(esc.chegou && eEsc.d > 0.5, `roda esquerda escorregando 5%: a estimativa chega, o robô real fica a ${mm(eEsc)} do alvo`);

  // a grade inteira com a direita 2 mm maior
  let n = 0, chegam = 0, soma = 0, max = 0, maxVer = 0;
  for (const P0 of GRADE) {
    const o = { lei: 'pose', alvo: ALVO, k: K.GANHOS_POSE, P0, rr: r + 0.001, tmax: 120 };
    const s = K.simularOdometria(o);
    const d = K.erroAoAlvo(s.real, ALVO).d;
    n++;
    if (s.chegou) chegam++;
    soma += d;
    max = Math.max(max, d);
    maxVer = Math.max(maxVer, K.erroAoAlvo(K.simular(Object.assign({ modelo: 'exato' }, o)).P, ALVO).d);
  }
  confere(chegam === n && maxVer < 0.01 && soma / n > 0.05,
    `grade, direita 2 mm maior: a estimativa chega em ${chegam} de ${n}; o robô real fica em média a ${(soma / n * 1000).toFixed(0)} mm ` +
    `(máx. ${(max * 1000).toFixed(0)} mm); com a pose verdadeira, no máx. ${(maxVer * 1000).toFixed(1)} mm`);
}

// 8. Ir a um ponto
{
  const s = K.simular({ lei: 'ponto', alvo: [5, 5, 0], k: K.GANHOS_PONTO, P0: [8, 5, deg(90)] });
  const yMax = Math.max(...s.R.map((p) => p[1]));
  confere(s.chegou && s.amostras.every((a) => a.v >= 0),
    `ponto, exemplo do Corke (8, 5, 90°) → (5, 5): chega em ${s.tChegada.toFixed(2)} s, sempre em frente, ` +
    `subindo até y = ${yMax.toFixed(2)} m e terminando virado para ${(s.P[2] * 180 / Math.PI).toFixed(1)}°`);

  // Fig. 4.8: 8 partidas a 3 m do alvo, tangentes no sentido anti-horário. A lei não muda ao girar a
  // cena, então cada uma chega girada de 45° da anterior: a orientação final depende da partida.
  const finais = [];
  let todas = true;
  for (let i = 0; i < 8; i++) {
    const b = (i * Math.PI) / 4;
    const p = K.simular({ lei: 'ponto', alvo: [5, 5, 0], k: K.GANHOS_PONTO, P0: [5 + 3 * Math.cos(b), 5 + 3 * Math.sin(b), K.angulo(b + Math.PI / 2)] });
    todas = todas && p.chegou;
    finais.push(p.P[2]);
  }
  const desvio = Math.max(...finais.map((t, i) => Math.abs(K.angulo(t - finais[0] - (i * Math.PI) / 4))));
  confere(todas && desvio < 1e-9,
    `Fig. 4.8: as 8 partidas chegam, cada uma girada de 45° da anterior (erro máx. ${desvio.toExponential(1)} rad): a orientação final não é controlada`);

  for (const modelo of ['diferencial', 'incremental', 'exato']) {
    let n = 0, tMax = 0;
    for (const P0 of GRADE) {
      const p = K.simular({ lei: 'ponto', alvo: ALVO, k: K.GANHOS_PONTO, P0, modelo, tmax: 120 });
      if (p.chegou) { n++; tMax = Math.max(tMax, p.tChegada); }
    }
    confere(n === GRADE.length, `ponto, ${modelo}: ${n} de ${GRADE.length} poses chegam (ρ < 1 cm); a mais lenta em ${tMax.toFixed(1)} s`);
  }

  // Kh < Kv: α não vai a zero, para onde Kv·sin α = Kh·α. Com Kh/Kv = 0,8 (acima de 2/π) a espiral
  // é para dentro e todos chegam, sem nunca apontar para o alvo; com Kh/Kv = 0,6, é para fora.
  const k8 = { v: 1, h: 0.8 }, ae = K.equilibrioPonto(k8);
  const naChegada = [];
  let chegam8 = 0;
  for (const P0 of GRADE) {
    const p = K.simular({ lei: 'ponto', alvo: ALVO, k: k8, P0, modelo: 'exato', dt: 0.01, tmax: 120 });
    if (p.chegou) chegam8++;
    naChegada.push(Math.abs(p.amostras[p.amostras.length - 1].alpha));
  }
  naChegada.sort((a, b) => a - b);
  const mediana = naChegada[GRADE.length / 2];
  confere(chegam8 === GRADE.length && naChegada[0] > deg(30) && Math.abs(mediana - ae) < deg(0.5),
    `Kh/Kv = 0,8: chegam ${chegam8} de ${GRADE.length} em espiral; |α| na chegada vai de ${(naChegada[0] * 180 / Math.PI).toFixed(1)}° a ` +
    `${(naChegada[GRADE.length - 1] * 180 / Math.PI).toFixed(1)}°, mediana ${(mediana * 180 / Math.PI).toFixed(2)}° (equilíbrio ${(ae * 180 / Math.PI).toFixed(2)}°, Δt = 0,01 s)`);

  const k6 = { v: 1, h: 0.6 };
  let chegam6 = 0, longe = 0;
  for (const P0 of GRADE) {
    const p = K.simular({ lei: 'ponto', alvo: ALVO, k: k6, P0, tmax: 120 });
    if (p.chegou) chegam6++;
    if (Math.hypot(p.P[0], p.P[1]) > Math.hypot(P0[0], P0[1])) longe++;
  }
  confere(chegam6 === 0 && longe === GRADE.length && K.equilibrioPonto(k6) > Math.PI / 2,
    `Kh/Kv = 0,6 (abaixo de 2/π): equilíbrio em ${(K.equilibrioPonto(k6) * 180 / Math.PI).toFixed(1)}°, ` +
    `nenhuma das ${GRADE.length} chega e todas terminam mais longe do que começaram`);

  const viola = (k) => GRADE.filter((P0) => K.simular({ lei: 'ponto', alvo: ALVO, k, P0, tmax: 120 }).chegou).length;
  const nV = viola({ v: -0.5, h: 1.5 }), nH = viola({ v: 0.5, h: -1 });
  confere(nV === 0 && nH === 0 && K.condicoesPonto(K.GANHOS_PONTO).every((c) => c.ok) && !K.condicoesPonto(k8).every((c) => c.ok),
    `Kv < 0 ou Kh < 0: chegam ${nV} e ${nH} de ${GRADE.length}; as condições Kv > 0 e Kh > Kv aprovam os ganhos da Fig. 4.8 e reprovam Kh/Kv = 0,8`);
}

// 9. Seguir uma reta
{
  const L = [1, -2, 4];
  const thReta = K.direcaoReta(L);
  confere(quase(thReta, Math.atan2(1, 2)) && quase(K.distanciaReta([8, 5], L), 2 / Math.sqrt(5)) && K.distanciaReta([2, 5], L) < 0,
    `reta (1, −2, 4): θ* = atan2(a, −b) = ${(thReta * 180 / Math.PI).toFixed(2)}° (para cima e à direita); d(8, 5) = +${(2 / Math.sqrt(5)).toFixed(3)} m ` +
    `(abaixo, à direita do percurso) e d(2, 5) < 0 (acima)`);
  const Lp = K.retaDaPose([4, 4, Math.atan2(1, 2)]);
  confere(Lp.every((x, i) => quase(x, L[i] / Math.sqrt(5), 1e-12)),
    'a reta marcada pela pose (4, 4, 26,57°) é a (1, −2, 4)/√5');

  // Fig. 4.10: duas partidas de cada lado da reta; todas convergem andando no sentido de θ*
  const linhas = [];
  let ok = true;
  for (const P0 of [[8, 5, deg(90)], [5, 8, deg(180)], [2, 5, deg(-90)], [5, 2, 0]]) {
    const s = K.simular({ lei: 'reta', reta: L, k: K.GANHOS_RETA, P0 });
    const d0 = K.distanciaReta(P0, L);
    const cruzou = s.R.some((p) => K.distanciaReta(p, L) * d0 < -1e-6);
    ok = ok && s.chegou && !cruzou && s.P[0] > P0[0] && Math.abs(K.angulo(s.P[2] - thReta)) < deg(1);
    linhas.push(`(${P0[0]}, ${P0[1]}) d₀ = ${d0 >= 0 ? '+' : '−'}${Math.abs(d0).toFixed(2)} m: ${s.tChegada.toFixed(2)} s`);
  }
  confere(ok, `Fig. 4.10 (Kd = 0,5, Kh = 1, v = 0,5 m/s): as 4 partidas convergem sem cruzar a reta, andando para a direita; ${linhas.join('; ')}`);

  for (const modelo of ['diferencial', 'incremental', 'exato']) {
    let n = 0, tMax = 0;
    for (const P0 of GRADE) {
      const s = K.simular({ lei: 'reta', alvo: ALVO, k: K.GANHOS_RETA, P0, modelo, tmax: 120 });
      if (s.chegou) { n++; tMax = Math.max(tMax, s.tChegada); }
    }
    confere(n === GRADE.length, `reta x = 0 rumo a +y, ${modelo}: ${n} de ${GRADE.length} poses convergem (|d| < 1 cm e erro de direção < 1°); a mais lenta em ${tMax.toFixed(1)} s`);
  }

  // Perto da reta: d̈ + Kh·ḋ + v·Kd·d = 0. Partindo alinhado a 20 cm, o quanto passa da reta é o
  // sobressinal do segundo grau, exp(−πζ/√(1 − ζ²)); com ζ ≥ 1, não passa.
  const sobre = (k) => {
    const s = K.simular({ lei: 'reta', alvo: ALVO, k, P0: [0.2, 0, deg(90)], modelo: 'exato', dt: 0.01, tolRho: 1e-4, tolTh: deg(0.01) });
    return Math.max(...s.R.map((p) => -p[0])) / 0.2;   // a reta é x = 0 e a partida está em x = +0,2 (à direita)
  };
  const casos = [{ d: 0.5, h: 0.3, v: 0.5 }, { d: 0.5, h: 0.5, v: 0.5 }, { d: 2, h: 1, v: 0.5 }].map((k) => {
    const { zeta, wn } = K.amortecimentoReta(k);
    return { zeta, wn, medido: sobre(k), teoria: Math.exp((-Math.PI * zeta) / Math.sqrt(1 - zeta * zeta)) };
  });
  const critico = sobre(K.GANHOS_RETA);
  confere(casos.every((c) => Math.abs(c.medido - c.teoria) < 0.005) && critico < 0 && quase(K.amortecimentoReta(K.GANHOS_RETA).zeta, 1),
    `sobressinal a partir de 20 cm: ${casos.map((c) => `ζ = ${c.zeta.toFixed(2)} e ωn = ${c.wn.toFixed(2)} rad/s: ${(100 * c.medido).toFixed(2)}% (teoria ${(100 * c.teoria).toFixed(2)}%)`).join('; ')}; ` +
    `com os ganhos padrão, ζ = 1, não passa da reta`);

  const viola = (k) => GRADE.filter((P0) => K.simular({ lei: 'reta', alvo: ALVO, k, P0, tmax: 120 }).chegou).length;
  const r = [{ d: -0.5, h: 1, v: 0.5 }, { d: 0.5, h: -1, v: 0.5 }, { d: 0.5, h: 0, v: 0.5 }].map(viola);
  confere(r.every((n) => n === 0), `Kd < 0, Kh < 0 e Kh = 0 (sem amortecimento): convergem ${r.join(', ')} de ${GRADE.length}`);

  // Roda direita maior, com a pose verdadeira: andando reto com v constante, o robô curva sozinho, e
  // a lei só compensa com ω ≠ 0, o que pede d ≠ 0. Sobra um erro estacionário d = ω/Kd, com
  // ω = −v·(rr − r) / (l·(rr + r)), o ω que deixa as rodas reais iguais (na pose e no ponto, v → 0
  // no alvo e o viés some junto).
  const rN = K.PADRAO.r, lN = K.PADRAO.l;
  const medidos = [], previstos = [];
  let maxErro = 0;
  for (const dD of [1, 2, 4]) {
    const rr = rN + dD / 2000;
    const previsto = (-K.GANHOS_RETA.v * (rr - rN)) / (lN * (rr + rN)) / K.GANHOS_RETA.d;
    for (const modelo of ['diferencial', 'incremental', 'exato']) {
      const o = { lei: 'reta', reta: L, k: K.GANHOS_RETA, P0: [8, 5, deg(90)], rr, modelo, tolRho: 1e-6, tolTh: 1e-6 };
      const d = K.distanciaReta(K.simular(o).P, L);
      maxErro = Math.max(maxErro, Math.abs(d - previsto));
      if (modelo === 'exato') { medidos.push(d); previstos.push(previsto); }
    }
  }
  confere(maxErro < 1e-5,
    `direita 1 / 2 / 4 mm maior, pose verdadeira: o robô assenta a ${medidos.map((d) => (d * 1000).toFixed(2)).join(' / ')} mm da reta ` +
    `(previsto ω/Kd: ${previstos.map((d) => (d * 1000).toFixed(2)).join(' / ')} mm; diferença máx. ${(maxErro * 1000).toExponential(1)} mm nos três modelos)`);
}

// 10. Seguir um caminho (pure pursuit)
{
  const cam = K.amostrarCaminho([[0, 0], [3, 0], [3, 3]]);
  confere(Math.abs(cam.total - 6) < 1e-12 && cam.n === 601 && quase(cam.P[300][0], 3) && quase(cam.P[300][1], 0, 1e-12) &&
    K.perseguido(cam, 0, 0.5) === 50 && K.perseguido(cam, 590, 0.5) === 600,
    'caminho amostrado a cada 1 cm (6 m, 601 pontos); o ponto perseguido fica a d = 0,5 m adiante, e no fim é o último');

  // os caminhos prontos, nos três modelos
  const linhas = [];
  let ok = true;
  for (const [nome, c] of Object.entries(K.CAMINHOS)) {
    const tempos = [];
    for (const modelo of ['diferencial', 'incremental', 'exato']) {
      const s = K.simular({ lei: 'caminho', caminho: c.W, k: K.GANHOS_CAMINHO, P0: c.P0, modelo, tmax: 120 });
      const fim = c.W[c.W.length - 1];
      ok = ok && s.chegou && Math.hypot(fim[0] - s.P[0], fim[1] - s.P[1]) < 0.01;
      tempos.push(s.tChegada);
    }
    linhas.push(`${nome} ${K.amostrarCaminho(c.W).total.toFixed(2)} m em ${tempos[0].toFixed(1)} s`);
  }
  confere(ok, `caminhos prontos (d = 0,5 m, Kh = 2, v = 0,4 m/s): todos chegam ao fim nos três modelos; ${linhas.join('; ')}`);

  // O Corke diz que o corte da quina é proporcional a d. Só vale quando d ≫ v/Kh: o atraso para virar
  // é outra escala de comprimento. Quina de 90° em (3, 0); o corte é a menor distância ao vértice.
  const corte = (h) => [0.125, 0.25, 0.5, 1].map((d) => {
    const s = K.simular({ lei: 'caminho', caminho: [[0, 0], [3, 0], [3, 3]], k: { d, h, v: 0.4 }, P0: [0, 0, 0], modelo: 'exato', dt: 0.01 });
    return Math.min(...s.R.map((p) => Math.hypot(p[0] - 3, p[1])));
  });
  const razao = (c) => c.slice(1).map((x, i) => x / c[i]);
  const c20 = corte(20), c2 = corte(2);
  confere(razao(c20).every((q) => q > 1.95 && q < 2.2) && razao(c2)[0] > 2.8,
    `corte da quina com d = 0,125 / 0,25 / 0,5 / 1 m: com Kh = 20 (v/Kh = 2 cm), ${c20.map((x) => (x * 1000).toFixed(0)).join(' / ')} mm, ` +
    `dobrar d dobra o corte (${razao(c20).map((q) => q.toFixed(2)).join(', ')}); com Kh = 2 (v/Kh = 20 cm), ${c2.map((x) => (x * 1000).toFixed(0)).join(' / ')} mm ` +
    `(${razao(c2).map((q) => q.toFixed(2)).join(', ')})`);

  // No cruzamento do 8, a busca do Corke (j no caminho inteiro) faz j saltar de ramo: o progresso s
  // pula mais de 1 m e volta. A busca a partir do j anterior nunca volta. Nos dois casos o robô termina.
  const oito = K.CAMINHOS.oito;
  const corrida = (buscaGlobal) => {
    const s = K.simular({ lei: 'caminho', caminho: oito.W, k: K.GANHOS_CAMINHO, P0: oito.P0, buscaGlobal, modelo: 'exato', tmax: 120 });
    let saltos = 0, voltas = 0;
    for (let i = 1; i < s.amostras.length; i++) {
      const ds = s.amostras[i].s - s.amostras[i - 1].s;
      if (ds > 1) saltos++;
      if (ds < 0) voltas++;
    }
    return { s, saltos, voltas };
  };
  const jan = corrida(false), glob = corrida(true);
  confere(jan.s.chegou && glob.s.chegou && jan.saltos === 0 && jan.voltas === 0 && glob.saltos >= 1 && glob.voltas >= 1,
    `8: a partir do j anterior, j nunca salta nem volta (${jan.s.tChegada.toFixed(2)} s); no caminho inteiro (Corke), ` +
    `j salta de ramo ${glob.saltos} vez(es) no cruzamento e volta, e o robô termina em ${glob.s.tChegada.toFixed(2)} s`);

  const parado = [0, -1].map((h) => K.simular({ lei: 'caminho', caminho: K.CAMINHOS.corke.W, k: { d: 0.5, h, v: 0.4 }, P0: K.CAMINHOS.corke.P0, tmax: 120 }));
  confere(parado.every((s) => !s.chegou && Math.max(...s.amostras.map((a) => a.s)) < 0.5),
    'Kh = 0 e Kh < 0: o robô não vira para o caminho e não avança nele');

  const odo = K.simularOdometria({ lei: 'caminho', caminho: K.CAMINHOS.corke.W, k: K.GANHOS_CAMINHO, P0: K.CAMINHOS.corke.P0, tmax: 120, rr: K.PADRAO.r + 0.001 });
  const fimOdo = Math.hypot(odo.real[0] - 5, odo.real[1] - 1);
  confere(odo.chegou && fimOdo > 0.5, `pela odometria, com a direita 2 mm maior: a estimativa chega ao fim em ${odo.tChegada.toFixed(2)} s; o robô real fica a ${fimOdo.toFixed(2)} m dele`);
}

// 11. Os exemplos do Corke de cada lei, que abrem a simulação e aparecem na página de escolha
{
  const GANHOS = { pose: K.GANHOS_POSE, ponto: K.GANHOS_PONTO, reta: K.GANHOS_RETA, caminho: K.GANHOS_CAMINHO };
  for (const [lei, ex] of Object.entries(K.EXEMPLOS_CORKE)) {
    const t = ['diferencial', 'incremental', 'exato'].map((modelo) =>
      K.simular(Object.assign({ lei, modelo, k: GANHOS[lei], tmax: 120 }, ex)));
    confere(t.every((s) => s.chegou), `exemplo do Corke, ${lei}: chega nos três modelos (${t.map((s) => s.tChegada.toFixed(2)).join(' / ')} s)`);
  }
}

console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntodos os testes passaram');
process.exitCode = falhas ? 1 : 0;
