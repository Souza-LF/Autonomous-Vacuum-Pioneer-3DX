// Página da simulação de controle (etapa 2): corrida do exemplo do Corke igual à do Node, ganhos e
// condições, mapa (alvo, início, mover), ré, Δt, tema escuro e 390 px. Fotos em fotos/controle_*.png.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const URL = BASE + 'simulacao.html';
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const ateFim = async () => { for (let i = 0; i < 1500 && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(40); };
const campo = (id, v, ev = 'change') => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.value = '${v}'; e.dispatchEvent(new Event('${ev}')); })()`);
const rapido = () => campo('p-escala', '10');

await b.tela(1400, 950);
await b.abrir(URL);
await dormir(500);

// 1) estado inicial
let s = await J(`{ conds: [...document.querySelectorAll('.cond')].map((c) => c.className + ' ' + c.textContent),
  instavel: document.getElementById('aviso-instavel').hidden, hud: document.getElementById('hud').innerText,
  leit: document.getElementById('leituras').innerText, sentido: window.simulador.est.m.diferencial.e.sentido }`);
conferir('3 condições verdes com os ganhos padrão', s.conds.length === 3 && s.conds.every((c) => c.startsWith('cond ok')) && s.instavel, s.conds.join(' | '));
conferir('exemplo do Corke abre de ré', s.sentido === -1 && /de ré/.test(s.hud) && /ré/.test(s.leit), s.hud.split('\n').slice(0, 3).join(' / '));

// 2) corrida inteira: igual a K.simular() com as mesmas opções (mesma função de passo)
await rapido();
await b.avaliar('window.simulador.recomecar()');
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores; const r = {};
  for (const m of ['diferencial', 'incremental', 'exato']) {
    const ref = K.simular(Object.assign({}, e.m[m].o, { P0: e.P0 }));
    r[m] = { t: e.m[m].tChegada, tRef: ref.tChegada, P: e.m[m].P, PRef: ref.P, iguais: e.m[m].P.every((x, i) => x === ref.P[i]) };
  }
  return r; })()`);
for (const m of ['diferencial', 'incremental', 'exato']) {
  const x = s[m];
  conferir(`${m}: chegada e pose final iguais às de simular() bit a bit`, x.iguais && Math.abs(x.t - x.tRef) < 1e-9,
    `t = ${x.t?.toFixed(2)} s (Node: ${x.tRef?.toFixed(2)}), P = (${x.P.map((v, i) => i === 2 ? (v * 180 / Math.PI).toFixed(2) + '°' : v.toFixed(4)).join('; ')})`);
}
s = await J(`{ hud: document.getElementById('hud').innerText, botao: document.getElementById('btn-rodar').textContent, n: window.simulador.est.hist.length }`);
conferir('HUD mostra "chegou em" e o botão vira "Rodar de novo"', /chegou em 16,55 s/.test(s.hud) && s.botao === 'Rodar de novo', s.hud.split('\n').slice(1, 3).join(' / ') + ` · ${s.n} pontos no histórico`);
await b.foto(SAIDA + 'controle_corrida.png');

// 3) condição violada: kβ = +0,5
await campo('k-beta-num', '0.5', 'input');
s = await J(`{ conds: [...document.querySelectorAll('.cond')].map((c) => c.className), instavel: document.getElementById('aviso-instavel').hidden }`);
conferir('kβ = +0,5: uma condição vermelha e o aviso aparece', s.conds.filter((c) => c === 'cond ruim').length === 1 && !s.instavel, s.conds.join(' | '));
await campo('p-tmax', '30');
await ateFim();
s = await J(`{ hud: document.getElementById('hud').innerText, chegou: window.simulador.est.m.diferencial.chegou, t: window.simulador.est.t }`);
conferir('kβ = +0,5: não chega e o HUD diz', !s.chegou && /não chegou em 30 s/.test(s.hud), `t = ${s.t.toFixed(2)} s`);
await b.foto(SAIDA + 'controle_instavel.png');
await b.avaliar(`document.getElementById('btn-ganhos').click()`);
await campo('p-tmax', '60');
s = await J(`window.simulador.est.k`);
conferir('"Ganhos padrão" volta a (0,5; 1,5; −0,5)', s.rho === 0.5 && s.alpha === 1.5 && s.beta === -0.5, JSON.stringify(s));

// 4) mapa: marcar o alvo arrastando para a direita (θ* = 0)
const tela = async (x, y) => J(`(() => { const r = document.getElementById('mundo').getBoundingClientRect(); const p = window.simulador.paraTela(${x}, ${y}); return [r.left + p[0], r.top + p[1]]; })()`);
let [px, py] = await tela(6, 6);
await b.arrastar(px, py, px + 80, py, 8);
await dormir(150);
s = await J(`{ alvo: window.simulador.est.alvo, campos: ['p-xa', 'p-ya', 'p-tha'].map((id) => document.getElementById(id).value), t: window.simulador.est.t, rodando: window.simulador.est.rodando }`);
conferir('arrastar no mapa põe o alvo em (6, 6) com θ* = 0 e recomeça', Math.abs(s.alvo[0] - 6) < 0.03 && Math.abs(s.alvo[1] - 6) < 0.03 && s.alvo[2] === 0 && s.campos[2] === '0' && s.rodando,
  `alvo (${s.alvo.map((v) => v.toFixed(2)).join('; ')}), campos ${s.campos.join(' / ')}`);
// arrastar para cima dá θ* = 90°
[px, py] = await tela(6, 6);
await b.arrastar(px, py, px, py - 80, 8);
await dormir(150);
s = await J(`window.simulador.est.alvo`);
conferir('arrastar para cima dá θ* = 90°', Math.abs(s[2] * 180 / Math.PI - 90) < 1e-9, `θ* = ${(s[2] * 180 / Math.PI).toFixed(1)}°`);

// 5) marcar o início só com um clique: muda a posição e mantém θ0
await b.avaliar(`document.querySelector('input[name="ferramenta"][value="inicio"]').click()`);
// um ponto dentro da vista atual (a 75% da largura e da altura do mapa)
const [wx, wy] = await J(`(() => { const v = window.simulador.vista; return window.simulador.paraMundo(v.w * 0.75, v.h * 0.75).map((a) => Math.round(a * 10) / 10); })()`);
[px, py] = await tela(wx, wy);
await b.arrastar(px, py, px + 2, py, 1);
await dormir(150);
s = await J(`{ P0: window.simulador.est.P0, dica: document.getElementById('dica-tela').textContent }`);
conferir(`ferramenta "início": clique em (${wx}, ${wy}) põe o início e mantém θ0`, Math.abs(s.P0[0] - wx) < 0.03 && Math.abs(s.P0[1] - wy) < 0.03 && s.P0[2] === 0 && /início/.test(s.dica),
  `P0 (${s.P0.map((v) => v.toFixed(2)).join('; ')})`);

// 6) mover a vista
await b.avaliar(`document.querySelector('input[name="ferramenta"][value="mover"]').click()`);
const vx0 = await b.avaliar('window.simulador.vista.x');
const alvoAntes = await J('window.simulador.est.alvo');
await b.arrastar(700, 300, 600, 300, 6);
const vx1 = await b.avaliar('window.simulador.vista.x');
const alvoDepois = await J('window.simulador.est.alvo');
conferir('ferramenta "mover": arrastar move a vista e não mexe no alvo', vx1 > vx0 + 0.3 && JSON.stringify(alvoAntes) === JSON.stringify(alvoDepois), `vista.x ${vx0.toFixed(2)} → ${vx1.toFixed(2)}`);

// 7) ré proibida no exemplo do Corke
await b.avaliar(`document.querySelector('[data-exemplo="corke"]').click()`);
await b.avaliar(`(() => { const e = document.getElementById('permitir-re'); e.checked = false; e.dispatchEvent(new Event('change')); })()`);
await rapido();
await ateFim();
s = await J(`{ sentido: window.simulador.est.m.diferencial.e.sentido, chegou: window.simulador.est.m.diferencial.chegou, t: window.simulador.est.m.diferencial.tChegada }`);
conferir('sem ré: o exemplo do Corke vai em frente e chega', s.sentido === 1 && s.chegou, `chegou em ${s.t?.toFixed(2)} s`);
await b.avaliar(`(() => { const e = document.getElementById('permitir-re'); e.checked = true; e.dispatchEvent(new Event('change')); })()`);

// 8) Δt = 0,2 s
await campo('p-dt-num', '0.2');
await ateFim();
s = await J(`{ dt: window.simulador.est.dt, slider: document.getElementById('p-dt').value, t: window.simulador.est.m.diferencial.tChegada, ref: window.Controladores.simular(Object.assign({}, window.simulador.est.m.diferencial.o, { P0: window.simulador.est.P0 })).tChegada }`);
conferir('Δt = 0,2 s: corrida refeita com o novo Δt', s.dt === 0.2 && Math.abs(s.t - s.ref) < 1e-9, `chegou em ${s.t?.toFixed(2)} s (Node ${s.ref?.toFixed(2)}), slider ${s.slider}`);
await campo('p-dt-num', '0.05');
await ateFim();

// 9) tema escuro e celular
await b.tela(1400, 950, true);
await dormir(400);
await b.foto(SAIDA + 'controle_escuro.png');
await b.tela(390, 844, false, true);
await b.abrir(URL);
await dormir(800);
s = await J(`{ sw: document.documentElement.scrollWidth, w: window.innerWidth }`);
conferir('390 px: sem rolagem horizontal', s.sw <= s.w, `scrollWidth ${s.sw}, largura ${s.w}`);
await b.foto(SAIDA + 'controle_celular.png');

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));
console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
