// Página da simulação de controle (etapa 2): corrida do exemplo do Corke igual à do Node, ganhos e
// condições (com a condição forte), mapa (alvo, início, mover), ré, Δt, os números dos eixos e as
// legendas dos gráficos, o botão de pausa, tema escuro e 390 px. Fotos em fotos/controle_*.png.
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
let s = await J(`{ conds: [...document.querySelectorAll('.cond:not(.forte)')].map((c) => c.className + ' ' + c.textContent),
  forte: [...document.querySelectorAll('.cond.forte')].map((c) => c.className + ' ' + c.textContent), sec: document.getElementById('lei-sec').textContent,
  instavel: document.getElementById('aviso-instavel').hidden, hud: document.getElementById('hud').innerText,
  leit: document.getElementById('leituras').innerText, sentido: window.simulador.est.m.diferencial.e.sentido }`);
conferir('3 condições verdes com os ganhos padrão', s.conds.length === 3 && s.conds.every((c) => c.startsWith('cond ok')) && s.instavel, s.conds.join(' | '));
conferir('a condição forte vale com os ganhos padrão (margem 0,35), e o texto diz o que ela garante',
  s.forte.length === 1 && s.forte[0].startsWith('cond forte ok') && /Condição forte/.test(s.sec) && /= 0,35 > 0/.test(s.sec) && /nunca sai de \(−90°, 90°\]/.test(s.sec), s.forte[0] + ' · ' + s.sec);
conferir('exemplo do Corke abre de ré', s.sentido === -1 && /de ré/.test(s.hud) && /ré/.test(s.leit), s.hud.split('\n').slice(0, 3).join(' / '));
conferir('quadro de leitura com espaço depois do "="', /ρ = \d/.test(s.hud) && !/ρ =\d/.test(s.hud), s.hud.split('\n')[1]);

// 1b) (1, 2, −1): as três condições valem, a forte não (margem −0,30); ela fica neutra, sem aviso
await b.avaliar('window.simulador.definirGanhos({ rho: 1, alpha: 2, beta: -1 })');
s = await J(`{ conds: [...document.querySelectorAll('.cond:not(.forte)')].map((c) => c.className), forte: document.querySelector('.cond.forte').className,
  sec: document.getElementById('lei-sec').textContent, instavel: document.getElementById('aviso-instavel').hidden }`);
conferir('(1, 2, −1): três verdes, a forte neutra e o texto diz que α pode sair do intervalo', s.conds.every((c) => c === 'cond ok') && s.forte === 'cond forte neutra' &&
  s.instavel && /= −0,30:/.test(s.sec) && /pode sair de \(−90°, 90°\]/.test(s.sec), s.sec);
await b.avaliar(`document.getElementById('btn-ganhos').click()`);

// 1c) os números dos eixos com passo de 2,5 (0 a 10 s no histórico; grade de 0,25 m no mapa) saem sem
// arredondar: guarda o que cada canvas escreve
await b.avaliar(`(() => {
  window.__textos = {};
  const escrever = CanvasRenderingContext2D.prototype.fillText;
  CanvasRenderingContext2D.prototype.fillText = function (t, x, y, ...resto) {
    (window.__textos[this.canvas.id] ||= []).push({ t: String(t), x, y, a: this.textAlign, w: this.measureText(t).width, largura: this.canvas.clientWidth });
    return escrever.call(this, t, x, y, ...resto);
  };
})()`);
await b.avaliar(`(() => { const S = window.simulador; if (S.est.rodando) document.getElementById('btn-rodar').click(); S.reiniciar(); window.__textos = {}; })()`);
await dormir(300);
s = await J(`[...new Set((window.__textos['serie-rho'] || []).map((e) => e.t).filter((t) => / s$/.test(t)))]`);
conferir('histórico de 0 a 10 s: o eixo do tempo escreve 2,5 e 7,5 s', s.includes('2,5 s') && s.includes('7,5 s') && !s.includes('3 s') && !s.includes('8 s'), s.join(' '));
await b.avaliar(`(() => { const S = window.simulador; S.est.exibir.seguir = false; Object.assign(S.vista, { esc: 300, x: 7.2, y: 5.1, roboVisivel: false }); window.__textos = {}; })()`);
await dormir(300);
s = await J(`[...new Set((window.__textos['mundo'] || []).map((e) => e.t))]`);
conferir('mapa com grade de 0,25 m: valores e escala sem arredondar', s.includes('6,25') && s.includes('6,75') && s.includes('0,25 m') && !s.includes('6,3') && !s.includes('0,3 m'),
  s.filter((t) => /^\d/.test(t)).slice(0, 12).join(' '));
await b.avaliar('window.simulador.enquadrar()');

// 1d) pausado no meio, o botão diz "Continuar"; reiniciado parado, "Iniciar"
await b.avaliar(`document.getElementById('btn-rodar').click()`);
await dormir(400);
await b.avaliar(`document.getElementById('btn-rodar').click()`);
s = await J(`{ botao: document.getElementById('btn-rodar').textContent, t: window.simulador.est.t, rodando: window.simulador.est.rodando }`);
const pausado = s;
await b.avaliar(`document.getElementById('btn-reiniciar').click()`);
s = await J(`document.getElementById('btn-rodar').textContent`);
conferir('pausado no meio: "Continuar"; reiniciado parado: "Iniciar"', pausado.botao === 'Continuar' && !pausado.rodando && pausado.t > 0 && s === 'Iniciar',
  `t = ${pausado.t.toFixed(2)} s: "${pausado.botao}"; depois de reiniciar: "${s}"`);

// 1e) a 1400 px os gráficos ficam em três colunas estreitas; no meio da corrida de ré (v ≈ −0,34 m/s, com
// as legendas mais compridas), nenhum texto dos gráficos passa da borda direita: a legenda desce de linha
const borda = () => J(`Object.entries(window.__textos).filter(([id]) => id.startsWith('serie-')).map(([id, l]) => {
  const fim = Math.max(...l.map((e) => (e.a === 'right' ? e.x : e.a === 'center' ? e.x + e.w / 2 : e.x + e.w)));
  return { id, sobra: Math.round((l[0].largura - fim) * 10) / 10 };
})`);
await b.avaliar(`(() => { const S = window.simulador; for (let i = 0; i < 130; i++) S.passo(); window.__textos = {}; })()`);
await dormir(300);
s = await borda();
conferir('a 1400 px, no meio da corrida de ré: nenhum texto dos gráficos passa da borda', s.length >= 5 && s.every((g) => g.sobra >= 0),
  s.map((g) => `${g.id.slice(6)} ${g.sobra} px`).join(' · '));
await b.foto(SAIDA + 'controle_legendas.png');
await b.avaliar(`(() => { window.simulador.reiniciar(); window.__textos = {}; })()`);
await dormir(300);
s = await borda();
conferir('no começo (eixo do tempo de 0 a 10 s), o "10,0 s" da borda também fica dentro', s.length >= 5 && s.every((g) => g.sobra >= 0),
  s.map((g) => `${g.id.slice(6)} ${g.sobra} px`).join(' · '));

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
