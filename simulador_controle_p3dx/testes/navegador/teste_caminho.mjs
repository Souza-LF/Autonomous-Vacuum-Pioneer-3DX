// Etapa 5: caminho (pure pursuit) na página. Abrir pela URL, painel e textos, corridas dos cinco
// caminhos prontos iguais às do Node bit a bit, edição dos waypoints no mapa (acrescentar, arrastar,
// inserir num trecho, apagar com Shift e com o botão direito, mínimo de dois), botões, ganhos,
// odometria no caminho, tema escuro e celular.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const URL = BASE + 'simulacao.html';
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const ateFim = async () => { for (let i = 0; i < 2500 && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(40); };
const ate = async (t) => { for (let i = 0; i < 1500 && (await b.avaliar('window.simulador.est.t')) < t; i++) await dormir(30); };
const campo = (id, v, ev = 'change') => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.value = '${v}'; e.dispatchEvent(new Event('${ev}')); })()`);
const rapido = () => campo('p-escala', '10');
const pausar = () => b.avaliar('window.simulador.est.rodando = false');
const W = () => J('window.simulador.est.caminho');
const visiveis = `[...document.querySelectorAll('[data-leis]')].filter((e) => !e.hidden).map((e) => e.dataset.exemplo || e.id || e.querySelector('h2')?.textContent || e.textContent.trim().slice(0, 20))`;
const contraNode = `(() => { const e = window.simulador.est, K = window.Controladores; const r = {};
  for (const m of ['diferencial', 'incremental', 'exato']) {
    const ref = K.simular(Object.assign({}, e.m[m].o, { P0: e.P0 }));
    r[m] = { t: e.m[m].tChegada, tRef: ref.tChegada, chegou: e.m[m].chegou, iguais: e.m[m].P.every((x, i) => x === ref.P[i]), P: e.m[m].P };
  }
  return r; })()`;
const tela = async (x, y) => J(`(() => { const r = document.getElementById('mundo').getBoundingClientRect(); const p = window.simulador.paraTela(${x}, ${y}); return [r.left + p[0], r.top + p[1]]; })()`);
const clique = async (x, y, { shift = false, direito = false } = {}) => {
  const button = direito ? 'right' : 'left', modifiers = shift ? 8 : 0;
  await b.enviar('Input.dispatchMouseEvent', { type: 'mouseMoved', x, y, button: 'none', buttons: 0, modifiers });
  await b.enviar('Input.dispatchMouseEvent', { type: 'mousePressed', x, y, button, buttons: direito ? 2 : 1, clickCount: 1, modifiers });
  await b.enviar('Input.dispatchMouseEvent', { type: 'mouseReleased', x, y, button, buttons: 0, clickCount: 1, modifiers });
  await dormir(150);
};
const perto = (p, q, tol = 0.02) => Math.abs(p[0] - q[0]) <= tol && Math.abs(p[1] - q[1]) <= tol;

await b.tela(1400, 950);

// 1) abrir já no caminho, pela URL
await b.abrir(URL + '?algoritmo=caminho');
await dormir(500);
let s = await J(`{ lei: window.simulador.est.lei, sel: document.getElementById('algoritmo').value, vis: ${visiveis},
  conds: [...document.querySelectorAll('.cond')].map((c) => c.className + ' ' + c.textContent), sec: document.getElementById('lei-sec').textContent,
  alvoFs: document.getElementById('p-xa').closest('fieldset').hidden, cam: document.getElementById('caminho-sec').textContent,
  rotulo: document.getElementById('rotulo-alvo').textContent, dica: document.getElementById('dica-tela').textContent,
  P0: window.simulador.est.P0, W: window.simulador.est.caminho, k: window.simulador.est.k }`);
conferir('?algoritmo=caminho abre o caminho do Corke, sem o alvo e com os botões dos caminhos prontos', s.lei === 'caminho' && s.sel === 'caminho' && s.alvoFs &&
  ['cam-corke', 'cam-zigue', 'cam-quadrado', 'cam-oito', 'cam-esse'].every((n) => s.vis.includes(n)) && !s.vis.includes('corke') && !s.vis.includes('reta-corke') &&
  s.W.length === 4 && s.W[3][0] === 5 && s.P0[0] === 0, s.vis.join(' | '));
conferir('caminho: d, Kh e v padrão, 3 condições verdes e "Editar caminho"', s.k.d === 0.5 && s.k.h === 2 && s.k.v === 0.4 && s.conds.length === 3 &&
  s.conds.every((c) => c.startsWith('cond ok')) && s.rotulo === 'Editar caminho' && /waypoint/.test(s.dica), s.conds.join(' | '));
conferir('caminho: o texto dá Kv = v/d = 0,80 e Kh > Kv; o quadro dá 4 waypoints e 19,90 m', /Kv = v\/d = 0,80/.test(s.sec) && /Kh > Kv/.test(s.sec) && /0,20 m/.test(s.sec) &&
  /4 waypoints, 19,90 m/.test(s.cam) && /49,7 s\.$/.test(s.cam), s.sec + ' · ' + s.cam);
await ate(25);
await pausar();
await dormir(300);
s = await J(`{ j: window.simulador.est.m.diferencial.e.j, leit: document.getElementById('leituras').innerText, estado: document.querySelector('#leituras .estado').textContent,
  serie: document.getElementById('serie-rho').getAttribute('aria-label') }`);
conferir('no meio da corrida, j avançou e o painel mostra desvio, α rumo a p_k e o andado', s.j > 100 && /andado/.test(s.leit) && /\d+,\d\d \/ 19,90 m/.test(s.leit) && /rumo a pk/.test(s.leit) &&
  s.estado === 'Seguindo o caminho', `j = ${s.j} · ${s.estado}`);
await b.foto(SAIDA + 'controle_caminho_meio.png');
await b.avaliar('window.simulador.est.rodando = true');

// 2) corrida do Corke igual à do Node
await rapido();
await ateFim();
s = await J(contraNode);
for (const m of ['diferencial', 'incremental', 'exato']) {
  conferir(`caminho do Corke, ${m}: chegada e pose final iguais às de simular() bit a bit`, s[m].chegou && s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9,
    `t = ${s[m].t?.toFixed(2)} s`);
}
s = await J(`{ hud: document.getElementById('hud').innerText }`);
conferir('o HUD diz "chegou ao fim" e para onde o robô ficou virado', /chegou ao fim em \d+,\d\d s/.test(s.hud) && /chegou virado para/.test(s.hud), s.hud.split('\n').slice(1, 2).join(''));
await b.foto(SAIDA + 'controle_caminho_fim.png');

// 3) os outros caminhos prontos
for (const nome of ['zigue', 'quadrado', 'oito', 'esse']) {
  await b.avaliar(`document.querySelector('[data-exemplo="cam-${nome}"]').click()`);
  await rapido();
  await ateFim();
  s = await J(contraNode);
  const ok = ['diferencial', 'incremental', 'exato'].every((m) => s[m].chegou && s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9);
  conferir(`${nome}: os três modelos chegam ao fim, iguais a simular() bit a bit`, ok,
    ['diferencial', 'incremental', 'exato'].map((m) => `${m.slice(0, 3)} ${s[m].t?.toFixed(2)} s`).join(', '));
  if (nome === 'oito') await b.foto(SAIDA + 'controle_caminho_oito.png');
}

// 4) edição no mapa: recomeçar do início dá dois waypoints a partir da pose inicial
await b.avaliar(`document.querySelector('[data-exemplo="cam-corke"]').click()`);
await dormir(200);
await b.avaliar(`document.getElementById('btn-cam-limpar').click()`);
await pausar();
let w = await W();
s = await J(`{ des: document.getElementById('btn-cam-desfazer').disabled, cam: document.getElementById('caminho-sec').textContent }`);
conferir('"Recomeçar do início": (0, 0) e 1 m à frente; "Apagar o último ponto" desligado', w.length === 2 && perto(w[0], [0, 0], 0) && perto(w[1], [1, 0], 0) && s.des && /2 waypoints, 1,00 m/.test(s.cam), s.cam);
let [px, py] = await tela(3, 2);
await clique(px, py);
await pausar();
w = await W();
conferir('clique longe de tudo acrescenta um waypoint no fim', w.length === 3 && perto(w[2], [3, 2]), JSON.stringify(w));
[px, py] = await tela(1, 0);
let [qx, qy] = await tela(1, -1);
await b.arrastar(px, py, qx, qy, 8);
await pausar();
w = await W();
conferir('arrastar um waypoint o move (no centímetro)', w.length === 3 && perto(w[1], [1, -1]) && w[1].every((x) => Math.abs(x * 100 - Math.round(x * 100)) < 1e-9), JSON.stringify(w));
[px, py] = await tela(2, 0.5);
[qx, qy] = await tela(2, -0.5);
await b.arrastar(px, py, qx, qy, 8);
await pausar();
w = await W();
conferir('arrastar a partir de um trecho insere um waypoint nele', w.length === 4 && perto(w[2], [2, -0.5]) && perto(w[3], [3, 2]), JSON.stringify(w));
[px, py] = await tela(...w[2]);
await clique(px, py, { shift: true });
await pausar();
w = await W();
conferir('Shift+clique apaga o waypoint', w.length === 3 && perto(w[1], [1, -1]) && perto(w[2], [3, 2]), JSON.stringify(w));
[px, py] = await tela(...w[1]);
await clique(px, py, { direito: true });
await pausar();
w = await W();
conferir('botão direito apaga o waypoint', w.length === 2 && perto(w[1], [3, 2]), JSON.stringify(w));
[px, py] = await tela(...w[1]);
await clique(px, py, { direito: true });
await pausar();
w = await W();
conferir('com dois waypoints, apagar não faz nada', w.length === 2, JSON.stringify(w));
// clique num waypoint sem arrastar: não recomeça a corrida
await b.avaliar('window.simulador.est.rodando = true');
await ate(1);
await pausar();
const t1 = await b.avaliar('window.simulador.est.t');
[px, py] = await tela(...w[1]);
await clique(px, py);
s = await J(`{ t: window.simulador.est.t, n: window.simulador.est.caminho.length }`);
conferir('clique num waypoint sem arrastar não muda o caminho nem recomeça', s.n === 2 && s.t >= t1, `t ${t1.toFixed(2)} → ${s.t.toFixed(2)} s`);
// dois pontos a mais e o "Apagar o último ponto"
for (const [x, y] of [[4, 0], [5, 1]]) { [px, py] = await tela(x, y); await clique(px, py); await pausar(); }
await b.avaliar(`document.getElementById('btn-cam-desfazer').click()`);
await pausar();
w = await W();
conferir('"Apagar o último ponto" tira o (5, 1) e deixa o (4, 0)', w.length === 3 && perto(w[2], [4, 0]), JSON.stringify(w));
await b.foto(SAIDA + 'controle_caminho_editado.png');
await b.avaliar('window.simulador.est.rodando = true');
await rapido();
await ateFim();
s = await J(contraNode);
conferir('o caminho editado corre igual a simular() bit a bit', ['diferencial', 'incremental', 'exato'].every((m) => s[m].chegou && s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9),
  ['diferencial', 'incremental', 'exato'].map((m) => `${m.slice(0, 3)} ${s[m].t?.toFixed(2)} s (iguais ${s[m].iguais}, Node ${s[m].tRef})`).join(', '));

// 5) ganhos: Kh < Kv, Kh = 0 e v pequeno
await b.avaliar(`document.querySelector('[data-exemplo="cam-corke"]').click()`);
await campo('k-ch-num', '0.5', 'input');
s = await J(`{ conds: [...document.querySelectorAll('.cond')].map((c) => c.className), sec: document.getElementById('lei-sec').textContent, aviso: document.getElementById('aviso-instavel').hidden }`);
conferir('Kh = 0,5 < Kv = 0,8: condições verdes, mas o texto avisa que α cresce no fim', s.conds.every((c) => c === 'cond ok') && s.aviso && /Kh < Kv/.test(s.sec), s.sec);
await campo('k-ch-num', '0', 'input');
s = await J(`{ conds: [...document.querySelectorAll('.cond')].map((c) => c.className), aviso: document.getElementById('aviso-instavel').hidden, txt: document.getElementById('aviso-instavel').textContent }`);
conferir('Kh = 0: "Kh > 0" vermelho e o aviso', s.conds[1] === 'cond ruim' && !s.aviso && /seguir o caminho/.test(s.txt), s.conds.join(' '));
await b.avaliar(`document.getElementById('btn-ganhos').click()`);
await campo('k-cv-num', '0.1', 'input');
s = await J(`document.getElementById('caminho-sec').textContent`);
conferir('v = 0,1 m/s: o quadro avisa que o percurso passa do tempo máximo', /199,0 s, mais que o tempo máximo de 60 s/.test(s), s);
await b.avaliar(`document.getElementById('btn-ganhos').click()`);
s = await J(`{ k: window.simulador.est.k, cam: document.getElementById('caminho-sec').textContent }`);
conferir('"Ganhos padrão" volta d, Kh e v e o quadro', s.k.d === 0.5 && s.k.h === 2 && s.k.v === 0.4 && /49,7 s\.$/.test(s.cam), s.cam);

// 6) odometria no caminho do Corke, com a direita 2 mm maior
await b.avaliar(`document.getElementById('btn-odo-exemplo').click()`);
await rapido();
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores, so = e.m.odometria;
  const ref = K.simularOdometria(Object.assign({}, so.o, { P0: e.P0 }));
  const h = e.hist[e.hist.length - 1].por.odometria;
  return { lei: e.lei, ativo: e.odo.ativo, iguais: so.real.every((x, i) => x === ref.real[i]) && so.P.every((x, i) => x === ref.odo[i]),
    t: so.tChegada, tRef: ref.tChegada, dReal: h.dReal, hud: document.getElementById('hud').innerText }; })()`);
conferir('odometria no caminho: igual a simularOdometria() bit a bit, e o HUD dá a distância do robô real ao fim', s.lei === 'caminho' && s.ativo && s.iguais &&
  Math.abs(s.t - s.tRef) < 1e-9 && /a estimativa chegou ao fim em 55,25 s/.test(s.hud) && /robô real a [\d.]+ mm do fim do caminho/.test(s.hud) && s.dReal > 0.1,
  s.hud.split('\n').filter((l) => /odometria|real/.test(l)).join(' / ') + ` · desvio real do caminho ${s.dReal?.toFixed(3)} m`);
await b.foto(SAIDA + 'controle_caminho_odo.png');
await b.avaliar(`(() => { const e = document.getElementById('odo-ativo'); e.checked = false; e.dispatchEvent(new Event('change')); })()`);

// 7) do caminho para o ponto e de volta
await b.avaliar(`(() => { const e = document.getElementById('algoritmo'); e.value = 'ponto'; e.dispatchEvent(new Event('change')); })()`);
await dormir(200);
s = await J(`{ lei: window.simulador.est.lei, vis: ${visiveis}, alvo: window.simulador.est.alvo, rot: document.getElementById('rotulo-alvo').textContent }`);
conferir('seletor → ponto: volta o alvo e o exemplo da Fig. 4.8', s.lei === 'ponto' && s.alvo[0] === 5 && s.rot === 'Marcar alvo' && !s.vis.includes('cam-corke'), s.vis.join(' | '));
await b.avaliar(`(() => { const e = document.getElementById('algoritmo'); e.value = 'caminho'; e.dispatchEvent(new Event('change')); })()`);
await dormir(200);
s = await J(`{ lei: window.simulador.est.lei, n: window.simulador.est.caminho.length }`);
conferir('seletor → caminho: abre o do Corke', s.lei === 'caminho' && s.n === 4, JSON.stringify(s));

// 8) tema escuro e celular
await b.tela(1400, 950, true);
await b.abrir(URL + '?algoritmo=caminho');
await dormir(2500);
await b.foto(SAIDA + 'controle_caminho_escuro.png');
await b.tela(390, 844, false, true);
await b.abrir(URL + '?algoritmo=caminho');
await dormir(800);
s = await J(`{ sw: document.documentElement.scrollWidth, w: window.innerWidth }`);
conferir('390 px no caminho: sem rolagem horizontal', s.sw <= s.w, `scrollWidth ${s.sw}, largura ${s.w}`);
await b.foto(SAIDA + 'controle_caminho_celular.png');

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));
console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
