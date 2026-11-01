// Etapa 3: malha pela odometria na página. Exemplo (direita 2 mm maior) igual ao simularOdometria() do
// Node, malhas com a pose verdadeira chegando apesar do erro, gráfico do desvio, escorregamento, fotos.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const URL = BASE + 'simulacao.html';
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const ateFim = async () => { for (let i = 0; i < 1500 && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(40); };
const campo = (id, v, ev = 'change') => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.value = '${v}'; e.dispatchEvent(new Event('${ev}')); })()`);
const ate = async (t) => { for (let i = 0; i < 1500 && (await b.avaliar('window.simulador.est.t')) < t; i++) await dormir(30); };

await b.tela(1400, 950);
await b.abrir(URL);
await dormir(400);
let s = await J(`{ oculto: document.getElementById('serie-odo').hidden, sec: document.getElementById('odo-sec').textContent }`);
conferir('abre sem a odometria: gráfico do desvio escondido', s.oculto && /500 pulsos/.test(s.sec) && /0,720°/.test(s.sec) && /1,23 mm/.test(s.sec), s.sec);

// 1) exemplo: direita 2 mm maior, pela odometria; foto no meio da corrida
await b.avaliar(`document.getElementById('btn-odo-exemplo').click()`);
await ate(9);
await b.avaliar('window.simulador.est.rodando = false');
await dormir(300);
await b.foto(SAIDA + 'controle_odo_meio.png');
await b.avaliar('window.simulador.est.rodando = true');
await campo('p-escala', '10');
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores, so = e.m.odometria;
  const ref = K.simularOdometria(Object.assign({}, so.o, { P0: e.P0 }));
  return { ativo: e.odo.ativo, dDR: e.odo.dDR, chegou: so.chegou, t: so.tChegada, tRef: ref.tChegada,
    realIgual: so.real.every((x, i) => x === ref.real[i]), odoIgual: so.P.every((x, i) => x === ref.odo[i]),
    erro: K.erroAoAlvo(so.real, e.alvo), hud: document.getElementById('hud').innerText,
    leit: document.getElementById('leituras').innerText, oculto: document.getElementById('serie-odo').hidden,
    outras: ['diferencial', 'incremental', 'exato'].map((m) => [m, e.m[m].chegou, K.erroAoAlvo(e.m[m].P, e.alvo).d]) }; })()`);
conferir('exemplo: liga a odometria com a direita 2 mm maior', s.ativo && Math.abs(s.dDR - 0.002) < 1e-12 && !s.oculto);
conferir('odometria na tela igual a simularOdometria() bit a bit', s.realIgual && s.odoIgual && Math.abs(s.t - s.tRef) < 1e-9,
  `estimativa chegou em ${s.t?.toFixed(2)} s (Node ${s.tRef?.toFixed(2)}); robô real a ${(s.erro.d * 1000).toFixed(1)} mm e ${(s.erro.th * 180 / Math.PI).toFixed(2)}°`);
conferir('HUD e leituras dizem onde ficou o robô real', /a estimativa chegou em 17,30 s/.test(s.hud) && /robô real a 240 mm/.test(s.hud), s.hud.split('\n').filter((l) => /odometria|real/.test(l)).join(' / '));
conferir('com a pose verdadeira, as três malhas chegam apesar da roda maior', s.outras.every(([, c, d]) => c && d < 0.01),
  s.outras.map(([m, c, d]) => `${m} ${c ? 'chegou' : 'NÃO'} a ${(d * 1000).toFixed(1)} mm`).join(', '));
await b.foto(SAIDA + 'controle_odo_fim.png');

// 2) escorregamento de 5% na esquerda, sem erro de diâmetro
await campo('err-dr', '0');
await campo('esc-l', '5');
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores, so = e.m.odometria;
  return { esc: e.odo.escL, erro: K.erroAoAlvo(so.real, e.alvo).d, ref: K.erroAoAlvo(K.simularOdometria(Object.assign({}, so.o, { P0: e.P0 })).real, e.alvo).d }; })()`);
conferir('escorregamento de 5% na esquerda: o robô real fica a mais de 1 m', s.esc === 0.05 && s.erro > 1 && s.erro === s.ref, `${(s.erro * 1000).toFixed(1)} mm`);
await campo('esc-l', '0');

// 3) desligar a odometria esconde o gráfico e a malha some do HUD
await b.avaliar(`(() => { const e = document.getElementById('odo-ativo'); e.checked = false; e.dispatchEvent(new Event('change')); })()`);
await dormir(200);
s = await J(`{ oculto: document.getElementById('serie-odo').hidden, hud: document.getElementById('hud').innerText }`);
conferir('desligar a odometria esconde o gráfico e a linha do HUD', s.oculto && !/odometria/.test(s.hud));

// 4) só a odometria marcada (sem os modelos): o painel "Agora" segue a estimativa
await b.avaliar(`document.getElementById('btn-odo-exemplo').click()`);
for (const id of ['modo-dif', 'modo-inc']) await b.avaliar(`(() => { const e = document.getElementById('${id}'); e.checked = false; e.dispatchEvent(new Event('change')); })()`);
await ateFim();
s = await J(`{ agora: document.getElementById('agora-modelo').textContent, estado: document.querySelector('#leituras .estado').textContent }`);
conferir('só a odometria: o painel "Agora" segue a estimativa', s.agora === 'odometria' && /estimativa chegou/.test(s.estado), s.estado);
await b.tela(1400, 950, true);
await dormir(400);
await b.foto(SAIDA + 'controle_odo_escuro.png');

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));
console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
