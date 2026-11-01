// Etapa 4: ponto e reta na página. Abrir pela URL, painel de cada lei, corridas iguais às do Node
// bit a bit, condições e textos (espiral, ζ), reta marcada no mapa, odometria na reta, fotos.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const URL = BASE + 'simulacao.html';
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const ateFim = async () => { for (let i = 0; i < 1500 && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(40); };
const ate = async (t) => { for (let i = 0; i < 1500 && (await b.avaliar('window.simulador.est.t')) < t; i++) await dormir(30); };
const campo = (id, v, ev = 'change') => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.value = '${v}'; e.dispatchEvent(new Event('${ev}')); })()`);
const rapido = () => campo('p-escala', '10');
const visiveis = `[...document.querySelectorAll('[data-leis]')].filter((e) => !e.hidden).map((e) => e.dataset.exemplo || e.id || e.querySelector('h2')?.textContent || e.textContent.trim().slice(0, 20))`;
// a corrida da tela contra simular() com as mesmas opções
const contraNode = `(() => { const e = window.simulador.est, K = window.Controladores; const r = {};
  for (const m of ['diferencial', 'incremental', 'exato']) {
    const ref = K.simular(Object.assign({}, e.m[m].o, { P0: e.P0 }));
    r[m] = { t: e.m[m].tChegada, tRef: ref.tChegada, iguais: e.m[m].P.every((x, i) => x === ref.P[i]), P: e.m[m].P };
  }
  return r; })()`;

await b.tela(1400, 950);

// 1) abrir já no ponto, pela URL
await b.abrir(URL + '?algoritmo=ponto');
await dormir(500);
let s = await J(`{ lei: window.simulador.est.lei, sel: document.getElementById('algoritmo').value, vis: ${visiveis},
  conds: [...document.querySelectorAll('.cond')].map((c) => c.className + ' ' + c.textContent), sec: document.getElementById('lei-sec').textContent,
  th: document.getElementById('p-tha').closest('label').hidden, tol: document.getElementById('tol-th').closest('.campo').hidden,
  rotulo: document.getElementById('rotulo-alvo').textContent, P0: window.simulador.est.P0, alvo: window.simulador.est.alvo }`);
conferir('?algoritmo=ponto abre no ponto, com os ganhos do ponto e sem θ*', s.lei === 'ponto' && s.sel === 'ponto' && s.th && s.tol &&
  s.vis.includes('ponto-corke') && !s.vis.includes('corke') && !s.vis.includes('reta-corke') && s.P0[0] === 8 && s.alvo[0] === 5, s.vis.join(' | '));
conferir('ponto: Kv > 0 e Kh > Kv verdes, e o texto diz que α vai a zero', s.conds.length === 2 && s.conds.every((c) => c.startsWith('cond ok')) && /α vai a zero/.test(s.sec),
  s.conds.join(' | ') + ' · ' + s.sec);
await ate(4);
await b.avaliar('window.simulador.est.rodando = false');
await dormir(300);
await b.foto(SAIDA + 'controle_ponto_meio.png');
await b.avaliar('window.simulador.est.rodando = true');

// 2) corrida do exemplo do Corke (Fig. 4.8) igual à do Node
await rapido();
await ateFim();
s = await J(contraNode);
for (const m of ['diferencial', 'incremental', 'exato']) {
  conferir(`ponto, ${m}: chegada e pose final iguais às de simular() bit a bit`, s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9,
    `t = ${s[m].t?.toFixed(2)} s, θ final ${(s[m].P[2] * 180 / Math.PI).toFixed(1)}°`);
}
s = await J(`{ hud: document.getElementById('hud').innerText, leit: document.getElementById('leituras').innerText }`);
conferir('ponto: o HUD diz quando chegou e para onde ficou virado', /chegou em 14,20 s/.test(s.hud) && /chegou virado para/.test(s.hud) && /α/.test(s.leit) && !/β/.test(s.leit),
  s.hud.split('\n').slice(1, 2).join(' / '));
await b.foto(SAIDA + 'controle_ponto_fim.png');

// 3) Kh < Kv: condição vermelha, aviso e o α de equilíbrio da espiral
await campo('k-pv-num', '1', 'input');
await campo('k-ph-num', '0.8', 'input');
s = await J(`{ conds: [...document.querySelectorAll('.cond')].map((c) => c.className), aviso: document.getElementById('aviso-instavel').hidden,
  sec: document.getElementById('lei-sec').textContent, k: window.simulador.est.k }`);
conferir('ponto, Kh/Kv = 0,8: "Kh > Kv" vermelho, aviso, e α para em 64,8° (espiral)', s.conds[1] === 'cond ruim' && !s.aviso && /64,8°/.test(s.sec) && /espiral/.test(s.sec) && s.k.v === 1 && s.k.h === 0.8,
  s.sec);
await campo('k-ph-num', '0.6', 'input');
s = await J(`document.getElementById('lei-sec').textContent`);
conferir('ponto, Kh/Kv = 0,6: o texto diz que a espiral é para fora', /95,1°/.test(s) && /se afasta/.test(s), s);
await b.avaliar(`document.getElementById('btn-ganhos').click()`);

// 4) trocar para a reta pelo seletor: abre o exemplo da Fig. 4.10
await b.avaliar(`(() => { const e = document.getElementById('algoritmo'); e.value = 'reta'; e.dispatchEvent(new Event('change')); })()`);
await dormir(300);
s = await J(`{ lei: window.simulador.est.lei, vis: ${visiveis}, conds: [...document.querySelectorAll('.cond')].map((c) => c.className + ' ' + c.textContent),
  sec: document.getElementById('lei-sec').textContent, abc: document.getElementById('reta-abc').textContent, rotulo: document.getElementById('rotulo-alvo').textContent,
  legenda: document.getElementById('legenda-alvo').textContent, dica: document.getElementById('dica-tela').textContent, alvo: window.simulador.est.alvo, P0: window.simulador.est.P0 }`);
conferir('seletor → reta: painel da reta, 3 condições verdes e ζ = 1', s.lei === 'reta' && s.conds.length === 3 && s.conds.every((c) => c.startsWith('cond ok')) &&
  /ζ = 1,00/.test(s.sec) && /crítico/.test(s.sec) && s.vis.includes('reta-corke') && !s.vis.includes('ponto-corke'), s.sec);
conferir('reta: (a, b, c) = (1, −2, 4)/√5, "Marcar reta" e a dica do mapa', /a = 0,447, b = −0,894, c = 1,789/.test(s.abc) && s.rotulo === 'Marcar reta' && /sentido do percurso/.test(s.dica) && /direção/.test(s.legenda),
  s.abc);
await ate(3);
await b.avaliar('window.simulador.est.rodando = false');
await dormir(300);
await b.foto(SAIDA + 'controle_reta_meio.png');
await b.avaliar('window.simulador.est.rodando = true');

// 5) corrida da reta igual à do Node, e o mesmo tempo da reta (1, −2, 4) sem normalizar
await rapido();
await ateFim();
s = await J(contraNode);
for (const m of ['diferencial', 'incremental', 'exato']) {
  conferir(`reta, ${m}: convergência e pose final iguais às de simular() bit a bit`, s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9,
    `t = ${s[m].t?.toFixed(2)} s, θ final ${(s[m].P[2] * 180 / Math.PI).toFixed(2)}°`);
}
s = await J(`(() => { const K = window.Controladores, e = window.simulador.est;
  const r = K.simular({ lei: 'reta', reta: [1, -2, 4], k: K.GANHOS_RETA, P0: [8, 5, Math.PI / 2] });
  return { t: e.m.diferencial.tChegada, tCorke: r.tChegada, hud: document.getElementById('hud').innerText }; })()`);
conferir('reta: o mesmo tempo que a reta (1, −2, 4) do Node, e o HUD diz "convergiu"', Math.abs(s.t - s.tCorke) < 1e-9 && /convergiu em 7,65 s/.test(s.hud),
  `${s.t?.toFixed(2)} s × ${s.tCorke?.toFixed(2)} s`);
await b.foto(SAIDA + 'controle_reta_fim.png');

// 6) ζ < 1: o texto avisa o sobressinal
await campo('k-rh-num', '0.3', 'input');
s = await J(`document.getElementById('lei-sec').textContent`);
conferir('reta, Kh = 0,3: subamortecido, com o sobressinal da teoria', /ζ = 0,30/.test(s) && /37,2%/.test(s), s);
await b.avaliar(`document.getElementById('btn-ganhos').click()`);

// 7) o outro lado da reta: d começa negativo e o painel diz "à esquerda"
await b.avaliar(`document.querySelector('[data-exemplo="reta-acima"]').click()`);
await dormir(200);
s = await J(`{ d: window.simulador.est.m.diferencial.u.d, estado: document.querySelector('#leituras .estado').textContent }`);
conferir('reta, partida de (2, 5): d < 0 e "à esquerda da reta"', s.d < 0 && /esquerda/.test(s.estado), `d = ${s.d.toFixed(3)} m · ${s.estado}`);

// 8) marcar a reta no mapa: arrastar para a direita dá uma reta horizontal rumo a +x
const tela = async (x, y) => J(`(() => { const r = document.getElementById('mundo').getBoundingClientRect(); const p = window.simulador.paraTela(${x}, ${y}); return [r.left + p[0], r.top + p[1]]; })()`);
let [px, py] = await tela(4, 3);
await b.arrastar(px, py, px + 90, py, 8);
await dormir(200);
s = await J(`{ alvo: window.simulador.est.alvo, abc: document.getElementById('reta-abc').textContent, reta: window.simulador.est.m.diferencial.e.reta }`);
conferir('arrastar no mapa marca a reta y = 3 rumo a +x', Math.abs(s.alvo[1] - 3) < 0.03 && s.alvo[2] === 0 && Math.abs(s.reta[0]) < 1e-12 && s.reta[1] === -1 && /a = 0,000, b = −1,000/.test(s.abc),
  s.abc);

// 9) odometria na reta: igual a simularOdometria() e o HUD diz de que lado da reta ficou o robô real
await b.avaliar(`document.getElementById('btn-odo-exemplo').click()`);
await rapido();
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores, so = e.m.odometria;
  const ref = K.simularOdometria(Object.assign({}, so.o, { P0: e.P0 }));
  return { lei: e.lei, ativo: e.odo.ativo, iguais: so.real.every((x, i) => x === ref.real[i]) && so.P.every((x, i) => x === ref.odo[i]),
    t: so.tChegada, tRef: ref.tChegada, hud: document.getElementById('hud').innerText }; })()`);
conferir('odometria na reta: igual a simularOdometria() bit a bit, e o HUD diz o lado do robô real', s.lei === 'reta' && s.ativo && s.iguais && Math.abs(s.t - s.tRef) < 1e-9 && /robô real a \d+ mm à (direita|esquerda) da reta/.test(s.hud),
  s.hud.split('\n').filter((l) => /odometria|real/.test(l)).join(' / '));
await b.foto(SAIDA + 'controle_reta_odo.png');
await b.avaliar(`(() => { const e = document.getElementById('odo-ativo'); e.checked = false; e.dispatchEvent(new Event('change')); })()`);

// 10) de volta à pose
await b.avaliar(`(() => { const e = document.getElementById('algoritmo'); e.value = 'pose'; e.dispatchEvent(new Event('change')); })()`);
await dormir(200);
s = await J(`{ lei: window.simulador.est.lei, vis: ${visiveis}, sentido: window.simulador.est.m.diferencial.e.sentido, conds: document.querySelectorAll('.cond').length, re: !document.getElementById('permitir-re').closest('.marcar').hidden }`);
conferir('seletor → pose: volta o exemplo do Corke de ré, 3 condições e a ré', s.lei === 'pose' && s.sentido === -1 && s.conds === 3 && s.re && s.vis.includes('corke'), s.vis.join(' | '));

// 11) tema escuro na reta e celular
await b.abrir(URL + '?algoritmo=reta');
await b.tela(1400, 950, true);
await dormir(2500);
await b.foto(SAIDA + 'controle_reta_escuro.png');
await b.tela(390, 844, false, true);
await b.abrir(URL + '?algoritmo=ponto');
await dormir(800);
s = await J(`{ sw: document.documentElement.scrollWidth, w: window.innerWidth }`);
conferir('390 px no ponto: sem rolagem horizontal', s.sw <= s.w, `scrollWidth ${s.sw}, largura ${s.w}`);
await b.foto(SAIDA + 'controle_ponto_celular.png');

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));
console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
