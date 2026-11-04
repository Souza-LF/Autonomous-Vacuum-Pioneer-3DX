// Etapa 6: página de escolha. Quatro cartões com links, legendas iguais às corridas do Node,
// miniaturas desenhadas e animando, "reduzir movimento", clique levando à simulação certa, o link de
// volta e o algoritmo na URL, fundo claro com o sistema escuro, 1000 px e celular.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const midia = (escuro, reduzir) => b.enviar('Emulation.setEmulatedMedia', { features: [
  { name: 'prefers-color-scheme', value: escuro ? 'dark' : 'light' }, { name: 'prefers-reduced-motion', value: reduzir ? 'reduce' : 'no-preference' }] });
// pixels da miniatura diferentes do fundo (para ver que algo foi desenhado)
const tinta = `[...document.querySelectorAll('canvas.miniatura')].map((cv) => { const d = cv.getContext('2d').getImageData(0, 0, cv.width, cv.height).data;
  const f = [d[0], d[1], d[2]]; let n = 0; for (let i = 0; i < d.length; i += 4) if (Math.abs(d[i] - f[0]) + Math.abs(d[i + 1] - f[1]) + Math.abs(d[i + 2] - f[2]) > 30) n++; return n; })`;

await b.tela(1400, 950);
await midia(false, false);

// 1) quatro cartões, na ordem, com os links da simulação
await b.abrir(BASE + 'index.html');
await dormir(800);
let s = await J(`{ cart: [...document.querySelectorAll('.lei-cartao')].map((a) => ({ lei: a.dataset.lei, href: a.getAttribute('href'), h2: a.querySelector('h2').textContent,
  leg: a.querySelector('.legenda-mini').textContent, principal: a.classList.contains('principal') })), titulo: document.title }`);
conferir('quatro cartões (pose, ponto, reta, caminho), cada um abrindo a simulação do seu algoritmo', s.cart.length === 4 &&
  s.cart.map((c) => c.lei).join() === 'pose,ponto,reta,caminho' && s.cart.every((c) => c.href === `simulacao.html?algoritmo=${c.lei}`) &&
  s.cart[0].principal && s.cart.slice(1).every((c) => !c.principal), s.cart.map((c) => c.h2).join(' | '));

// 2) as legendas dão os tempos de simular() com os exemplos do Corke e os ganhos padrão
const ref = await J(`(() => { const K = window.Controladores, G = { pose: K.GANHOS_POSE, ponto: K.GANHOS_PONTO, reta: K.GANHOS_RETA, caminho: K.GANHOS_CAMINHO };
  const r = {}; for (const [lei, ex] of Object.entries(K.EXEMPLOS_CORKE)) { const x = K.simular(Object.assign({ lei, modelo: 'diferencial', k: G[lei] }, ex)); r[lei] = { t: x.tChegada, n: x.R.length }; }
  return r; })()`);
const virgula = (t) => t.toFixed(2).replace('.', ',');
for (const c of s.cart) {
  conferir(`legenda de ${c.lei}: o tempo de simular() (${virgula(ref[c.lei].t)} s)`, c.leg.includes(virgula(ref[c.lei].t) + ' s'), c.leg);
}
conferir('a pose diz "de ré" e o ponto, para onde o robô ficou virado', /de ré/.test(s.cart[0].leg) && /virado para/.test(s.cart[1].leg) && /19,90 m/.test(s.cart[3].leg),
  s.cart[0].leg);

// 3) miniaturas desenhadas e animando
s = await J(tinta);
conferir('as quatro miniaturas têm desenho', s.every((n) => n > 500), s.join(' / '));
const i1 = await J(`window.escolha.cartoes.map((c) => c.i)`);
await dormir(1200);
const i2 = await J(`window.escolha.cartoes.map((c) => c.i)`);
conferir('as miniaturas animam (o passo avança)', i2.every((x, k) => x !== i1[k]), `${i1.join(',')} → ${i2.join(',')}`);
const fatores = await J(`window.escolha.cartoes.map((c) => c.fator)`);
conferir('a volta mais longa (caminho, 55 s) é acelerada; nenhuma fica mais lenta que o tempo real', fatores.every((f) => f >= 1) && fatores[3] > 9,
  fatores.map((f) => f.toFixed(2)).join(' / '));
await b.foto(SAIDA + 'escolha_claro.png');

// 4) reduzir movimento: a trajetória inteira, parada
await midia(false, true);
await dormir(500);
s = await J(`window.escolha.cartoes.map((c) => [c.i, c.s.R.length - 1])`);
await dormir(800);
const s2 = await J(`window.escolha.cartoes.map((c) => c.i)`);
conferir('com "reduzir movimento", cada miniatura mostra a trajetória inteira, parada', s.every(([i, u], k) => i === u && s2[k] === u), JSON.stringify(s));
await midia(false, false);

// 5) clicar no cartão da reta abre a simulação na reta
const pos = await J(`(() => { const r = document.querySelector('[data-lei="reta"] .abrir').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
const carregou = b.enviar('Page.enable');
await carregou;
await b.mouse('mouseMoved', pos[0], pos[1], 0);
await b.mouse('mousePressed', pos[0], pos[1]);
await b.mouse('mouseReleased', pos[0], pos[1], 0);
await dormir(1500);
s = await J(`{ url: location.pathname + location.search, lei: window.simulador && window.simulador.est.lei, voltar: document.querySelector('.voltar')?.getAttribute('href') }`);
conferir('clicar no cartão da reta abre a simulação já na reta, com o link de volta', s.url === '/simulacao.html?algoritmo=reta' && s.lei === 'reta' && s.voltar === 'index.html', JSON.stringify(s));

// 6) trocar o algoritmo na simulação põe o novo na URL, e recarregar cai nele
await b.avaliar(`(() => { const e = document.getElementById('algoritmo'); e.value = 'caminho'; e.dispatchEvent(new Event('change')); })()`);
s = await J(`location.search`);
conferir('trocar para o caminho põe ?algoritmo=caminho na URL', s === '?algoritmo=caminho', s);
await b.abrir(BASE + 'simulacao.html' + s);
s = await J(`window.simulador.est.lei`);
conferir('recarregar abre no caminho', s === 'caminho', s);
await b.foto(SAIDA + 'escolha_simulacao_topo.png', { x: 0, y: 0, width: 1400, height: 140 });

// 7) o link de volta leva à escolha
const pv = await J(`(() => { const r = document.querySelector('.voltar').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; })()`);
await b.mouse('mouseMoved', pv[0], pv[1], 0);
await b.mouse('mousePressed', pv[0], pv[1]);
await b.mouse('mouseReleased', pv[0], pv[1], 0);
await dormir(1200);
s = await J(`{ url: location.pathname, n: document.querySelectorAll('.lei-cartao').length }`);
conferir('"← Os quatro algoritmos" volta à página de escolha', s.url === '/index.html' && s.n === 4, JSON.stringify(s));

// 8) sistema no modo escuro, 1000 px (dois por linha) e celular (um por linha)
await midia(true, false);
await dormir(1500);
const fundo = await b.avaliar(`getComputedStyle(document.body).backgroundColor`);
conferir('sistema no modo escuro: o fundo continua claro', fundo === 'rgb(243, 245, 248)', fundo);
await b.foto(SAIDA + 'escolha_escuro.png');
await b.tela(1000, 900);
await b.abrir(BASE + 'index.html');
await dormir(800);
s = await J(`[...document.querySelectorAll('.lei-cartao')].map((a) => Math.round(a.getBoundingClientRect().top))`);
conferir('a 1000 px, dois cartões por linha', s[0] === s[1] && s[2] === s[3] && s[2] > s[0], s.join(' / '));
await b.foto(SAIDA + 'escolha_1000.png');
await b.tela(390, 844, false, true);
await b.abrir(BASE + 'index.html');
await dormir(800);
s = await J(`{ sw: document.documentElement.scrollWidth, w: window.innerWidth, tops: [...document.querySelectorAll('.lei-cartao')].map((a) => Math.round(a.getBoundingClientRect().top)), tinta: ${tinta} }`);
conferir('390 px: um cartão por linha, sem rolagem horizontal, miniaturas desenhadas', s.sw <= s.w && s.tops.every((t, k) => k === 0 || t > s.tops[k - 1]) && s.tinta.every((n) => n > 300),
  `scrollWidth ${s.sw}, largura ${s.w}, tinta ${s.tinta.join('/')}`);
await b.foto(SAIDA + 'escolha_celular.png');

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));
console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
