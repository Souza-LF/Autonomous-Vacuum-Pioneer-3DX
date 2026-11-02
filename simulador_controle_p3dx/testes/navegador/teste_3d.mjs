// Vista 3D: liga e desliga pela aba sem mudar a corrida (bit a bit com simular() com a vista aberta),
// robôs, rodas e rastros nas poses do estado, o que aparece igual ao mapa (modelos, arco exato,
// odometria com os diâmetros reais), as marcas de cada algoritmo, as letras dos eixos que saem da frente
// de outro rótulo, câmera de cima, enquadrar e seguir,
// tema escuro, #3d na URL, celular e o aviso quando a página é aberta como arquivo ou quando o three.js
// não chega. Fotos em fotos/3d_*.
import { conectar, dormir, erros, BASE, FOTOS } from './cdp.mjs';

const SAIDA = FOTOS;
const URL = BASE + 'simulacao.html';
const b = await conectar();
let falhas = 0;
const conferir = (nome, ok, extra = '') => { console.log((ok ? 'OK   ' : 'FALHA') + ' ' + nome + (extra ? ' — ' + extra : '')); if (!ok) falhas++; };
const J = (expr) => b.avaliar(`JSON.stringify(${expr})`).then(JSON.parse);
const ateFim = async () => { for (let i = 0; i < 1500 && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(40); };
const ate = async (t) => { for (let i = 0; i < 1500 && (await b.avaliar('window.simulador.est.t')) < t && !(await b.avaliar('window.simulador.est.fim')); i++) await dormir(30); };
const pausar = async () => { await b.avaliar(`(() => { if (window.simulador.est.rodando) document.getElementById('btn-rodar').click(); })()`); await dormir(400); };
const campo = (id, v, ev = 'change') => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.value = '${v}'; e.dispatchEvent(new Event('${ev}')); })()`);
const marcar = (id, v) => b.avaliar(`(() => { const e = document.getElementById('${id}'); e.checked = ${v}; e.dispatchEvent(new Event('change')); })()`);
const vista = (modo) => b.avaliar(`(() => { const e = document.querySelector('input[name="vista"][value="${modo}"]'); e.checked = true; e.dispatchEvent(new Event('change')); })()`);
const estado = () => J('window.vista3d.estado()');
const elevacao = (c) => (Math.atan2(c.posicao[2] - c.alvo[2], Math.hypot(c.posicao[0] - c.alvo[0], c.posicao[1] - c.alvo[1])) * 180) / Math.PI;
// as letras x e y dos eixos à vista, e as que cobrem a caixa do texto de outro rótulo na tela
const letras = () => J(`(() => {
  const caixa = (e) => { const r = document.createRange(); r.selectNodeContents(e); return r.getBoundingClientRect(); };
  const todos = [...document.querySelectorAll('#cena3d .rotulo-3d')];
  const letras = todos.filter((e) => e.textContent === 'x' || e.textContent === 'y');
  const outros = todos.filter((e) => !letras.includes(e) && e.style.display !== 'none').map(caixa);
  const vistas = letras.filter((e) => e.style.display !== 'none' && e.style.visibility !== 'hidden');
  const cobrem = vistas.filter((e) => { const a = caixa(e); return outros.some((b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom); });
  return { vistas: vistas.map((e) => e.textContent), cobrem: cobrem.map((e) => e.textContent) };
})()`);
const luz = (hex) => [1, 3, 5].reduce((s, i) => s + parseInt(hex.slice(i, i + 2), 16), 0);

await b.tela(1400, 950);
await b.abrir(URL);
await dormir(500);

// 1) a página abre no mapa 2D: o módulo da vista 3D carregou, mas não montou nada
let s = await J(`{ oculto: document.getElementById('cena3d').hidden, e: window.vista3d && window.vista3d.estado(), hash: location.hash }`);
conferir('abre no mapa 2D, com a vista 3D carregada e parada', s.oculto && s.e && !s.e.ativa && s.e.tela === null && s.hash === '');

// 2) a aba "Vista 3D": a área aparece do tamanho do mapa, as ferramentas do mapa saem, entram os botões
// e a dica da vista 3D, e a URL ganha #3d
await vista('3d');
await dormir(700);
s = await J(`{ e: window.vista3d.estado(), oculto: document.getElementById('cena3d').hidden,
  area: [document.getElementById('cena3d').clientWidth, document.getElementById('cena3d').clientHeight],
  mapa: [document.getElementById('mundo').clientWidth, document.getElementById('mundo').clientHeight],
  ferramentas: getComputedStyle(document.getElementById('ferramentas-2d')).display,
  zoom2d: document.getElementById('zoom-2d').hidden, zoom3d: document.getElementById('zoom-3d').hidden,
  dica: document.getElementById('dica-tela').textContent, hash: location.hash, hud: document.getElementById('hud').innerText }`);
conferir('a aba liga a vista 3D, do tamanho do mapa', s.e.ativa && !s.oculto && s.area[0] === s.mapa[0] && s.area[1] === s.mapa[1] && s.e.tela[0] > 0,
  `${s.area.join(' × ')} px, canvas ${s.e.tela.join(' × ')}`);
conferir('as ferramentas do mapa saem; entram "De cima", "Enquadrar" e a dica da vista 3D',
  s.ferramentas === 'none' && s.zoom2d && !s.zoom3d && /Arraste para girar/.test(s.dica));
conferir('#3d na URL e o quadro de leitura por cima', s.hash === '#3d' && /t = /.test(s.hud));
const q0 = (await estado()).quadros;
await dormir(500);
const q1 = (await estado()).quadros;
conferir('a vista 3D desenha a cada quadro', q1 > q0 + 5, `${q1 - q0} quadros em 0,5 s`);

// 3) com a vista 3D aberta, a corrida do exemplo do Corke é a mesma de simular(), bit a bit
await campo('p-escala', '10');
await b.avaliar('window.simulador.recomecar()');
await ateFim();
s = await J(`(() => { const e = window.simulador.est, K = window.Controladores; const r = {};
  for (const m of ['diferencial', 'incremental', 'exato']) {
    const ref = K.simular(Object.assign({}, e.m[m].o, { P0: e.P0 }));
    r[m] = { t: e.m[m].tChegada, tRef: ref.tChegada, iguais: e.m[m].P.every((x, i) => x === ref.P[i]) };
  }
  return r; })()`);
for (const m of ['diferencial', 'incremental', 'exato']) {
  conferir(`${m}: com a vista 3D aberta, chegada e pose final iguais às de simular()`, s[m].iguais && Math.abs(s[m].t - s[m].tRef) < 1e-9,
    `t = ${s[m].t?.toFixed(2)} s (Node: ${s[m].tRef?.toFixed(2)})`);
}

// 4) no meio da corrida: cada robô na pose do estado, as rodas no giro acumulado e o rastro inteiro
await b.avaliar('window.simulador.recomecar()');
await ate(6);
await pausar();
const conferirRobos = async () => J(`(() => { const e = window.simulador.est, v = window.vista3d.estado(), so = e.m.odometria;
  const fonte = { diferencial: [e.m.diferencial.P, e.m.diferencial.ang, e.m.diferencial.rastro], incremental: [e.m.incremental.P, e.m.incremental.ang, e.m.incremental.rastro],
    exato: [e.m.exato.P, e.m.exato.ang, e.m.exato.rastro], real: [so.real, so.ang, so.rastroReal], odo: [so.P, so.ang, so.rastro] };
  const r = {};
  for (const [nome, [P, ang, rastro]] of Object.entries(fonte)) {
    const x = v.robos[nome], f = v.rastros[nome];
    r[nome] = { vis: x.visivel && f.visivel, oculto: !x.visivel && !f.visivel, pose: x.x === P[0] && x.y === P[1] && x.th === P[2],
      rodas: x.rodaE === ang.e && x.rodaD === ang.d, rastro: f.pontos === rastro.length, raios: x.raios,
      raiosEst: [(e.D + e.odo.dDL) / 2, (e.D + e.odo.dDR) / 2] };
  }
  r.t = e.t;
  return r; })()`);
s = await conferirRobos();
for (const m of ['diferencial', 'incremental']) {
  const x = s[m];
  conferir(`${m}: robô na pose do estado, rodas no giro acumulado, rastro com todos os pontos`, x.vis && x.pose && x.rodas && x.rastro, `t = ${s.t.toFixed(2)} s`);
}
conferir('arco exato e odometria escondidos, como no mapa', ['exato', 'real', 'odo'].every((m) => s[m].oculto));
await b.foto(SAIDA + '3d_pose.png');

// 5) o que aparece segue o mapa: sem o incremental, com o arco exato e com a odometria (o exemplo
// põe a roda direita 2 mm maior: o robô é refeito com os diâmetros reais)
await marcar('modo-inc', false);
await marcar('ex-exato', true);
await b.avaliar(`document.getElementById('btn-odo-exemplo').click()`);
await campo('p-escala', '10');
await ate(6);
await pausar();
s = await conferirRobos();
conferir('sem o modelo incremental, o robô e o rastro dele somem', s.incremental.oculto);
for (const m of ['exato', 'real', 'odo']) conferir(`${m}: à vista, na pose do estado, com o rastro inteiro`, s[m].vis && s[m].pose && s[m].rodas && s[m].rastro);
conferir('rodas com os diâmetros reais (direita 2 mm maior)', s.real.raios.every((r, i) => Math.abs(r - s.real.raiosEst[i]) < 1e-12) && s.real.raios[1] - s.real.raios[0] > 0.00099,
  `raios ${s.real.raios.map((r) => (r * 1000).toFixed(2)).join(' e ')} mm`);
await b.foto(SAIDA + '3d_odometria.png');
await marcar('odo-ativo', false);
await marcar('ex-exato', false);
await marcar('modo-inc', true);

// 6) cada algoritmo com as suas marcas (o seletor abre o exemplo do Corke de cada um)
for (const [lei, rotulos] of [['ponto', ['alvo', 'início']], ['reta', ['reta', 'início']], ['caminho', ['1', '2', '3', 'fim', 'início']], ['pose', ['alvo', 'início']]]) {
  await campo('algoritmo', lei);
  await dormir(600);
  s = await J(`{ e: window.vista3d.estado(), lei: window.simulador.est.lei }`);
  conferir(`${lei}: as marcas dele na vista 3D`, s.lei === lei && rotulos.every((r) => s.e.rotulos.includes(r)), s.e.rotulos.join(' · '));
  if (lei === 'caminho') {
    await campo('p-escala', '10');
    await ate(25);
    await pausar();
    await b.foto(SAIDA + '3d_caminho.png');
    // o início fica na origem: a letra de um eixo cairia em cima do "início" e sai; de cima, a sobreposição
    // muda, mas nenhuma letra à vista cobre outro rótulo
    s = await letras();
    conferir('caminho, início na origem: a letra que cairia no "início" sai, e nenhuma cobre outro rótulo', s.vistas.length < 2 && s.cobrem.length === 0,
      `à vista: ${s.vistas.join(', ') || 'nenhuma'}`);
    await b.avaliar(`document.getElementById('btn-3d-cima').click()`);
    await dormir(500);
    s = await letras();
    conferir('caminho, de cima: alguma letra à vista, e nenhuma cobre outro rótulo', s.vistas.length > 0 && s.cobrem.length === 0, `à vista: ${s.vistas.join(', ')}`);
    await b.avaliar(`document.getElementById('btn-3d-enquadrar').click()`);
  }
}

// 7) câmera: "De cima", "Enquadrar" (com o robô dentro da vista) e "Câmera segue o robô"
await b.avaliar(`document.getElementById('btn-3d-cima').click()`);
await dormir(500);
s = await estado();
conferir('"De cima" olha o chão de cima', elevacao(s.camera) > 85, `elevação ${elevacao(s.camera).toFixed(1)}°`);
await b.foto(SAIDA + '3d_cima.png');
await b.avaliar(`document.getElementById('btn-3d-enquadrar').click()`);
await dormir(500);
s = await estado();
conferir('"Enquadrar" volta à perspectiva', Math.abs(elevacao(s.camera) - 38) < 3, `elevação ${elevacao(s.camera).toFixed(1)}°`);
s = await J(`(() => { const e = window.simulador.est, c = document.getElementById('cena3d'); const ok = [];
  for (const P of [e.P0, e.alvo, e.m.diferencial.P]) { const [x, y, frente] = window.vista3d.naTela(P[0], P[1], 0.12); ok.push(frente && x > 0 && x < c.clientWidth && y > 0 && y < c.clientHeight); }
  return ok; })()`);
conferir('depois de enquadrar, o início, o alvo e o robô estão dentro da vista', s.every(Boolean), JSON.stringify(s));
await marcar('ex-seguir', true);
await campo('p-escala', '1');
await b.avaliar('window.simulador.recomecar()');
await ate(1.5);
await pausar();
await dormir(500);
s = await J(`(() => { const P = window.simulador.est.m.diferencial.P, c = window.vista3d.estado().camera; return Math.hypot(c.alvo[0] - P[0], c.alvo[1] - P[1]); })()`);
conferir('"Câmera segue o robô" leva a câmera junto', s < 0.01, `${(s * 1000).toFixed(1)} mm do robô`);
await marcar('ex-seguir', false);

// 8) tema escuro: o fundo da vista 3D troca junto
const claro = (await estado()).ceu;
await b.tela(1400, 950, true);
await dormir(500);
const escuro = (await estado()).ceu;
conferir('tema escuro escurece o fundo da vista 3D', luz(escuro) < luz(claro), `${claro} → ${escuro}`);
await b.avaliar(`document.getElementById('btn-3d-enquadrar').click()`);
await dormir(400);
await b.foto(SAIDA + '3d_escuro.png');
await b.tela(1400, 950);

// 9) de volta ao mapa 2D: a vista 3D para e some, a URL perde o #3d, as ferramentas e a dica voltam
await vista('2d');
await dormir(300);
const qa = (await estado()).quadros;
await dormir(500);
s = await J(`{ e: window.vista3d.estado(), hash: location.hash, oculto: document.getElementById('cena3d').hidden,
  ferramentas: getComputedStyle(document.getElementById('ferramentas-2d')).display, dica: document.getElementById('dica-tela').textContent,
  w: window.simulador.vista.w }`);
conferir('no mapa 2D, a vista 3D para de desenhar e some', !s.e.ativa && s.e.quadros === qa && s.oculto && s.hash === '', `${s.e.quadros - qa} quadros depois de sair`);
conferir('as ferramentas e a dica do mapa voltam', s.ferramentas !== 'none' && !/girar/.test(s.dica) && s.w > 0);

// 10) #3d na URL abre direto na vista 3D, aqui já na reta
await b.abrir(URL + '?algoritmo=reta#3d');
await dormir(800);
s = await J(`{ e: window.vista3d.estado(), marcado: document.querySelector('input[name="vista"][value="3d"]').checked, lei: window.simulador.est.lei }`);
conferir('#3d na URL abre na vista 3D', s.e.ativa && s.marcado && s.lei === 'reta' && s.e.rotulos.includes('reta'));

// 11) celular: 390 px sem rolagem horizontal, a vista 3D na largura da tela
await b.tela(390, 844, false, true);
await b.abrir(URL + '#3d');
await dormir(800);
s = await J(`{ sw: document.documentElement.scrollWidth, w: innerWidth, c: document.getElementById('cena3d').getBoundingClientRect().width, e: window.vista3d.estado() }`);
conferir('390 px: sem rolagem horizontal, a vista 3D cabe na tela', s.sw <= s.w && s.c <= s.w && s.e.ativa, `scrollWidth ${s.sw}, vista ${Math.round(s.c)} px`);
await b.foto(SAIDA + '3d_celular.png');
await b.tela(1400, 950);

const e = erros(b.eventos);
conferir('sem erros nem avisos no console', e.length === 0, e.slice(0, 3).join(' | '));

// 12) aberta como arquivo, o navegador não carrega módulos: a área da vista 3D diz por quê, e a
// corrida roda assim mesmo (este passo vem depois da conferência do console, que ganha o erro do módulo)
await b.abrir(new globalThis.URL('../../simulacao.html', import.meta.url).href + '#3d');
await dormir(1200);
s = await J(`{ aviso: (document.querySelector('#cena3d .aviso-3d') || {}).textContent || '', t: window.simulador.est.t, modulo: !!window.vista3d }`);
conferir('aberta como arquivo: a vista 3D avisa que precisa do servidor, e a corrida roda', !s.modulo && /python servidor\.py/.test(s.aviso) && s.t > 0, s.aviso.slice(0, 70) + '…');
await b.foto(SAIDA + '3d_arquivo.png');

// 13) servida, mas sem um arquivo do módulo (aqui, o three.js bloqueado): o aviso diz o que faltou,
// e a corrida roda assim mesmo
await b.enviar('Network.enable');
await b.enviar('Network.setBlockedURLs', { urls: ['*three.module.min.js*'] });
await b.abrir(URL + '?algoritmo=pose#3d');
await dormir(1200);
s = await J(`{ aviso: (document.querySelector('#cena3d .aviso-3d') || {}).textContent || '', t: window.simulador.est.t, modulo: !!window.vista3d }`);
conferir('sem o three.js: o aviso diz que um arquivo do módulo não chegou, e a corrida roda', !s.modulo && /não chegou/.test(s.aviso) && s.t > 0, s.aviso.slice(0, 90) + '…');
await b.enviar('Network.setBlockedURLs', { urls: [] });

console.log(falhas ? `\n${falhas} FALHA(S)` : '\ntudo certo');
b.fechar();
process.exit(falhas ? 1 : 0);
