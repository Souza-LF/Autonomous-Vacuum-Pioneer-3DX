/*
 * simulacao.js: estado, tempo, desenho e eventos da simulação de controle.
 *
 * A lei de controle e o passo da malha estão em controladores.js; a cinemática, em cinematica.js.
 * Cada modelo (diferencial, incremental e o arco exato de referência) tem a própria malha fechada:
 * a cada Δt o controlador lê a pose daquele modelo e dá (v, ω), as rodas saem da inversa e
 * saturam, e o modelo anda um passo. Os três partem da mesma pose inicial rumo ao mesmo alvo.
 * A chegada é conferida depois de cada passo, como em simular(), então o tempo de chegada da tela
 * é o mesmo dos testes. O desenho (grade, escala, robô, rastro, gráficos) segue o da Aula 14.
 *
 * Os erros do robô real (diâmetro de cada roda e escorregamento) valem para todas as malhas; o
 * controlador comanda sempre pelo diâmetro nominal. A malha pela odometria é uma quarta malha: o
 * robô real anda pelo arco exato e o controlador só vê a estimativa dos encoders (slide 43).
 *
 * O algoritmo (pose, ponto, reta ou caminho) escolhe a lei de K.LEIS; cada um guarda os próprios
 * ganhos. O "alvo" é uma pose: na pose, a final; no ponto, só a posição conta; na reta, é um ponto
 * dela e a direção do percurso. O caminho usa os waypoints de est.caminho, editados no mapa.
 */
(function () {
  'use strict';

  const C = window.Cinematica;
  const K = window.Controladores;
  const $ = (id) => document.getElementById(id);
  const RAD = Math.PI / 180;
  const GRAU = 180 / Math.PI;
  const MODELOS = ['diferencial', 'incremental'];
  const TODOS = ['diferencial', 'incremental', 'exato'];
  const COR_MODELO = { diferencial: 'dif', incremental: 'inc', exato: 'exato', odometria: 'odo' };
  const NOME = { diferencial: 'diferencial', incremental: 'incremental', exato: 'arco exato', odometria: 'odometria' };
  const MALHAS = [...TODOS, 'odometria'];
  const LIMITE_HIST = 4000;          // pontos guardados por corrida no histórico
  const DT_MIN = 0.005, DT_MAX = 1;  // faixa do slider logarítmico de Δt [s]

  // os exemplos do Corke vêm de controladores.js, os mesmos da página de escolha; a reta da
  // Fig. 4.10, (1, −2, 4), passa por (4, 4) e é percorrida para cima e à direita
  const CORKE = K.EXEMPLOS_CORKE;
  const RETA_CORKE = CORKE.reta.alvo[2];
  const EXEMPLOS = {
    corke: { lei: 'pose', P0: CORKE.pose.P0, alvo: CORKE.pose.alvo },
    frente: { lei: 'pose', P0: [0, 0, 0], alvo: [3, 1.5, 90 * RAD] },
    lado: { lei: 'pose', P0: [0, 0, 0], alvo: [0, 1, 0] },
    'ponto-corke': { lei: 'ponto', P0: CORKE.ponto.P0, alvo: CORKE.ponto.alvo },
    'ponto-atras': { lei: 'ponto', P0: [0, 0, 0], alvo: [-2, 1, 0] },
    'reta-corke': { lei: 'reta', P0: CORKE.reta.P0, alvo: CORKE.reta.alvo },
    'reta-acima': { lei: 'reta', P0: [2, 5, -90 * RAD], alvo: [4, 4, RETA_CORKE] },
    'reta-costas': { lei: 'reta', P0: [6, 5, RETA_CORKE + Math.PI], alvo: [4, 4, RETA_CORKE] },
  };
  // os caminhos prontos de controladores.js viram exemplos "cam-<nome>"
  for (const [nome, c] of Object.entries(K.CAMINHOS)) EXEMPLOS['cam-' + nome] = { lei: 'caminho', P0: c.P0, caminho: c.W };
  // o exemplo com que cada algoritmo abre, e os ganhos padrão de cada um
  const PRIMEIRO = { pose: 'corke', ponto: 'ponto-corke', reta: 'reta-corke', caminho: 'cam-corke' };
  const GANHOS_PADRAO = { pose: K.GANHOS_POSE, ponto: K.GANHOS_PONTO, reta: K.GANHOS_RETA, caminho: K.GANHOS_CAMINHO };
  const CONDICOES = { pose: K.condicoesPose, ponto: K.condicoesPonto, reta: K.condicoesReta, caminho: K.condicoesCaminho };
  const copiarCaminho = (W) => W.map((p) => p.slice());

  const est = {
    D: K.PADRAO.r * 2,      // diâmetro da roda [m]
    L: K.PADRAO.l * 2,      // distância entre as rodas, L = 2l [m]
    wmax: K.PADRAO.wmax,    // limite de cada roda [rad/s]
    dt: 0.05, escala: 1, tmax: 60,
    lei: 'pose',
    ganhos: {
      pose: Object.assign({}, K.GANHOS_POSE), ponto: Object.assign({}, K.GANHOS_PONTO),
      reta: Object.assign({}, K.GANHOS_RETA), caminho: Object.assign({}, K.GANHOS_CAMINHO),
    },
    get k() { return this.ganhos[this.lei]; },   // os ganhos do algoritmo escolhido
    caminho: copiarCaminho(K.CAMINHOS.corke.W),  // waypoints do caminho
    re: true,
    tolRho: 0.01, tolTh: 1 * RAD,
    P0: EXEMPLOS.corke.P0.slice(), alvo: EXEMPLOS.corke.alvo.slice(),
    ativo: { diferencial: true, incremental: true },
    // robô real (diâmetro a mais de cada roda [m] e escorregamento [fração]) e a malha pelos encoders
    odo: { ativo: false, pulsos: 500, dDL: 0, dDR: 0, escL: 0, escR: 0 },
    exibir: { polares: true, exato: false, amostras: true, seguir: false },
    ferramenta: 'alvo',
    rodando: true, fim: false, t: 0, passos: 0, acumulado: 0,
    // por modelo: opções da malha, pose, estado da lei, último comando, chegada, rastro e giro das rodas
    m: {},
    hist: [],
    edicao: null,           // pose sendo marcada no mapa: { tipo, x, y, th, px, py }
  };
  const vista = { x: 0, y: 0, esc: 80, w: 0, h: 0, iniciada: false, roboVisivel: true, ultimoPasso: -1 };
  const sujo = { series: true };
  let cor = {};
  const fonte = { ui: 'system-ui, sans-serif', mat: 'Cambria, Georgia, serif' };
  let hudAnterior = '', leiturasAnterior = '';
  let arraste = null;

  const el = {
    algoritmo: $('algoritmo'), modoDif: $('modo-dif'), modoInc: $('modo-inc'),
    rodar: $('btn-rodar'), reiniciar: $('btn-reiniciar'),
    kRho: $('k-rho'), kRhoNum: $('k-rho-num'), kAlfa: $('k-alfa'), kAlfaNum: $('k-alfa-num'),
    kBeta: $('k-beta'), kBetaNum: $('k-beta-num'),
    kPv: $('k-pv'), kPvNum: $('k-pv-num'), kPh: $('k-ph'), kPhNum: $('k-ph-num'),
    kRd: $('k-rd'), kRdNum: $('k-rd-num'), kRh: $('k-rh'), kRhNum: $('k-rh-num'), kRv: $('k-rv'), kRvNum: $('k-rv-num'),
    kCd: $('k-cd'), kCdNum: $('k-cd-num'), kCh: $('k-ch'), kChNum: $('k-ch-num'), kCv: $('k-cv'), kCvNum: $('k-cv-num'),
    caminhoSec: $('caminho-sec'), camDesfazer: $('btn-cam-desfazer'), camLimpar: $('btn-cam-limpar'),
    leiSec: $('lei-sec'), rotuloAlvo: $('rotulo-alvo'), legendaAlvo: $('legenda-alvo'), retaAbc: $('reta-abc'),
    condicoes: $('condicoes'), avisoInstavel: $('aviso-instavel'), permitirRe: $('permitir-re'),
    leituras: $('leituras'), agoraModelo: $('agora-modelo'), avisoSat: $('aviso-saturacao'),
    mundo: $('mundo'), hud: $('hud'), dicaTela: $('dica-tela'), historicoSub: $('historico-sub'),
    serieRho: $('serie-rho'), serieAngulos: $('serie-angulos'), serieV: $('serie-v'), serieW: $('serie-w'), serieRodas: $('serie-rodas'),
    pX0: $('p-x0'), pY0: $('p-y0'), pTh0: $('p-th0'), pXa: $('p-xa'), pYa: $('p-ya'), pTha: $('p-tha'),
    tolRho: $('tol-rho'), tolTh: $('tol-th'), pTmax: $('p-tmax'),
    pD: $('p-diametro'), pL: $('p-distancia'), pWmax: $('p-wmax'), pDerivados: $('p-derivados'),
    pDt: $('p-dt'), pDtNum: $('p-dt-num'), pEscala: $('p-escala'),
    exPolares: $('ex-polares'), exExato: $('ex-exato'), exAmostras: $('ex-amostras'), exSeguir: $('ex-seguir'),
    errDl: $('err-dl'), errDr: $('err-dr'), escL: $('esc-l'), escR: $('esc-r'),
    odoAtivo: $('odo-ativo'), odoPulsos: $('odo-pulsos'), odoSec: $('odo-sec'), serieOdo: $('serie-odo'),
  };

  // ---------------------------------------------------------------- utilitários

  function num(x, casas) {
    if (!Number.isFinite(x)) return x > 0 ? '∞' : x < 0 ? '−∞' : '—';
    if (Math.abs(x) < 0.5 * Math.pow(10, -casas)) x = 0;
    return x.toLocaleString('pt-BR', { minimumFractionDigits: casas, maximumFractionDigits: casas }).replace('-', '−');
  }
  const limitar = (x, a, b) => Math.min(b, Math.max(a, x));
  function lerNumero(input) {
    const x = parseFloat(String(input.value).replace(',', '.'));
    return Number.isFinite(x) ? x : null;
  }
  function passoBonito(x) {
    const p = Math.pow(10, Math.floor(Math.log10(x)));
    for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= x) return m * p;
    return 10 * p;
  }
  const casasDoPasso = (p) => Math.max(0, -Math.floor(Math.log10(p) + 1e-9));
  const graus = (a) => num(K.angulo(a) * GRAU, 1);

  function lerCores() {
    const s = getComputedStyle(document.documentElement);
    for (const nome of ['texto', 'texto-suave', 'borda', 'painel', 'fundo-cena', 'grade', 'grade-forte',
      'dif', 'dif-corpo', 'inc', 'inc-corpo', 'exato', 'real', 'real-corpo', 'odo', 'wl', 'wr', 'v', 'w', 'alfa', 'beta', 'alvo',
      'eixo-x', 'eixo-y', 'orientacao', 'destaque']) {
      cor[nome] = s.getPropertyValue('--' + nome).trim();
    }
    fonte.ui = s.getPropertyValue('--fonte-ui').trim() || fonte.ui;
    fonte.mat = s.getPropertyValue('--fonte-mat').trim() || fonte.mat;
  }

  function prepararCanvas(cv) {
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (!w || !h) return null;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr);
      cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
  }

  // Texto no canvas com índice: em "φ̇_L" o L sai subscrito. Devolve a largura.
  function textoIndice(ctx, s, x, y, alinhar, soMedir) {
    const fonteAtual = ctx.font;
    const m = /(\d+(?:\.\d+)?)px/.exec(fonteAtual);
    const tam = m ? parseFloat(m[1]) : 12;
    const fonteIndice = fonteAtual.replace(/(\d+(?:\.\d+)?)px/, `${Math.round(tam * 0.78)}px`);
    const partes = s.split(/(_[A-Za-z0-9])/).filter(Boolean)
      .map((p) => (p.length === 2 && p[0] === '_' ? { t: p[1], indice: true } : { t: p, indice: false }));
    let largura = 0;
    for (const p of partes) {
      ctx.font = p.indice ? fonteIndice : fonteAtual;
      p.w = ctx.measureText(p.t).width;
      largura += p.w;
    }
    if (!soMedir) {
      const alinhamento = ctx.textAlign;
      ctx.textAlign = 'left';
      let px = alinhar === 'center' ? x - largura / 2 : alinhar === 'right' ? x - largura : x;
      for (const p of partes) {
        ctx.font = p.indice ? fonteIndice : fonteAtual;
        ctx.fillText(p.t, px, p.indice ? y + tam * 0.28 : y);
        px += p.w;
      }
      ctx.textAlign = alinhamento;
    }
    ctx.font = fonteAtual;
    return largura;
  }

  // ---------------------------------------------------------------- corrida

  function opcoesMalha(modelo) {
    return K.opcoes({
      lei: est.lei, modelo, alvo: est.alvo, caminho: est.caminho, k: est.k, re: est.re, dt: est.dt, tmax: est.tmax,
      r: est.D / 2, l: est.L / 2, wmax: est.wmax, tolRho: est.tolRho, tolTh: est.tolTh,
      rl: (est.D + est.odo.dDL) / 2, rr: (est.D + est.odo.dDR) / 2, escL: est.odo.escL, escR: est.odo.escR, pulsos: est.odo.pulsos,
    });
  }

  // Comando "parado" depois da chegada: v = ω = 0, mas as grandezas da lei (ρ, α, β ou d) continuam
  // as da pose final
  function parado(s) {
    const u = K.LEIS[s.o.lei].comando(s.P, s.o, s.e);
    return Object.assign(u, { v: 0, w: 0, wl: 0, wr: 0, vReal: 0, wReal: 0, saturou: false });
  }

  function conferirChegada(s) {
    if (s.chegou || !K.LEIS[s.o.lei].chegou(s.P, s.o, s.e)) return;
    s.chegou = true;
    s.tChegada = est.t;
    s.u = parado(s);
  }

  // O que falta para uma pose P chegar, pela lei da malha s: { d, th } (th null no ponto; na reta,
  // d tem sinal, positivo à direita do percurso)
  const erroLei = (P, s) => K.LEIS[s.o.lei].erro(P, s.o, s.e);
  // a distância que o controlador vê: ρ na pose e no ponto, d (com sinal) na reta e, no caminho, a
  // distância até p_j, o ponto mais perto
  const distancia = (u) => (est.lei === 'reta' ? u.d : est.lei === 'caminho' ? u.desvio : u.rho);
  // distância de uma pose ao ponto mais perto do caminho inteiro (para o robô real, na odometria)
  function distanciaAoCaminho(P, cam) {
    const j = K.maisPerto(P, cam, 0, 0, true);
    return Math.hypot(cam.P[j][0] - P[0], cam.P[j][1] - P[1]);
  }

  function reiniciar() {
    const P0 = est.P0;
    for (const m of TODOS) {
      const o = opcoesMalha(m);
      const e = K.LEIS[est.lei].iniciar(P0, o);
      const s = { o, e, P: P0.slice(), chegou: false, tChegada: null, rastro: [[P0[0], P0[1]]], ang: { e: 0, d: 0 } };
      // o comando que o controlador daria na pose inicial, para as leituras antes do primeiro passo
      // (rodas() não mexe no estado da lei; o passo, sim, porque o caminho guarda j)
      s.u = K.rodas(s.P, o, e);
      est.m[m] = s;
    }
    // malha pela odometria: P é a estimativa (o que o controlador vê) e real, o robô de verdade
    {
      const o = opcoesMalha('exato');
      const e = K.LEIS[est.lei].iniciar(P0, o);
      const enc = K.inicioOdometria(P0);
      est.m.odometria = {
        o, e, enc, P: enc.odo, real: enc.real, chegou: false, tChegada: null, ang: { e: 0, d: 0 },
        rastro: [[P0[0], P0[1]]], rastroReal: [[P0[0], P0[1]]], u: K.rodas(P0, o, e),
      };
    }
    est.t = 0;
    est.passos = 0;
    est.acumulado = 0;
    est.hist = [];
    est.fim = false;
    for (const m of MALHAS) conferirChegada(est.m[m]);
    registrarHistorico(true);
    vista.roboVisivel = true;
    atualizarBotaoRodar();
  }

  // Recomeça a corrida e põe para rodar: é o que acontece depois de qualquer mudança
  function recomecar() {
    reiniciar();
    est.rodando = true;
    atualizarBotaoRodar();
  }

  function passo() {
    for (const m of TODOS) {
      const s = est.m[m];
      if (s.chegou) continue;
      const p = K.passoMalha(s.P, s.o, s.e);
      const anterior = s.P;
      s.P = p.P;
      s.u = p.amostra;
      s.ang.e += p.amostra.wl * est.dt;
      s.ang.d += p.amostra.wr * est.dt;
      registrarRastro(s.rastro, anterior, s.P, m === 'exato' ? { v: p.amostra.vReal, w: p.amostra.wReal } : null);
    }
    const so = est.m.odometria;
    if (!so.chegou) {
      const p = K.passoMalhaOdometria(so.enc, so.o, so.e);
      const antes = so.real;
      so.enc = p.s;
      so.P = p.s.odo;
      so.real = p.s.real;
      so.u = p.amostra;
      so.ang.e += p.amostra.wl * est.dt;
      so.ang.d += p.amostra.wr * est.dt;
      registrarRastro(so.rastro, null, so.P, null);
      registrarRastro(so.rastroReal, antes, so.real, { v: p.amostra.vReal, w: p.amostra.wReal });
    }
    est.passos += 1;
    est.t += est.dt;
    for (const m of MALHAS) conferirChegada(est.m[m]);
    registrarHistorico(false);
    // a corrida acaba quando todas as malhas que aparecem chegaram (o arco exato sempre conta)
    const todos = TODOS.every((m) => est.m[m].chegou) && (!est.odo.ativo || so.chegou);
    if (todos || est.t >= est.tmax - est.dt / 2) {
      est.rodando = false;
      est.fim = true;
      registrarHistorico(true);
      atualizarBotaoRodar();
    }
  }

  // arco: { v, w } quando a trajetória é um arco exato, para desenhá-la curva entre os passos
  function registrarRastro(r, anterior, P, arco) {
    if (arco) {
      // pontos intermediários a cada ~2° de giro
      const n = Math.min(64, Math.ceil(Math.abs(arco.w * est.dt) / (2 * RAD)));
      for (let i = 1; i < n; i++) {
        const Q = C.passoExato(anterior, arco.v, arco.w, (est.dt * i) / n);
        r.push([Q[0], Q[1]]);
      }
    }
    if (est.dt < 0.01) {
      // passos muito curtos: guarda um ponto a cada 2 mm andados
      const u = r[r.length - 1];
      if (u && Math.hypot(P[0] - u[0], P[1] - u[1]) < 0.002) return;
    }
    r.push([P[0], P[1]]);
  }

  // O histórico guarda a corrida inteira (no máximo LIMITE_HIST pontos), não uma janela deslizante
  function registrarHistorico(forcar) {
    const h = est.hist;
    const u = h[h.length - 1];
    const intervalo = Math.max(0.02, est.tmax / LIMITE_HIST);
    if (!forcar && u && est.t - u.t < intervalo - 1e-9) return;
    if (u && forcar && Math.abs(est.t - u.t) < 1e-9) h.pop();
    const por = {};
    for (const m of MALHAS) {
      const a = est.m[m].u;
      por[m] = {
        dist: distancia(a), alpha: a.alpha, beta: a.beta, erro: a.erro,
        v: a.v, w: a.w, vReal: a.vReal, wReal: a.wReal, wl: a.wl, wr: a.wr,
      };
    }
    // odometria: quanto o robô real está do alvo (da reta, ou do caminho) e da estimativa
    const so = est.m.odometria;
    por.odometria.dReal = est.lei === 'caminho' ? distanciaAoCaminho(so.real, so.e.cam) : erroLei(so.real, so).d;
    por.odometria.desvio = Math.hypot(so.real[0] - so.P[0], so.real[1] - so.P[1]);
    h.push({ t: est.t, por });
    sujo.series = true;
  }

  function atualizarBotaoRodar() {
    el.rodar.textContent = est.rodando ? 'Pausar' : est.fim ? 'Rodar de novo' : 'Iniciar';
  }

  function alternar() {
    if (!est.rodando && est.fim) reiniciar();
    est.rodando = !est.rodando;
    atualizarBotaoRodar();
  }

  let ultimoQuadro = null;
  function quadro(agora) {
    if (ultimoQuadro === null) ultimoQuadro = agora;
    const real = Math.min((agora - ultimoQuadro) / 1000, 0.1);
    ultimoQuadro = agora;
    if (est.rodando) {
      est.acumulado += real * est.escala;
      let n = 0;
      while (est.rodando && est.acumulado >= est.dt && n < 4000) {
        passo();
        est.acumulado -= est.dt;
        n++;
      }
      if (n >= 4000) est.acumulado = 0; // Δt minúsculo com escala alta: não deixa acumular atraso
    }
    desenharMundo();
    if (sujo.series) { desenharSeries(); sujo.series = false; }
    atualizarLeituras();
    requestAnimationFrame(quadro);
  }

  // ---------------------------------------------------------------- cena (robô no plano)

  const paraTela = (x, y) => [vista.w / 2 + (x - vista.x) * vista.esc, vista.h / 2 - (y - vista.y) * vista.esc];
  const paraMundo = (px, py) => [vista.x + (px - vista.w / 2) / vista.esc, vista.y - (py - vista.h / 2) / vista.esc];

  function enquadrarCaixa(x0, x1, y0, y1) {
    const margem = 0.5;
    x0 -= margem; x1 += margem; y0 -= margem; y1 += margem;
    const lx = Math.max(x1 - x0, 1.5), ly = Math.max(y1 - y0, 1.5);
    // o quadro de leitura ocupa o alto da tela: a caixa fica centrada no espaço abaixo dele
    const reservado = Math.min(vista.h * 0.3, el.hud.offsetHeight + 12);
    vista.esc = limitar(Math.min(vista.w / lx, (vista.h - reservado) / ly), 8, 4000);
    vista.x = (x0 + x1) / 2;
    vista.y = (y0 + y1) / 2 + reservado / 2 / vista.esc;
    vista.roboVisivel = true;
  }

  function enquadrar() {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    const incluir = (x, y) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); };
    incluir(est.P0[0], est.P0[1]);
    if (est.lei === 'caminho') for (const p of est.caminho) incluir(p[0], p[1]);
    else incluir(est.alvo[0], est.alvo[1]);
    if (est.lei === 'reta') {
      // o pé da perpendicular do início até a reta e 3,5 m adiante dele, por onde o robô vai seguir
      const L = K.retaDaPose(est.alvo), d = K.distanciaReta(est.P0, L);
      const fx = est.P0[0] - d * L[0], fy = est.P0[1] - d * L[1];
      incluir(fx, fy);
      incluir(fx + 3.5 * Math.cos(est.alvo[2]), fy + 3.5 * Math.sin(est.alvo[2]));
    }
    for (const m of MALHAS) {
      if (!visivel(m) || !est.m[m]) continue;
      for (const p of est.m[m].rastro) incluir(p[0], p[1]);
      if (m === 'odometria') for (const p of est.m[m].rastroReal) incluir(p[0], p[1]);
    }
    enquadrarCaixa(x0, x1, y0, y1);
  }

  function visivel(m) {
    if (m === 'exato') return est.exibir.exato;
    if (m === 'odometria') return est.odo.ativo;
    return est.ativo[m];
  }

  // o modelo cujas leituras aparecem no painel e cujo ρ, α e β são desenhados
  function principal() {
    return MODELOS.find((m) => est.ativo[m]) || (est.odo.ativo ? 'odometria' : 'exato');
  }

  function roboDentro() {
    const P = est.m[principal()].P;
    const [px, py] = paraTela(P[0], P[1]);
    return px >= 40 && px <= vista.w - 40 && py >= 40 && py <= vista.h - 40;
  }

  // Câmera: segue o robô, ou só o mantém à vista quando ele sai pela borda
  function ajustarCamera() {
    if (arraste || est.edicao) return;
    const P = est.m[principal()].P;
    if (est.exibir.seguir) {
      vista.x += (P[0] - vista.x) * 0.2;
      vista.y += (P[1] - vista.y) * 0.2;
      return;
    }
    if (est.passos === vista.ultimoPasso) return;
    vista.ultimoPasso = est.passos;
    const mg = 40;
    const [px, py] = paraTela(P[0], P[1]);
    const dentro = px >= mg && px <= vista.w - mg && py >= mg && py <= vista.h - mg;
    if (dentro) { vista.roboVisivel = true; return; }
    if (!vista.roboVisivel) return; // o usuário levou a vista para outro lugar: não brigar com ele
    if (px < mg) vista.x -= (mg - px) / vista.esc;
    else if (px > vista.w - mg) vista.x += (px - (vista.w - mg)) / vista.esc;
    if (py < mg) vista.y += (mg - py) / vista.esc;
    else if (py > vista.h - mg) vista.y -= (py - (vista.h - mg)) / vista.esc;
  }

  function desenharMundo() {
    const c = prepararCanvas(el.mundo);
    if (!c) return;
    const { ctx, w, h } = c;
    vista.w = w;
    vista.h = h;
    if (!vista.iniciada) {
      enquadrar();
      vista.iniciada = true;
    }
    ajustarCamera();

    desenharGrade(ctx, w, h);
    // o caminho fica embaixo dos rastros, para se ver onde o robô sai dele; os waypoints, por cima
    const W = est.edicao && est.edicao.tipo === 'caminho' ? est.edicao.W : est.caminho;
    if (est.lei === 'caminho') desenharCaminho(ctx, W);
    const p = principal();
    if (est.exibir.polares && visivel(p)) desenharPolares(ctx, est.m[p]);
    if (est.exibir.exato) desenharRastro(ctx, est.m.exato.rastro, cor.exato, [7, 5], false);
    // os pontos de cada passo só fazem sentido enquanto o rastro guarda todos os passos (Δt ≥ 0,01 s)
    const pontos = est.exibir.amostras && est.dt >= 0.01;
    for (const m of MODELOS) if (est.ativo[m]) desenharRastro(ctx, est.m[m].rastro, cor[COR_MODELO[m]], null, pontos);
    const so = est.m.odometria;
    if (est.odo.ativo) {
      desenharRastro(ctx, so.rastroReal, cor.real, null, false);
      desenharRastro(ctx, so.rastro, cor.odo, [8, 5], false);
    }

    desenharInicio(ctx, est.edicao && est.edicao.tipo === 'inicio' ? [est.edicao.x, est.edicao.y, est.edicao.th] : est.P0);
    if (est.lei === 'caminho') desenharWaypoints(ctx, W);
    else desenharAlvo(ctx, est.edicao && est.edicao.tipo === 'alvo' ? [est.edicao.x, est.edicao.y, est.edicao.th] : est.alvo);
    if (est.exibir.exato) desenharRobo(ctx, est.m.exato.P, 'exato', est.m.exato.ang);
    if (est.odo.ativo) {
      desenharRobo(ctx, so.real, 'real', so.ang);
      desenharRobo(ctx, so.P, 'odo', so.ang);
    }
    // o diferencial por cima, como nos slides (robô amarelo)
    for (const m of ['incremental', 'diferencial']) if (est.ativo[m]) desenharRobo(ctx, est.m[m].P, m, est.m[m].ang);
    // o desvio da odometria por cima de tudo, para o número não ficar embaixo de um robô
    if (est.odo.ativo) desenharDesvio(ctx, so.real, so.P);
    desenharEscala(ctx, w, h);
    atualizarHud();
  }

  function desenharGrade(ctx, w, h) {
    const g = passoBonito(70 / vista.esc);
    const casas = casasDoPasso(g);
    const [x0, y1] = paraMundo(0, 0);
    const [x1, y0] = paraMundo(w, h);
    ctx.lineWidth = 1;
    ctx.strokeStyle = cor.grade;
    ctx.beginPath();
    for (let i = Math.ceil(x0 / g); i * g <= x1; i++) {
      const px = Math.round(paraTela(i * g, 0)[0]) + 0.5;
      ctx.moveTo(px, 0); ctx.lineTo(px, h);
    }
    for (let i = Math.ceil(y0 / g); i * g <= y1; i++) {
      const py = Math.round(paraTela(0, i * g)[1]) + 0.5;
      ctx.moveTo(0, py); ctx.lineTo(w, py);
    }
    ctx.stroke();

    // eixos do frame global
    const [ox, oy] = paraTela(0, 0);
    ctx.strokeStyle = cor['grade-forte'];
    ctx.beginPath();
    ctx.moveTo(0, Math.round(oy) + 0.5); ctx.lineTo(w, Math.round(oy) + 0.5);
    ctx.moveTo(Math.round(ox) + 0.5, 0); ctx.lineTo(Math.round(ox) + 0.5, h);
    ctx.stroke();
    const eixo = Math.max(0.5, 50 / vista.esc);
    seta(ctx, [0, 0], [eixo, 0], cor['eixo-x'], 2);
    seta(ctx, [0, 0], [0, eixo], cor['eixo-y'], 2);
    ctx.font = `12px ${fonte.ui}`;
    ctx.fillStyle = cor['eixo-x'];
    const [ax, ay] = paraTela(eixo, 0);
    ctx.fillText('x', ax + 4, ay + 14);
    ctx.fillStyle = cor['eixo-y'];
    const [bx, by] = paraTela(0, eixo);
    ctx.fillText('y', bx - 12, by + 4);

    // valores nas bordas
    ctx.fillStyle = cor['texto-suave'];
    ctx.font = `11px ${fonte.ui}`;
    ctx.textAlign = 'center';
    for (let i = Math.ceil(x0 / g); i * g <= x1; i++) {
      const px = paraTela(i * g, 0)[0];
      if (px > 30 && px < w - 30) ctx.fillText(num(i * g, casas), px, h - 6);
    }
    ctx.textAlign = 'left';
    for (let i = Math.ceil(y0 / g); i * g <= y1; i++) {
      const py = paraTela(0, i * g)[1];
      if (py > 70 && py < h - 24) ctx.fillText(num(i * g, casas), 6, py - 3);
    }
  }

  function desenharEscala(ctx, w, h) {
    const g = passoBonito(70 / vista.esc);
    const px = g * vista.esc;
    const x = w - px - 16, y = h - 26;
    ctx.strokeStyle = cor.texto;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 4); ctx.lineTo(x, y); ctx.lineTo(x + px, y); ctx.lineTo(x + px, y - 4);
    ctx.stroke();
    ctx.fillStyle = cor.texto;
    ctx.font = `11px ${fonte.ui}`;
    ctx.textAlign = 'center';
    ctx.fillText(`${num(g, casasDoPasso(g))} m`, x + px / 2, y - 6);
    ctx.textAlign = 'left';
  }

  function desenharRastro(ctx, pts, corT, tracejado, marcas) {
    if (!pts.length) return;
    ctx.strokeStyle = corT;
    ctx.lineWidth = 2;
    ctx.setLineDash(tracejado || []);
    ctx.beginPath();
    let ux = 0, uy = 0;
    for (let i = 0; i < pts.length; i++) {
      const [px, py] = paraTela(pts[i][0], pts[i][1]);
      if (i === 0) ctx.moveTo(px, py);
      else if (Math.abs(px - ux) + Math.abs(py - uy) >= 0.6 || i === pts.length - 1) ctx.lineTo(px, py);
      else continue;
      ux = px; uy = py;
    }
    ctx.stroke();
    ctx.setLineDash([]);
    if (!marcas) return;
    // um ponto por passo Δt, quando estão separados o bastante para se ver
    ctx.fillStyle = corT;
    let mx = -1e9, my = -1e9;
    for (const p of pts) {
      const [px, py] = paraTela(p[0], p[1]);
      if (Math.abs(px - mx) + Math.abs(py - my) < 6) continue;
      ctx.beginPath();
      ctx.arc(px, py, 2.2, 0, 2 * Math.PI);
      ctx.fill();
      mx = px; my = py;
    }
  }

  // Contorno do P3-DX (slide 26 da Aula 14) na pose P
  function contornoRobo(ctx, P) {
    const [x, y, th] = P;
    const c = Math.cos(th), s = Math.sin(th);
    ctx.beginPath();
    C.P3DX.corpo[0].forEach((bx, i) => {
      const px = bx / 1000, py = C.P3DX.corpo[1][i] / 1000;
      const [a, b] = paraTela(x + c * px - s * py, y + s * px + c * py);
      if (i) ctx.lineTo(a, b); else ctx.moveTo(a, b);
    });
    ctx.closePath();
  }

  // modelo: 'diferencial', 'incremental', 'real', ou 'exato' e 'odo' (só em contorno tracejado, porque
  // não são um robô de verdade); ang: giro de cada roda. As rodas têm os diâmetros reais.
  function desenharRobo(ctx, P, modelo, ang) {
    const DL = est.D + est.odo.dDL, DR = est.D + est.odo.dDR, l = est.L / 2;
    const [x, y, th] = P;
    const c = Math.cos(th), s = Math.sin(th);
    const T = (px, py) => paraTela(x + c * px - s * py, y + s * px + c * py);
    const poligono = (xs, ys) => {
      ctx.beginPath();
      xs.forEach((px, i) => {
        const [a, b] = T(px, ys[i]);
        if (i) ctx.lineTo(a, b); else ctx.moveTo(a, b);
      });
      ctx.closePath();
    };
    const fantasma = modelo === 'exato' || modelo === 'odo';
    const corFantasma = modelo === 'odo' ? cor.odo : cor.exato;
    const corCorpo = { incremental: cor['inc-corpo'], real: cor['real-corpo'] }[modelo] || cor['dif-corpo'];

    contornoRobo(ctx, P);
    ctx.lineWidth = modelo === 'odo' ? 1.8 : 1.2;
    if (fantasma) {
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = corFantasma;
    } else {
      ctx.fillStyle = corCorpo;
      ctx.fill();
      ctx.strokeStyle = cor.texto;
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // rodas nas cores de φ̇_L e φ̇_R, com faixas que andam junto com a superfície da roda (r·φ)
    const lw = C.P3DX.larguraRoda / 1000;
    for (const [lado, D, giro, corRoda] of [[1, DL, ang.e, cor.wl], [-1, DR, ang.d, cor.wr]]) {
      const yc = lado * l;
      poligono([D / 2, D / 2, -D / 2, -D / 2], [yc - lw / 2, yc + lw / 2, yc + lw / 2, yc - lw / 2]);
      if (fantasma) {
        ctx.strokeStyle = corFantasma;
        ctx.stroke();
        continue;
      }
      ctx.fillStyle = corRoda;
      ctx.fill();
      ctx.strokeStyle = cor.texto;
      ctx.stroke();
      if (vista.esc * D < 28) continue;
      const espaco = D / 4;
      const desl = (((giro * D) / 2) % espaco + espaco) % espaco;
      ctx.beginPath();
      for (let u = -D / 2 + desl; u < D / 2; u += espaco) {
        const [a1, b1] = T(u, yc - lw / 2);
        const [a2, b2] = T(u, yc + lw / 2);
        ctx.moveTo(a1, b1);
        ctx.lineTo(a2, b2);
      }
      ctx.strokeStyle = 'rgba(255,255,255,0.75)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.lineWidth = 1.2;
    }

    // ponto médio e orientação, como na figura dos slides (ponto vermelho e traço de 0,12 m)
    const [cx, cy] = T(0, 0);
    const [hx, hy] = T(0.12, 0);
    ctx.strokeStyle = fantasma ? corFantasma : cor.orientacao;
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy); ctx.stroke();
    ctx.fillStyle = fantasma ? corFantasma : cor.orientacao;
    ctx.beginPath(); ctx.arc(cx, cy, 3.5, 0, 2 * Math.PI); ctx.fill();
  }

  // Segmento entre o robô real e a estimativa da odometria, com o tamanho do desvio escrito no meio
  function desenharDesvio(ctx, a, b) {
    const d = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const [x0, y0] = paraTela(a[0], a[1]);
    const [x1, y1] = paraTela(b[0], b[1]);
    if (Math.hypot(x1 - x0, y1 - y0) < 4) return;
    ctx.strokeStyle = cor.odo;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    ctx.setLineDash([]);
    const texto = d >= 1 ? `${num(d, 2)} m` : `${num(d * 1000, 0)} mm`;
    ctx.font = `600 12px ${fonte.ui}`;
    const larg = ctx.measureText(texto).width;
    let lx = (x0 + x1) / 2 + 8;
    const ly = (y0 + y1) / 2 - 6;
    if (lx + larg + 6 > vista.w) lx = (x0 + x1) / 2 - 8 - larg; // perto da borda direita: escreve do outro lado
    ctx.fillStyle = cor.painel;
    ctx.globalAlpha = 0.85;
    ctx.fillRect(lx - 3, ly - 12, larg + 6, 16);
    ctx.globalAlpha = 1;
    ctx.fillStyle = cor.odo;
    ctx.fillText(texto, lx, ly);
  }

  // Alvo de cada algoritmo: a pose final, o ponto ou a reta
  function desenharAlvo(ctx, A) {
    if (est.lei === 'ponto') desenharPonto(ctx, A);
    else if (est.lei === 'reta') desenharReta(ctx, A);
    else desenharPoseFinal(ctx, A);
  }

  // Ponto: mira (anel, cruz e ponto); a orientação não conta
  function desenharPonto(ctx, A) {
    const [px, py] = paraTela(A[0], A[1]);
    ctx.strokeStyle = cor.alvo;
    ctx.fillStyle = cor.alvo;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, 11, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px - 17, py); ctx.lineTo(px - 6, py); ctx.moveTo(px + 6, py); ctx.lineTo(px + 17, py);
    ctx.moveTo(px, py - 17); ctx.lineTo(px, py - 6); ctx.moveTo(px, py + 6); ctx.lineTo(px, py + 17);
    ctx.stroke();
    ctx.beginPath(); ctx.arc(px, py, 3.5, 0, 2 * Math.PI); ctx.fill();
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillText('alvo', px + 15, py + 20);
  }

  // Reta: tracejada de borda a borda da vista, com setas no sentido do percurso θ* e o ponto marcado
  function desenharReta(ctx, A) {
    const ux = Math.cos(A[2]), uy = Math.sin(A[2]);
    const [cx, cy] = paraMundo(vista.w / 2, vista.h / 2);
    const t0 = (cx - A[0]) * ux + (cy - A[1]) * uy;      // o centro da vista projetado na reta
    const meia = Math.hypot(vista.w, vista.h) / vista.esc;
    const ponto = (t) => paraTela(A[0] + t * ux, A[1] + t * uy);
    const [x1, y1] = ponto(t0 - meia), [x2, y2] = ponto(t0 + meia);
    ctx.strokeStyle = cor.alvo;
    ctx.lineWidth = 2;
    ctx.setLineDash([9, 6]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.setLineDash([]);
    // setas a cada ~110 px, em posições fixas da reta (não andam quando a vista anda)
    const passoSeta = passoBonito(110 / vista.esc);
    const ex = ux, ey = -uy;   // direção da reta na tela (y para baixo)
    ctx.fillStyle = cor.alvo;
    for (let i = Math.ceil((t0 - meia) / passoSeta); i * passoSeta <= t0 + meia; i++) {
      if (i === 0) continue;   // no ponto marcado fica o disco
      const [sx, sy] = ponto(i * passoSeta);
      ctx.beginPath();
      ctx.moveTo(sx + 7 * ex, sy + 7 * ey);
      ctx.lineTo(sx - 5 * ex - 5 * ey, sy - 5 * ey + 5 * ex);
      ctx.lineTo(sx - 5 * ex + 5 * ey, sy - 5 * ey - 5 * ex);
      ctx.closePath();
      ctx.fill();
    }
    const [px, py] = paraTela(A[0], A[1]);
    ctx.beginPath(); ctx.arc(px, py, 4.5, 0, 2 * Math.PI); ctx.fill();
    ctx.font = `600 12px ${fonte.ui}`;
    // o rótulo vai do lado esquerdo do percurso, um pouco adiante do ponto marcado
    ctx.fillText('reta', px + 20 * ex + 14 * ey - 12, py + 20 * ey - 14 * ex + 4);
  }

  // Caminho: os waypoints ligados por retas, largo e meio transparente, com uma seta no meio de
  // cada trecho no sentido do percurso
  function desenharCaminho(ctx, W) {
    const tela = W.map((p) => paraTela(p[0], p[1]));
    ctx.strokeStyle = cor.alvo;
    ctx.fillStyle = cor.alvo;
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    tela.forEach(([x, y], i) => { if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.lineJoin = 'miter';
    for (let i = 1; i < tela.length; i++) {
      const [x0, y0] = tela[i - 1], [x1, y1] = tela[i];
      const comp = Math.hypot(x1 - x0, y1 - y0);
      if (comp < 36) continue;
      const ex = (x1 - x0) / comp, ey = (y1 - y0) / comp;
      const sx = (x0 + x1) / 2, sy = (y0 + y1) / 2;
      ctx.beginPath();
      ctx.moveTo(sx + 7 * ex, sy + 7 * ey);
      ctx.lineTo(sx - 5 * ex - 5 * ey, sy - 5 * ey + 5 * ex);
      ctx.lineTo(sx - 5 * ex + 5 * ey, sy - 5 * ey - 5 * ex);
      ctx.closePath();
      ctx.fill();
    }
  }

  // Waypoints numerados (o último é o "fim"), com o número do lado de fora da curva. Num caminho
  // fechado, o fim coincide com o 1 e fica só o anel do fim.
  function desenharWaypoints(ctx, W) {
    const tela = W.map((p) => paraTela(p[0], p[1]));
    const n = tela.length;
    const unit = (dx, dy) => { const c = Math.hypot(dx, dy) || 1; return [dx / c, dy / c]; };
    const fechado = n > 2 && Math.hypot(tela[n - 1][0] - tela[0][0], tela[n - 1][1] - tela[0][1]) < 4;
    let ux = -1e9, uy = -1e9;   // o último waypoint numerado, para não amontoar números
    ctx.font = `600 11px ${fonte.ui}`;
    ctx.textAlign = 'center';
    for (let i = 0; i < n; i++) {
      const [x, y] = tela[i];
      const ant = tela[Math.max(0, i - 1)], seg = tela[Math.min(n - 1, i + 1)];
      // fora da curva: o oposto da soma das direções para os vizinhos; nas pontas, à esquerda do percurso
      let [ox, oy] = [0, 0];
      if (i > 0 && i < n - 1) {
        const a = unit(ant[0] - x, ant[1] - y), b = unit(seg[0] - x, seg[1] - y);
        [ox, oy] = [-(a[0] + b[0]), -(a[1] + b[1])];
      }
      if (Math.hypot(ox, oy) < 0.3) { const t = unit(seg[0] - ant[0], seg[1] - ant[1]); [ox, oy] = [t[1], -t[0]]; }
      [ox, oy] = unit(ox, oy);
      const ultimo = i === n - 1;
      if (ultimo) {
        ctx.strokeStyle = cor.alvo;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.stroke();
      }
      // o "fim" fica fora do anel de raio 9
      if (ultimo && fechado) { ctx.fillStyle = cor.alvo; ctx.fillText('fim', x + 24 * ox, y + 24 * oy + 4); continue; }
      ctx.fillStyle = cor.alvo;
      ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
      if (!ultimo && Math.hypot(x - ux, y - uy) < 24) continue;
      ctx.fillText(ultimo ? 'fim' : String(i + 1), x + (ultimo ? 24 : 15) * ox, y + (ultimo ? 24 : 15) * oy + 4);
      ux = x; uy = y;
    }
    ctx.textAlign = 'left';
    // o waypoint que está sendo arrastado
    if (est.edicao && est.edicao.tipo === 'caminho') {
      const [x, y] = tela[est.edicao.i];
      ctx.strokeStyle = cor.destaque || cor.alvo;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, 12, 0, 2 * Math.PI); ctx.stroke();
    }
  }

  // Pose final: contorno tracejado do robô na pose final, com a seta de θ*
  function desenharPoseFinal(ctx, A) {
    contornoRobo(ctx, A);
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 1.6;
    ctx.strokeStyle = cor.alvo;
    ctx.stroke();
    ctx.setLineDash([]);
    seta(ctx, [A[0], A[1]], [0.4 * Math.cos(A[2]), 0.4 * Math.sin(A[2])], cor.alvo, 2.5);
    const [px, py] = paraTela(A[0], A[1]);
    ctx.fillStyle = cor.alvo;
    ctx.beginPath(); ctx.arc(px, py, 4.5, 0, 2 * Math.PI); ctx.fill();
    // o rótulo fica fora do contorno do robô (que vai a ~0,23 m do centro)
    const fora = Math.max(8, 0.24 * vista.esc + 4);
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillText('alvo', px + fora, py + 16);
  }

  // Início: anel com a seta de θ0
  function desenharInicio(ctx, P) {
    const [px, py] = paraTela(P[0], P[1]);
    seta(ctx, [P[0], P[1]], [0.3 * Math.cos(P[2]), 0.3 * Math.sin(P[2])], cor['texto-suave'], 1.8);
    ctx.strokeStyle = cor['texto-suave'];
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = cor['texto-suave'];
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillText('início', px + 9, py + 16);
  }

  // Arco de um ângulo do mundo, de "de" até "de + delta", com o rótulo no meio
  function arcoAngulo(ctx, centro, raio, de, delta, corA, rotulo) {
    const [cx, cy] = paraTela(centro[0], centro[1]);
    ctx.strokeStyle = corA;
    ctx.lineWidth = 2;
    if (Math.abs(delta) > 0.5 * RAD) {
      // o y da tela é invertido: ângulo a do mundo é −a no canvas
      ctx.beginPath();
      ctx.arc(cx, cy, raio, -de, -(de + delta), delta > 0);
      ctx.stroke();
    }
    const meio = de + delta / 2;
    ctx.fillStyle = corA;
    ctx.font = `italic 600 15px ${fonte.mat}`;
    ctx.textAlign = 'center';
    ctx.fillText(rotulo, cx + (raio + 11) * Math.cos(meio), cy - (raio + 11) * Math.sin(meio) + 5);
    ctx.textAlign = 'left';
  }

  // Semirreta tracejada a partir de um ponto do mundo, na direção "ang", com "comp" pixels
  function semirreta(ctx, P, ang, comp, corS) {
    const [x0, y0] = paraTela(P[0], P[1]);
    ctx.strokeStyle = corS;
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x0 + comp * Math.cos(ang), y0 - comp * Math.sin(ang)); ctx.stroke();
    ctx.setLineDash([]);
  }

  // ρ, α e β do controlador sobre o mapa (Fig. 4.13 do Corke). De ré, a "frente" que vale é θ + 180°
  // e a orientação final é θ* + 180°, que é como o controlador vê o robô virado. No ponto, só ρ e α
  // (= θ* ⊖ θ); na reta, d e θ* ⊖ θ; no caminho, p_j, p_k e α.
  function desenharPolares(ctx, s) {
    if (s.chegou) return;
    if (est.lei === 'reta') { desenharGrandezasReta(ctx, s); return; }
    if (est.lei === 'caminho') { desenharPerseguicao(ctx, s); return; }
    const P = s.P, A = est.alvo;
    const re = s.e.sentido < 0;
    const rho = Math.hypot(A[0] - P[0], A[1] - P[1]);
    if (rho * vista.esc < 30) return;
    const phi = Math.atan2(A[1] - P[1], A[0] - P[0]);   // direção do vetor até o alvo
    const frente = P[2] + (re ? Math.PI : 0);
    const final = A[2] + (re ? Math.PI : 0);
    const [x0, y0] = paraTela(P[0], P[1]);
    const [x1, y1] = paraTela(A[0], A[1]);
    ctx.strokeStyle = cor['texto-suave'];
    ctx.lineWidth = 1.2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = cor['texto-suave'];
    ctx.font = `italic 600 15px ${fonte.mat}`;
    // o rótulo de ρ fica ao lado da reta, no meio dela
    const nx = -Math.sin(phi), ny = Math.cos(phi);
    ctx.fillText('ρ', (x0 + x1) / 2 + 10 * nx - 4, (y0 + y1) / 2 - 10 * ny + 5);

    const raio = limitar(rho * vista.esc * 0.25, 22, 46);
    semirreta(ctx, P, frente, raio * 1.6, cor.alfa);
    arcoAngulo(ctx, P, raio, frente, K.angulo(phi - frente), cor.alfa, 'α');
    if (est.lei !== 'pose') return;
    semirreta(ctx, A, phi, raio * 1.6, cor.beta);
    arcoAngulo(ctx, A, raio, phi, K.angulo(final - phi), cor.beta, 'β');
  }

  // Reta: a perpendicular d do robô até ela e o erro de direção θ* ⊖ θ, medido no robô
  function desenharGrandezasReta(ctx, s) {
    const P = s.P, L = s.e.reta;
    const n = Math.hypot(L[0], L[1]);
    const d = K.distanciaReta(P, L);
    const F = [P[0] - (d * L[0]) / n, P[1] - (d * L[1]) / n];   // pé da perpendicular
    const [x0, y0] = paraTela(P[0], P[1]);
    const [x1, y1] = paraTela(F[0], F[1]);
    if (Math.abs(d) * vista.esc >= 14) {
      ctx.strokeStyle = cor.beta;
      ctx.lineWidth = 1.6;
      ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
      ctx.setLineDash([]);
      // o rótulo fica ao lado da perpendicular, no meio dela
      const tx = -(y1 - y0), ty = x1 - x0, tn = Math.hypot(tx, ty) || 1;
      ctx.fillStyle = cor.beta;
      ctx.font = `italic 600 15px ${fonte.mat}`;
      ctx.fillText('d', (x0 + x1) / 2 + (10 * tx) / tn - 4, (y0 + y1) / 2 + (10 * ty) / tn + 5);
    }
    const th = K.direcaoReta(L);
    const raio = 34;
    semirreta(ctx, P, th, raio * 1.6, cor.alfa);
    arcoAngulo(ctx, P, raio, P[2], K.angulo(th - P[2]), cor.alfa, 'θ*⊖θ');
  }

  // Caminho (Fig. 4.11 do Corke): p_j, o ponto mais perto do robô, o círculo de raio d em volta dele,
  // o trecho até p_k (o ponto perseguido), a visada do robô até p_k e α = θ* ⊖ θ. j e k saem da pose
  // desenhada, pela mesma busca que o próximo passo vai fazer (maisPerto não mexe no estado da lei).
  function desenharPerseguicao(ctx, s) {
    const P = s.P, cam = s.e.cam, d = s.o.k.d;
    if (!(d > 0)) return;
    const j = K.maisPerto(P, cam, s.e.j, d, s.o.buscaGlobal), k = K.perseguido(cam, j, d);
    const pj = cam.P[j], pk = cam.P[k];
    const [xr, yr] = paraTela(P[0], P[1]);
    const [xj, yj] = paraTela(pj[0], pj[1]);
    const [xk, yk] = paraTela(pk[0], pk[1]);
    ctx.strokeStyle = cor.alfa;
    ctx.globalAlpha = 0.4;
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.arc(xj, yj, d * vista.esc, 0, 2 * Math.PI); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    // o trecho de p_j a p_k, por cima do caminho
    ctx.lineWidth = 4;
    ctx.beginPath();
    for (let i = j; i <= k; i++) {
      const [x, y] = paraTela(cam.P[i][0], cam.P[i][1]);
      if (i === j) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    // o desvio (robô até p_j) e a visada (robô até p_k), tracejados
    if (Math.hypot(xj - xr, yj - yr) >= 6) {
      ctx.strokeStyle = cor.beta;
      ctx.lineWidth = 1.6;
      ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(xr, yr); ctx.lineTo(xj, yj); ctx.stroke();
    }
    ctx.strokeStyle = cor['texto-suave'];
    ctx.lineWidth = 1.2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.moveTo(xr, yr); ctx.lineTo(xk, yk); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = cor.beta;
    ctx.beginPath(); ctx.arc(xj, yj, 4, 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = cor.alfa;
    ctx.beginPath(); ctx.arc(xk, yk, 5.5, 0, 2 * Math.PI); ctx.fill();
    // rótulos: p_j do lado do robô, p_k adiante, na direção do trecho
    ctx.font = `italic 600 15px ${fonte.mat}`;
    const ex = xk - xj, ey = yk - yj, ec = Math.hypot(ex, ey) || 1;
    ctx.fillStyle = cor.beta;
    textoIndice(ctx, 'p_j', xj - (12 * ex) / ec - 6, yj - (12 * ey) / ec + 16, 'center');
    ctx.fillStyle = cor.alfa;
    textoIndice(ctx, 'p_k', xk + (14 * ex) / ec, yk + (14 * ey) / ec - 8, 'center');
    const th = Math.atan2(pk[1] - P[1], pk[0] - P[0]);
    const raio = 30;
    semirreta(ctx, P, P[2], raio * 1.6, cor.alfa);
    arcoAngulo(ctx, P, raio, P[2], K.angulo(th - P[2]), cor.alfa, 'α');
  }

  // Seta entre dois pontos do mundo (origem + vetor)
  function seta(ctx, o, d, corS, largura) {
    const [x0, y0] = paraTela(o[0], o[1]);
    const [x1, y1] = paraTela(o[0] + d[0], o[1] + d[1]);
    const len = Math.hypot(x1 - x0, y1 - y0);
    if (len < 2) return;
    const ux = (x1 - x0) / len, uy = (y1 - y0) / len;
    const cab = Math.min(9, len * 0.45);
    ctx.strokeStyle = corS;
    ctx.fillStyle = corS;
    ctx.lineWidth = largura || 2;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1 - ux * cab * 0.6, y1 - uy * cab * 0.6);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - ux * cab - uy * cab * 0.5, y1 - uy * cab + ux * cab * 0.5);
    ctx.lineTo(x1 - ux * cab + uy * cab * 0.5, y1 - uy * cab - ux * cab * 0.5);
    ctx.closePath();
    ctx.fill();
  }

  // "chegou" na pose e no ponto; na reta, que não tem fim, o robô "converge"; o caminho tem fim
  const chegouEm = () => ({ reta: 'convergiu', caminho: 'chegou ao fim' }[est.lei] || 'chegou');
  const naoChegou = () => ({ reta: 'não convergiu', caminho: 'não chegou ao fim' }[est.lei] || 'não chegou');
  // o nome da distância que o controlador vê
  const nomeDistancia = () => ({ reta: 'd', caminho: 'desvio' }[est.lei] || 'ρ');

  // Onde uma pose ficou, em palavras: do alvo (pose e ponto), da reta (com o lado) ou do fim do caminho
  function textoErro(e) {
    if (est.lei === 'reta') return `${num(Math.abs(e.d) * 1000, 0)} mm à ${e.d >= 0 ? 'direita' : 'esquerda'} da reta e ${num(e.th * GRAU, 1)}° fora da direção`;
    if (est.lei === 'ponto') return `${num(e.d * 1000, 0)} mm do alvo`;
    if (est.lei === 'caminho') return `${num(e.d * 1000, 0)} mm do fim do caminho`;
    return `${num(e.d * 1000, 0)} mm e ${num(e.th * GRAU, 1)}° do alvo`;
  }

  // As grandezas do controlador numa linha
  function resumoLei(s) {
    const u = s.u;
    if (est.lei === 'reta') return `d = ${num(u.d, 3)} m · θ* ⊖ θ = ${graus(u.erro)}°`;
    if (est.lei === 'ponto') return `ρ = ${num(u.rho, 3)} m · α = ${graus(u.alpha)}°`;
    if (est.lei === 'caminho') return `desvio = ${num(u.desvio, 3)} m · α = ${graus(u.alpha)}° · s = ${num(u.s, 2)} m`;
    return `ρ =${num(u.rho, 3)} m · α = ${graus(u.alpha)}° · β = ${graus(u.beta)}°${s.e.sentido < 0 ? ' · de ré' : ''}`;
  }

  // Como cada modelo está: chegou (quando e com que erro), não chegou no tempo máximo, ou a caminho
  function situacao(s) {
    if (s === est.m.odometria) {
      // a odometria só sabe da estimativa: o que importa é onde ficou o robô real
      const real = `robô real a ${textoErro(erroLei(s.real, s))}`;
      if (s.chegou) return `<span class="chegou">a estimativa ${chegouEm()} em ${num(s.tChegada, 2)} s</span><span class="suave">${real}</span>`;
      if (est.fim) return `<span class="alerta">${naoChegou()} em ${num(est.tmax, 0)} s</span><span class="suave">${real}</span>`;
      return `${nomeDistancia()} estimado = ${num(distancia(s.u), 3)} m${s.e.sentido < 0 ? ' · de ré' : ''} · ` +
        `real × estimativa: ${num(Math.hypot(s.real[0] - s.P[0], s.real[1] - s.P[1]) * 1000, 0)} mm`;
    }
    if (s.chegou) {
      const e = erroLei(s.P, s);
      const erro = est.lei === 'reta' ? `d = ${num(e.d * 1000, 1)} mm · ${num(e.th * GRAU, 2)}°`
        : est.lei === 'ponto' || est.lei === 'caminho' ? `erro ${num(e.d * 1000, 1)} mm · chegou virado para ${graus(s.P[2])}°`
        : `erro ${num(e.d * 1000, 1)} mm · ${num(e.th * GRAU, 2)}°`;
      return `<span class="chegou">${chegouEm()} em ${num(s.tChegada, 2)} s</span><span class="suave">${erro}</span>`;
    }
    const linha = resumoLei(s);
    return est.fim ? `<span class="alerta">${naoChegou()} em ${num(est.tmax, 0)} s</span><span class="suave">${linha}</span>` : linha;
  }

  function atualizarHud() {
    const linhas = [`<div class="linha"><b>t = ${num(est.t, 2)} s</b><span class="suave">passo ${est.passos} · Δt = ${num(est.dt, 3)} s</span></div>`];
    for (const m of MALHAS) {
      if (!visivel(m)) continue;
      linhas.push(`<div class="linha"><span class="marca" style="background:${cor[COR_MODELO[m]]}"></span>${NOME[m]}: ${situacao(est.m[m])}</div>`);
    }
    if (!est.ativo.diferencial && !est.ativo.incremental && !est.odo.ativo) linhas.push('<div class="linha suave">Marque um dos modelos no topo.</div>');
    if (est.ativo.diferencial && est.ativo.incremental) {
      const a = est.m.diferencial.P, b = est.m.incremental.P;
      linhas.push(`<div class="linha suave">entre os modelos: ${num(Math.hypot(a[0] - b[0], a[1] - b[1]) * 1000, 1)} mm</div>`);
    }
    const html = linhas.join('');
    if (html !== hudAnterior) {
      el.hud.innerHTML = html;
      hudAnterior = html;
    }
  }

  // Painel "Agora": o comando do controlador para o modelo principal
  function atualizarLeituras() {
    const m = principal(), s = est.m[m], u = s.u;
    const maiuscula = (t) => t[0].toUpperCase() + t.slice(1);
    let estado;
    if (s.chegou && m === 'odometria') estado = `A estimativa ${chegouEm()} em ${num(s.tChegada, 2)} s; o robô real está a ${textoErro(erroLei(s.real, s))}`;
    else if (s.chegou) estado = `${maiuscula(chegouEm())} em ${num(s.tChegada, 2)} s`;
    else if (est.fim) estado = `${maiuscula(naoChegou())} em ${num(est.tmax, 0)} s`;
    else if (est.lei === 'reta') estado = `Robô à ${u.d >= 0 ? 'direita' : 'esquerda'} da reta`;
    else if (est.lei === 'ponto') estado = Math.abs(u.alpha) > Math.PI / 2 ? 'Alvo atrás: virando, sem ré' : 'Alvo à frente';
    else if (est.lei === 'caminho') estado = u.k === s.e.cam.n - 1 ? 'Chegando ao fim: v cai com a distância' : 'Seguindo o caminho';
    else estado = s.e.sentido < 0 ? 'Alvo atrás: indo de ré' : 'Alvo à frente: indo em frente';
    const linha = (simb, nome, valor) => `<dt><span class="simb">${simb}</span> ${nome}</dt><dd>${valor}</dd>`;
    const grandezas = est.lei === 'reta'
      ? linha('d', 'distância à reta', `${num(u.d, 3)} m`) + linha('θ* ⊖ θ', 'erro de direção', `${graus(u.erro)}°`)
      : est.lei === 'caminho'
        ? linha('ρ<sub>j</sub>', 'desvio, até p<sub>j</sub>', `${num(u.desvio, 3)} m`) +
          linha('α', 'rumo a p<sub>k</sub>', `${graus(u.alpha)}°`) +
          linha('s', 'andado', `${num(u.s, 2)} / ${num(s.e.cam.total, 2)} m`) +
          linha('ρ<sub>fim</sub>', 'até o fim', `${num(u.rhoFim, 3)} m`)
      : est.lei === 'ponto'
        ? linha('ρ', 'distância', `${num(u.rho, 3)} m`) + linha('α', 'θ* ⊖ θ, rumo ao alvo', `${graus(u.alpha)}°`)
        : linha('ρ', 'distância', `${num(u.rho, 3)} m`) + linha('α', 'alvo visto do robô', `${graus(u.alpha)}°`) +
          linha('β', 'falta girar', `${graus(u.beta)}°`);
    const html = `<dd class="estado">${estado}</dd>` + grandezas +
      // com as rodas saturadas, o robô executa menos do que o comando
      linha('v', 'comando', `${num(u.v, 3)} m/s`) +
      linha('v', 'executado', `${num(u.vReal, 3)} m/s`) +
      linha('ω', 'comando', `${num(u.w * GRAU, 1)} °/s`) +
      linha('ω', 'executado', `${num(u.wReal * GRAU, 1)} °/s`) +
      linha('φ̇<sub>L</sub>', 'roda esquerda', `${num(u.wl * GRAU, 1)} °/s`) +
      linha('φ̇<sub>R</sub>', 'roda direita', `${num(u.wr * GRAU, 1)} °/s`);
    if (html !== leiturasAnterior) {
      el.leituras.innerHTML = html;
      leiturasAnterior = html;
    }
    el.agoraModelo.textContent = NOME[m];
    el.avisoSat.hidden = !u.saturou;
  }

  // ---------------------------------------------------------------- histórico da corrida

  function serie(cv, titulo, linhas, opcoes) {
    const c = prepararCanvas(cv);
    if (!c) return;
    const { ctx, w, h } = c;
    const t0 = 0, t1 = Math.max(est.t, 10);
    const dados = est.hist;
    // a legenda fica ao lado do título; se não couber, desce para uma segunda linha
    const ultimo = dados[dados.length - 1];
    const rotulos = linhas.map((L) => `${L.nome}${ultimo ? ' = ' + num(L.valor(ultimo), opcoes.casas) : ''}`);
    ctx.font = `600 12px ${fonte.ui}`;
    const larguraTitulo = ctx.measureText(titulo).width;
    ctx.font = `11px ${fonte.ui}`;
    const largura = rotulos.reduce((soma, t) => soma + 14 + ctx.measureText(t).width + 28, 0) - 28;
    const duasLinhas = 8 + larguraTitulo + 14 + largura > w - 6;
    const m = { esq: 50, dir: 10, topo: duasLinhas ? 38 : 24, base: 20 };
    const W = w - m.esq - m.dir, H = h - m.topo - m.base;
    let lo = Infinity, hi = -Infinity;
    for (const L of linhas) for (const p of dados) { const y = L.valor(p); lo = Math.min(lo, y); hi = Math.max(hi, y); }
    for (const y of opcoes.limites || []) { lo = Math.min(lo, y); hi = Math.max(hi, y); }
    if (!Number.isFinite(lo)) { lo = 0; hi = 0; }
    lo = Math.min(lo, 0);
    hi = Math.max(hi, 0);
    if (hi - lo < opcoes.minFaixa) {
      const meio = opcoes.base0 ? opcoes.minFaixa / 2 : (hi + lo) / 2;
      lo = meio - opcoes.minFaixa / 2;
      hi = meio + opcoes.minFaixa / 2;
    }
    const folga = (hi - lo) * 0.08;
    lo -= opcoes.base0 ? 0 : folga;
    hi += folga;
    const px = (t) => m.esq + ((t - t0) / (t1 - t0)) * W;
    const py = (y) => m.topo + ((hi - y) / (hi - lo)) * H;

    // grade e valores
    const passo = passoBonito((hi - lo) / 4);
    const casas = casasDoPasso(passo);
    ctx.font = `11px ${fonte.ui}`;
    ctx.lineWidth = 1;
    ctx.strokeStyle = cor.grade;
    ctx.fillStyle = cor['texto-suave'];
    ctx.textAlign = 'right';
    ctx.beginPath();
    for (let i = Math.ceil(lo / passo); i * passo <= hi; i++) {
      const y = Math.round(py(i * passo)) + 0.5;
      ctx.moveTo(m.esq, y); ctx.lineTo(m.esq + W, y);
      ctx.fillText(num(i * passo, casas), m.esq - 5, y + 4);
    }
    ctx.textAlign = 'center';
    const passoT = passoBonito((t1 - t0) / 4);
    for (let s = 0; s <= t1 + 1e-9; s += passoT) {
      const x = Math.round(px(s)) + 0.5;
      ctx.moveTo(x, m.topo); ctx.lineTo(x, m.topo + H);
      ctx.fillText(`${num(s, casasDoPasso(passoT))} s`, x, h - 5);
    }
    ctx.stroke();
    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = cor['grade-forte'];
      ctx.beginPath(); ctx.moveTo(m.esq, Math.round(py(0)) + 0.5); ctx.lineTo(m.esq + W, Math.round(py(0)) + 0.5); ctx.stroke();
    }

    ctx.save();
    ctx.beginPath(); ctx.rect(m.esq, m.topo, W, H); ctx.clip();
    // limites (o das rodas), tracejados
    ctx.strokeStyle = cor['texto-suave'];
    ctx.setLineDash([4, 4]);
    for (const y of opcoes.limites || []) {
      ctx.beginPath(); ctx.moveTo(m.esq, Math.round(py(y)) + 0.5); ctx.lineTo(m.esq + W, Math.round(py(y)) + 0.5); ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const L of linhas) {
      ctx.strokeStyle = L.cor;
      ctx.lineWidth = L.tracejado ? 1.5 : 2;
      ctx.setLineDash(L.tracejado ? [5, 4] : []);
      ctx.beginPath();
      dados.forEach((p, i) => { const x = px(p.t), y = py(L.valor(p)); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.restore();

    // título e legenda com o valor atual
    ctx.textAlign = 'left';
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillStyle = cor.texto;
    ctx.fillText(titulo, 8, 15);
    let x = duasLinhas ? 8 : 8 + larguraTitulo + 14;
    const yLeg = duasLinhas ? 31 : 15;
    ctx.font = `11px ${fonte.ui}`;
    linhas.forEach((L, i) => {
      // série tracejada: quadrado vazado na legenda
      if (L.tracejado) {
        ctx.strokeStyle = L.cor;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(x + 0.75, yLeg - 7.25, 8.5, 8.5);
      } else {
        ctx.fillStyle = L.cor;
        ctx.fillRect(x, yLeg - 8, 10, 10);
      }
      ctx.fillStyle = cor.texto;
      x += textoIndice(ctx, rotulos[i], x + 14, yLeg, 'left') + 28;
    });
  }

  function desenharSeries() {
    const p = principal();
    const de = (campo, fator) => (h) => h.por[p][campo] * (fator || 1);
    const reta = est.lei === 'reta';
    const distancias = TODOS.filter(visivel).map((m) => ({ nome: m === 'exato' ? 'exato' : m.slice(0, 3) + '.', cor: cor[COR_MODELO[m]], valor: (h) => h.por[m].dist }));
    if (est.odo.ativo) {
      // pela odometria: a distância que o controlador vê (estimativa, tracejado) e a do robô real
      distancias.push({ nome: 'estim.', cor: cor.odo, tracejado: true, valor: (h) => h.por.odometria.dist });
      distancias.push({ nome: 'real', cor: cor.real, valor: (h) => h.por.odometria.dReal });
    }
    // na reta, d tem sinal (positivo à direita do percurso), então o gráfico não começa no zero
    const titulo = { reta: 'Distância à reta d (m)', caminho: 'Desvio do caminho (m)' }[est.lei] || 'Distância ρ (m)';
    serie(el.serieRho, titulo,
      distancias.length ? distancias : [{ nome: nomeDistancia(), cor: cor.exato, valor: de('dist') }],
      { minFaixa: 0.1, casas: 3, base0: !reta });
    const angulos = reta ? [{ nome: 'θ* ⊖ θ', cor: cor.alfa, valor: de('erro', GRAU) }]
      : est.lei === 'ponto' || est.lei === 'caminho' ? [{ nome: 'α = θ* ⊖ θ', cor: cor.alfa, valor: de('alpha', GRAU) }]
        : [{ nome: 'α', cor: cor.alfa, valor: de('alpha', GRAU) }, { nome: 'β', cor: cor.beta, valor: de('beta', GRAU) }];
    serie(el.serieAngulos, 'Ângulos (°)', angulos, { minFaixa: 10, casas: 1 });
    // comando do controlador (tracejado) e o que o robô executa depois da saturação das rodas
    serie(el.serieV, 'v (m/s)', [
      { nome: 'comando', cor: cor.v, tracejado: true, valor: de('v') },
      { nome: 'executado', cor: cor.v, valor: de('vReal') },
    ], { minFaixa: 0.1, casas: 3 });
    serie(el.serieW, 'ω (°/s)', [
      { nome: 'comando', cor: cor.w, tracejado: true, valor: de('w', GRAU) },
      { nome: 'executado', cor: cor.w, valor: de('wReal', GRAU) },
    ], { minFaixa: 10, casas: 1 });
    serie(el.serieRodas, 'Rodas (°/s)', [
      { nome: 'φ̇_L', cor: cor.wl, valor: de('wl', GRAU) },
      { nome: 'φ̇_R', cor: cor.wr, valor: de('wr', GRAU) },
    ], { minFaixa: 20, casas: 0, limites: [est.wmax * GRAU, -est.wmax * GRAU] });
    if (est.odo.ativo) {
      serie(el.serieOdo, 'Real × odometria (mm)', [
        { nome: 'desvio', cor: cor.odo, valor: (h) => h.por.odometria.desvio * 1000 },
      ], { minFaixa: 1, casas: 1, base0: true });
    }
    el.historicoSub.textContent = `${NOME[p]}${est.fim ? ' · corrida encerrada' : ''}`;
  }

  // ---------------------------------------------------------------- controles

  // os sliders de cada algoritmo: chave em est.ganhos[lei], slider e campo numérico
  const GANHOS = [
    { lei: 'pose', chave: 'rho', faixa: 'kRho', campo: 'kRhoNum', casas: 2 },
    { lei: 'pose', chave: 'alpha', faixa: 'kAlfa', campo: 'kAlfaNum', casas: 2 },
    { lei: 'pose', chave: 'beta', faixa: 'kBeta', campo: 'kBetaNum', casas: 2 },
    { lei: 'ponto', chave: 'v', faixa: 'kPv', campo: 'kPvNum', casas: 2 },
    { lei: 'ponto', chave: 'h', faixa: 'kPh', campo: 'kPhNum', casas: 2 },
    { lei: 'reta', chave: 'd', faixa: 'kRd', campo: 'kRdNum', casas: 2 },
    { lei: 'reta', chave: 'h', faixa: 'kRh', campo: 'kRhNum', casas: 2 },
    { lei: 'reta', chave: 'v', faixa: 'kRv', campo: 'kRvNum', casas: 2 },
    { lei: 'caminho', chave: 'd', faixa: 'kCd', campo: 'kCdNum', casas: 2 },
    { lei: 'caminho', chave: 'h', faixa: 'kCh', campo: 'kChNum', casas: 2 },
    { lei: 'caminho', chave: 'v', faixa: 'kCv', campo: 'kCvNum', casas: 2 },
  ];

  // O que a escolha de ganhos faz, além das condições: no ponto, onde α para; na reta, ζ e ωn; no
  // caminho, a chegada (a lei do ponto com Kv = v/d) e o corte das quinas
  function textoLei() {
    const k = est.k;
    if (est.lei === 'ponto') {
      const ae = K.equilibrioPonto(k);
      if (ae === null) return '';
      if (ae === 0) return `Kh/Kv = ${num(k.h / k.v, 2)}: α vai a zero e o robô termina apontando para o alvo.`;
      if (ae < Math.PI / 2) return `Kh < Kv: α para em ${num(ae * GRAU, 1)}°, e o robô chega em espiral, sem nunca apontar para o alvo.`;
      return `Kh/Kv abaixo de 2/π: α para em ${num(ae * GRAU, 1)}°, além de 90°, e o robô se afasta em espiral.`;
    }
    if (est.lei === 'reta') {
      if (!(k.d > 0 && k.v > 0)) return '';
      const { wn, zeta } = K.amortecimentoReta(k);
      const tipo = zeta < 1 ? `subamortecido: passa da reta, com sobressinal de ${num(100 * Math.exp((-Math.PI * zeta) / Math.sqrt(1 - zeta * zeta)), 1)}%`
        : zeta === 1 ? 'amortecimento crítico: não passa da reta' : 'superamortecido: não passa da reta';
      return `Perto da reta, ωn = ${num(wn, 2)} rad/s e ζ = ${num(zeta, 2)} (${tipo}).`;
    }
    if (est.lei === 'caminho') {
      if (!(k.d > 0 && k.h > 0 && k.v > 0)) return '';
      const kv = k.v / k.d;
      const fim = k.h > kv ? 'Kh > Kv: α vai a zero e o robô chega ao fim apontando para ele.'
        : 'Kh < Kv: perto do fim α cresce em vez de ir a zero, como no ponto, e o robô chega torto, se chegar.';
      return `No fim, v cai para v·ρ/d: é a lei do ponto com Kv = v/d = ${num(kv, 2)} 1/s. ${fim} ` +
        `O robô corta as quinas; com d bem maior que v/Kh = ${num(k.v / k.h, 2)} m, o corte é proporcional a d.`;
    }
    return '';
  }

  const AVISO = {
    pose: 'Uma condição de estabilidade não vale: o robô não deve chegar ao alvo.',
    ponto: 'Uma condição não vale: α não vai a zero, ou o robô nem chega ao ponto.',
    reta: 'Uma condição não vale: o robô não deve convergir para a reta.',
    caminho: 'Uma condição não vale: o robô não deve seguir o caminho.',
  };

  function escreverGanhos() {
    for (const g of GANHOS) {
      const k = est.ganhos[g.lei];
      el[g.faixa].value = k[g.chave];
      if (document.activeElement !== el[g.campo]) el[g.campo].value = Number(k[g.chave].toFixed(g.casas));
    }
    const conds = CONDICOES[est.lei](est.k);
    el.condicoes.innerHTML = conds.map((c) =>
      `<span class="cond ${c.ok ? 'ok' : 'ruim'}">${c.texto.replace(/k(ρ|α|β)/g, 'k<sub>$1</sub>').replace(/K(v|h|d)/g, 'K<sub>$1</sub>')} ${c.ok ? '✓' : '✗'}</span>`).join('');
    el.avisoInstavel.textContent = AVISO[est.lei];
    el.avisoInstavel.hidden = conds.every((c) => c.ok);
    const texto = textoLei();
    el.leiSec.innerHTML = texto.replace(/K(v|h|d)/g, 'K<sub>$1</sub>');
    el.leiSec.hidden = !texto;
    escreverCaminho();   // o tempo do percurso depende de v
  }

  // O caminho em números, e se o tempo máximo dá para percorrê-lo
  function escreverCaminho() {
    const W = est.caminho;
    let L = 0;
    for (let i = 1; i < W.length; i++) L += Math.hypot(W[i][0] - W[i - 1][0], W[i][1] - W[i - 1][1]);
    const v = est.ganhos.caminho.v, t = L / v;
    el.caminhoSec.textContent = `${W.length} waypoints, ${num(L, 2)} m. A ${num(v, 2)} m/s, o percurso leva pelo menos ${num(t, 1)} s` +
      (t > est.tmax ? `, mais que o tempo máximo de ${num(est.tmax, 0)} s (em Chegada).` : '.');
    el.camDesfazer.disabled = W.length <= 2;
  }

  function definirCaminho(W) {
    est.caminho = W;
    escreverCaminho();
    recomecar();
  }

  function definirGanhos(k) {
    Object.assign(est.k, k);
    escreverGanhos();
    recomecar();
  }

  function escreverPoses() {
    const campos = [[el.pX0, est.P0[0], 2], [el.pY0, est.P0[1], 2], [el.pTh0, est.P0[2] * GRAU, 1],
      [el.pXa, est.alvo[0], 2], [el.pYa, est.alvo[1], 2], [el.pTha, est.alvo[2] * GRAU, 1]];
    for (const [e, valor, casas] of campos) if (document.activeElement !== e) e.value = Number(valor.toFixed(casas));
    // a reta marcada em (a, b, c), como o Corke a escreve
    const [a, b, c] = K.retaDaPose(est.alvo);
    el.retaAbc.textContent = `ax + by + c = 0 com a = ${num(a, 3)}, b = ${num(b, 3)}, c = ${num(c, 3)}; θ* = atan2(a, −b) é a direção do percurso.`;
  }

  // Mostra os controles do algoritmo (sem mexer nas poses): ganhos, rótulos do alvo e dica do mapa
  const LEGENDA_ALVO = { pose: 'Alvo', ponto: 'Alvo (só a posição conta)', reta: 'Reta: um ponto e a direção θ*', caminho: 'Alvo' };
  function mostrarLei(lei) {
    est.lei = lei;
    el.algoritmo.value = lei;
    document.querySelectorAll('[data-leis]').forEach((e) => { e.hidden = !e.dataset.leis.split(' ').includes(lei); });
    el.rotuloAlvo.textContent = { reta: 'Marcar reta', caminho: 'Editar caminho' }[lei] || 'Marcar alvo';
    el.legendaAlvo.textContent = LEGENDA_ALVO[lei];
    definirFerramenta(est.ferramenta);
    escreverGanhos();
    sujo.series = true;
  }

  // Trocar de algoritmo abre o primeiro exemplo dele (o do Corke) e põe o algoritmo na URL, para
  // recarregar a página ou voltar da escolha cair nele
  function definirLei(lei) {
    aplicarExemplo(PRIMEIRO[lei]);
    try { history.replaceState(null, '', '?algoritmo=' + lei + location.hash); } catch (e) { /* file:// pode recusar */ }
  }

  function lerPoses() {
    const ler = (e, antes) => { const x = lerNumero(e); return x === null ? antes : x; };
    est.P0 = [ler(el.pX0, est.P0[0]), ler(el.pY0, est.P0[1]), ler(el.pTh0, est.P0[2] * GRAU) * RAD];
    est.alvo = [ler(el.pXa, est.alvo[0]), ler(el.pYa, est.alvo[1]), ler(el.pTha, est.alvo[2] * GRAU) * RAD];
  }

  function aplicarExemplo(nome) {
    const ex = EXEMPLOS[nome];
    if (ex.lei !== est.lei) mostrarLei(ex.lei);
    est.P0 = ex.P0.slice();
    if (ex.alvo) est.alvo = ex.alvo.slice();
    if (ex.caminho) { est.caminho = copiarCaminho(ex.caminho); escreverCaminho(); }
    escreverPoses();
    recomecar();
    enquadrar();
  }

  function escreverDerivados() {
    const vmax = (est.D / 2) * est.wmax;
    const wmax = vmax / (est.L / 2);
    el.pDerivados.textContent = `Em linha reta, v máx. = r·φ̇ máx. = ${num(vmax, 3)} m/s; girando no lugar, ω máx. = ${num(wmax * GRAU, 0)} °/s.`;
  }

  function lerParametros() {
    const D = lerNumero(el.pD), L = lerNumero(el.pL), wmax = lerNumero(el.pWmax);
    if (D && D > 0) est.D = D / 1000;
    if (L && L > 0) est.L = L / 1000;
    if (wmax && wmax > 0) est.wmax = wmax * RAD;
    escreverDerivados();
    recomecar();
  }

  function escreverOdometria() {
    const N = est.odo.pulsos;
    const mmPorPulso = (Math.PI * est.D * 1000) / N;
    el.odoSec.textContent = `${num(N, 0)} pulsos por volta: um pulso vale ${num(360 / N, 3)}° de roda e ${num(mmPorPulso, 2)} mm de chão.`;
    el.serieOdo.hidden = !est.odo.ativo;
    el.odoAtivo.checked = est.odo.ativo;
    for (const [e, valor] of [[el.errDl, est.odo.dDL * 1000], [el.errDr, est.odo.dDR * 1000], [el.escL, est.odo.escL * 100],
      [el.escR, est.odo.escR * 100], [el.odoPulsos, N]]) {
      if (document.activeElement !== e) e.value = Number(valor.toFixed(3));
    }
  }

  function lerOdometria() {
    const ler = (e, antes, fator, min, max) => { const x = lerNumero(e); return x === null ? antes : limitar(x, min, max) / fator; };
    est.odo.dDL = ler(el.errDl, est.odo.dDL * 1000, 1000, -50, 50);
    est.odo.dDR = ler(el.errDr, est.odo.dDR * 1000, 1000, -50, 50);
    est.odo.escL = ler(el.escL, est.odo.escL * 100, 100, 0, 90);
    est.odo.escR = ler(el.escR, est.odo.escR * 100, 100, 0, 90);
    est.odo.pulsos = Math.round(ler(el.odoPulsos, est.odo.pulsos, 1, 1, 1e6));
    est.odo.ativo = el.odoAtivo.checked;
    escreverOdometria();
    sujo.series = true;
    recomecar();
  }

  // O exemplo dos testes: o do Corke (do algoritmo escolhido) com a roda direita 2 mm maior, pela odometria
  function exemploOdometria() {
    Object.assign(est.odo, { ativo: true, dDL: 0, dDR: 0.002, escL: 0, escR: 0, pulsos: 500 });
    escreverOdometria();
    aplicarExemplo(PRIMEIRO[est.lei]);
  }

  const dtDoSlider = (u) => Number((DT_MIN * Math.pow(DT_MAX / DT_MIN, u / 100)).toPrecision(2));
  const sliderDoDt = (dt) => (100 * Math.log(dt / DT_MIN)) / Math.log(DT_MAX / DT_MIN);
  function definirDt(dt, origem) {
    est.dt = limitar(dt, DT_MIN, DT_MAX);
    if (origem !== el.pDt) el.pDt.value = sliderDoDt(est.dt);
    if (origem !== el.pDtNum) el.pDtNum.value = est.dt;
  }

  // Ferramenta do mapa: marcar o alvo, marcar o início ou só mover a vista
  const DICAS = {
    alvo: {
      pose: 'Clique para pôr o alvo; arraste a partir dele para escolher a orientação final θ*.',
      ponto: 'Clique para pôr o alvo: só a posição conta.',
      reta: 'Clique num ponto da reta e arraste no sentido do percurso θ*.',
      caminho: 'Clique para pôr um waypoint no fim; arraste um waypoint para movê-lo, ou um trecho para inserir outro; Shift+clique ou botão direito apaga.',
    },
    inicio: 'Clique para pôr o início; arraste a partir dele para escolher a orientação θ0.',
    mover: 'Arraste para mover a vista; roda do mouse ou botões para o zoom; duplo clique enquadra.',
  };
  function definirFerramenta(f) {
    est.ferramenta = f;
    el.mundo.classList.toggle('mover', f === 'mover');
    el.dicaTela.textContent = f === 'alvo' ? DICAS.alvo[est.lei] : DICAS[f];
  }

  function ligar() {
    el.rodar.addEventListener('click', alternar);
    el.reiniciar.addEventListener('click', () => { reiniciar(); sujo.series = true; });
    el.modoDif.addEventListener('change', () => { est.ativo.diferencial = el.modoDif.checked; sujo.series = true; });
    el.modoInc.addEventListener('change', () => { est.ativo.incremental = el.modoInc.checked; sujo.series = true; });

    for (const g of GANHOS) {
      el[g.faixa].addEventListener('input', () => definirGanhos({ [g.chave]: Number(el[g.faixa].value) }));
      el[g.campo].addEventListener('input', () => { const x = lerNumero(el[g.campo]); if (x !== null) definirGanhos({ [g.chave]: x }); });
      el[g.campo].addEventListener('change', escreverGanhos);
    }
    $('btn-ganhos').addEventListener('click', () => definirGanhos(GANHOS_PADRAO[est.lei]));
    el.algoritmo.addEventListener('change', () => definirLei(el.algoritmo.value));
    el.permitirRe.addEventListener('change', () => { est.re = el.permitirRe.checked; recomecar(); });

    for (const e of [el.pX0, el.pY0, el.pTh0, el.pXa, el.pYa, el.pTha]) {
      e.addEventListener('change', () => { lerPoses(); escreverPoses(); recomecar(); enquadrar(); });
    }
    document.querySelectorAll('[data-exemplo]').forEach((b) => b.addEventListener('click', () => aplicarExemplo(b.dataset.exemplo)));
    el.camDesfazer.addEventListener('click', () => { if (est.caminho.length > 2) definirCaminho(est.caminho.slice(0, -1)); });
    // um caminho novo: do início, 1 m à frente do robô; o resto se acrescenta clicando no mapa
    el.camLimpar.addEventListener('click', () => {
      const [x, y, th] = est.P0;
      definirCaminho([[arred(x), arred(y)], [arred(x + Math.cos(th)), arred(y + Math.sin(th))]]);
    });

    const lerChegada = () => {
      const tr = lerNumero(el.tolRho), tt = lerNumero(el.tolTh), tm = lerNumero(el.pTmax);
      if (tr && tr > 0) est.tolRho = tr / 100;
      if (tt && tt > 0) est.tolTh = tt * RAD;
      if (tm && tm > 0) est.tmax = tm;
      escreverCaminho();
      recomecar();
    };
    for (const e of [el.tolRho, el.tolTh, el.pTmax]) e.addEventListener('change', lerChegada);

    for (const e of [el.pD, el.pL, el.pWmax]) e.addEventListener('change', lerParametros);
    $('btn-p3dx').addEventListener('click', () => {
      el.pD.value = C.P3DX.diametroRoda;
      el.pL.value = C.P3DX.distanciaRodas;
      el.pWmax.value = 360;
      lerParametros();
    });

    el.pDt.addEventListener('input', () => { definirDt(dtDoSlider(Number(el.pDt.value)), el.pDt); recomecar(); });
    el.pDtNum.addEventListener('change', () => { const x = lerNumero(el.pDtNum); if (x) { definirDt(x, null); recomecar(); } });
    el.pEscala.addEventListener('change', () => { est.escala = Number(el.pEscala.value); });
    for (const e of [el.errDl, el.errDr, el.escL, el.escR, el.odoPulsos, el.odoAtivo]) e.addEventListener('change', lerOdometria);
    $('btn-odo-exemplo').addEventListener('click', exemploOdometria);

    el.exPolares.addEventListener('change', () => { est.exibir.polares = el.exPolares.checked; });
    el.exExato.addEventListener('change', () => { est.exibir.exato = el.exExato.checked; sujo.series = true; });
    el.exAmostras.addEventListener('change', () => { est.exibir.amostras = el.exAmostras.checked; });
    el.exSeguir.addEventListener('change', () => { est.exibir.seguir = el.exSeguir.checked; vista.roboVisivel = true; });

    document.querySelectorAll('input[name="ferramenta"]').forEach((r) => r.addEventListener('change', () => definirFerramenta(r.value)));

    // mapa: com "mover", arrastar move a vista; com "alvo" ou "início", o clique põe a pose e
    // arrastar a partir dela escolhe a orientação (no caminho, "alvo" edita os waypoints). A roda do
    // mouse dá zoom nas três ferramentas.
    const pontoTela = (e) => { const r = el.mundo.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; };
    el.mundo.addEventListener('pointerdown', (e) => {
      el.mundo.setPointerCapture(e.pointerId);
      if (est.ferramenta === 'mover') {
        arraste = { x: e.clientX, y: e.clientY, vx: vista.x, vy: vista.y };
        el.mundo.classList.add('arrastando');
        return;
      }
      const [px, py] = pontoTela(e);
      if (est.ferramenta === 'alvo' && est.lei === 'caminho') { pegarCaminho(px, py, e.shiftKey || e.button === 2); return; }
      const [x, y] = paraMundo(px, py);
      const antes = est.ferramenta === 'alvo' ? est.alvo : est.P0;
      est.edicao = { tipo: est.ferramenta, x, y, th: antes[2], px, py };
    });
    el.mundo.addEventListener('pointermove', (e) => {
      if (arraste) {
        vista.x = arraste.vx - (e.clientX - arraste.x) / vista.esc;
        vista.y = arraste.vy + (e.clientY - arraste.y) / vista.esc;
        return;
      }
      if (!est.edicao) return;
      const [px, py] = pontoTela(e);
      if (est.edicao.tipo === 'caminho') { est.edicao.W[est.edicao.i] = paraMundo(px, py); return; }
      const dx = px - est.edicao.px, dy = py - est.edicao.py;
      if (Math.hypot(dx, dy) > 10) est.edicao.th = Math.atan2(-dy, dx); // y da tela para baixo
    });
    const soltar = () => {
      if (arraste) {
        arraste = null;
        el.mundo.classList.remove('arrastando');
        vista.roboVisivel = roboDentro();
      }
      if (est.edicao && est.edicao.tipo === 'caminho') {
        // o waypoint fica no centímetro; um clique sem arrastar num waypoint não muda nada
        const { W, i, antes } = est.edicao;
        W[i] = W[i].map(arred);
        est.edicao = null;
        if (JSON.stringify(W) !== antes) definirCaminho(W);
      } else if (est.edicao) {
        const { tipo, x, y, th } = est.edicao;
        const pose = [arred(x), arred(y), Math.round(th * GRAU) * RAD];
        if (tipo === 'alvo') est.alvo = pose; else est.P0 = pose;
        est.edicao = null;
        escreverPoses();
        recomecar();
      }
    };
    el.mundo.addEventListener('pointerup', soltar);
    el.mundo.addEventListener('pointercancel', () => { est.edicao = null; soltar(); });
    // o botão direito apaga waypoints: sem o menu do navegador por cima
    el.mundo.addEventListener('contextmenu', (e) => { if (est.ferramenta === 'alvo' && est.lei === 'caminho') e.preventDefault(); });
    el.mundo.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [px, py] = pontoTela(e);
      zoom(Math.exp(-e.deltaY * 0.0015), px, py);
    }, { passive: false });
    el.mundo.addEventListener('dblclick', () => { if (est.ferramenta === 'mover') enquadrar(); });
    $('btn-zoom-mais').addEventListener('click', () => zoom(1.25, vista.w / 2, vista.h / 2));
    $('btn-zoom-menos').addEventListener('click', () => zoom(0.8, vista.w / 2, vista.h / 2));
    $('btn-enquadrar').addEventListener('click', enquadrar);

    document.addEventListener('keydown', (e) => {
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.target.closest && e.target.closest('input, select, textarea, button, summary')) return;
      if (e.key === ' ') alternar();
      else if (e.key === 'r' || e.key === 'R') { reiniciar(); sujo.series = true; }
      else return;
      e.preventDefault();
    });

    // tema: o do sistema ou o escolhido no atributo data-theme da raiz; e as fontes que chegam depois
    const redesenhar = () => { lerCores(); sujo.series = true; };
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', redesenhar);
    new MutationObserver(redesenhar).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    if (document.fonts) document.fonts.ready.then(redesenhar);
    new ResizeObserver(() => { sujo.series = true; }).observe(document.body);
  }

  const arred = (a) => Math.round(a * 100) / 100;   // no centímetro

  // Edição do caminho pelo clique em (px, py) na tela: perto de um waypoint (até 10 px), arrastar o
  // move, e apagar (Shift+clique ou botão direito) o tira, se sobrarem dois; perto de um trecho (até
  // 8 px), insere um waypoint nele; longe de tudo, acrescenta um no fim. O arraste fica em est.edicao.
  function pegarCaminho(px, py, apagar) {
    const W = copiarCaminho(est.caminho);
    const tela = W.map((p) => paraTela(p[0], p[1]));
    let i = -1, melhor = 10;
    tela.forEach(([x, y], n) => { const d = Math.hypot(x - px, y - py); if (d <= melhor) { melhor = d; i = n; } });
    if (apagar) {
      if (i >= 0 && W.length > 2) { W.splice(i, 1); definirCaminho(W); }
      return;
    }
    if (i < 0) {
      let perto = 8;
      for (let n = 1; n < tela.length; n++) {
        const d = distanciaSegmento(px, py, tela[n - 1], tela[n]);
        if (d <= perto) { perto = d; i = n; }
      }
      if (i >= 0) W.splice(i, 0, paraMundo(px, py));
      else { W.push(paraMundo(px, py)); i = W.length - 1; }
    }
    est.edicao = { tipo: 'caminho', W, i, antes: JSON.stringify(est.caminho) };
  }

  // distância, em pixels, de (px, py) ao segmento de a a b
  function distanciaSegmento(px, py, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const t = limitar(((px - a[0]) * dx + (py - a[1]) * dy) / (dx * dx + dy * dy || 1), 0, 1);
    return Math.hypot(a[0] + t * dx - px, a[1] + t * dy - py);
  }

  function zoom(fator, px, py) {
    const [wx, wy] = paraMundo(px, py);
    vista.esc = limitar(vista.esc * fator, 8, 4000);
    vista.x = wx - (px - vista.w / 2) / vista.esc;
    vista.y = wy + (py - vista.h / 2) / vista.esc;
    vista.roboVisivel = roboDentro();
  }

  // ---------------------------------------------------------------- início

  lerCores();
  ligar();
  // o algoritmo pode vir na URL (?algoritmo=ponto), que é como a página de escolha abre esta
  const pedido = new URLSearchParams(location.search).get('algoritmo');
  if (pedido && pedido !== 'pose' && PRIMEIRO[pedido]) {
    const ex = EXEMPLOS[PRIMEIRO[pedido]];
    est.P0 = ex.P0.slice();
    if (ex.alvo) est.alvo = ex.alvo.slice();
    if (ex.caminho) est.caminho = copiarCaminho(ex.caminho);
    mostrarLei(pedido);
  } else mostrarLei('pose');
  definirFerramenta('alvo');
  definirDt(est.dt);
  escreverGanhos();
  escreverPoses();
  escreverDerivados();
  escreverOdometria();
  el.modoDif.checked = est.ativo.diferencial;
  el.modoInc.checked = est.ativo.incremental;
  reiniciar();
  requestAnimationFrame(quadro);

  // ganchos para inspeção pelo console (e para os testes no navegador)
  window.simulador = { est, vista, reiniciar, recomecar, definirGanhos, definirLei, definirCaminho, aplicarExemplo, exemploOdometria, enquadrar, paraTela, paraMundo, passo };
})();
