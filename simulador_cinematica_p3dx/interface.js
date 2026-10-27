/*
 * interface.js: estado, tempo, desenho e eventos do simulador.
 *
 * Toda a física está em cinematica.js. As velocidades das rodas são a fonte da
 * verdade (são os atuadores). O seletor de entrada escolhe quem o usuário comanda:
 * no modo 1, φ̇_L e φ̇_R, e v e ω saem da cinemática direta; no modo 2, v e ω, e as
 * rodas saem da inversa. A inversa usa o raio nominal (quem comanda não sabe da
 * diferença de diâmetro); a direta usa os raios reais, então com rodas diferentes o
 * robô executa um v e um ω diferentes do comando.
 * Os dois modelos (e o arco exato) recebem sempre os mesmos v e ω a cada passo Δt,
 * então podem ser comparados a qualquer momento.
 */
(function () {
  'use strict';

  const C = window.Cinematica;
  const $ = (id) => document.getElementById(id);
  const RAD = Math.PI / 180;
  const GRAU = 180 / Math.PI;
  const JANELA = 20;            // segundos mostrados no histórico
  const LIMITE_RASTRO = 80000;  // pontos guardados por trajetória
  const MODELOS = ['diferencial', 'incremental'];
  const COR_MODELO = { diferencial: 'dif', incremental: 'inc', exato: 'exato' };
  const PADRAO = {
    D: C.P3DX.diametroRoda / 1000,
    L: C.P3DX.distanciaRodas / 1000,
    wmax: 360 * RAD,
  };

  const est = {
    D: PADRAO.D,          // diâmetro nominal da roda [m]
    L: PADRAO.L,          // distância entre as rodas, L = 2l [m]
    dD: 0,                // slider de diferença [m]: < 0 aumenta a esquerda, > 0 aumenta a direita
    wmax: PADRAO.wmax,    // limite de cada roda [rad/s]
    dt: 0.1, escala: 1, tmax: 0, tLimite: Infinity,
    wl: 0, wr: 0, v: 0, w: 0,
    entrada: 'rodas',     // 'rodas' (modo 1) ou 'robo' (modo 2)
    vCmd: 0, wCmd: 0,     // v e ω que as rodas dão com o raio nominal: o comando, no modo 2
    ativo: { diferencial: true, incremental: false },
    exibir: { exato: false, icc: true, amostras: true, seguir: false },
    rodando: true, fim: false, t: 0, k: 0, acumulado: 0,
    pose: {}, rastro: {}, ang: { e: 0, d: 0 }, hist: [],
    saturouAte: 0,
  };
  const vista = { x: 1.2, y: 0.7, esc: 120, w: 0, h: 0, iniciada: false, roboVisivel: true, ultimoK: -1 };
  const sujo = { planos: true, series: true };
  let cor = {};
  const fonte = { ui: 'system-ui, sans-serif', mat: 'Cambria, Georgia, serif' };
  let hudAnterior = '';
  let arraste = null;

  const el = {
    modoDif: $('modo-dif'), modoInc: $('modo-inc'),
    entradaRodas: $('entrada-rodas'), entradaRobo: $('entrada-robo'), entradaSec: $('entrada-sec'),
    blocoRodas: $('bloco-rodas'), blocoRobo: $('bloco-robo'), papelRodas: $('papel-rodas'), papelRobo: $('papel-robo'),
    papelPlanoRodas: $('papel-plano-rodas'), papelPlanoRobo: $('papel-plano-robo'),
    planosSub: $('planos-sub'), historicoSub: $('historico-sub'), series: $('series'), dica: $('dica-teclado'),
    rodar: $('btn-rodar'), reiniciar: $('btn-reiniciar'), limpar: $('btn-limpar'),
    wl: $('wl'), wlNum: $('wl-num'), wlSec: $('wl-sec'),
    wr: $('wr'), wrNum: $('wr-num'), wrSec: $('wr-sec'),
    v: $('v'), vNum: $('v-num'), vSec: $('v-sec'),
    w: $('w'), wNum: $('w-num'), wSec: $('w-sec'),
    aviso: $('aviso-saturacao'),
    mundo: $('mundo'), hud: $('hud'),
    dif: $('dif-diametro'), difSec: $('dif-sec'),
    pD: $('p-diametro'), pL: $('p-distancia'), pWmax: $('p-wmax'), pDerivados: $('p-derivados'),
    pDt: $('p-dt'), pDtNum: $('p-dt-num'), pEscala: $('p-escala'), pTmax: $('p-tmax'),
    pX0: $('p-x0'), pY0: $('p-y0'), pTh0: $('p-th0'),
    exExato: $('ex-exato'), exIcc: $('ex-icc'), exAmostras: $('ex-amostras'), exSeguir: $('ex-seguir'),
    planoRodas: $('plano-rodas'), planoRobo: $('plano-robo'), matrizes: $('matrizes'),
    serieRodas: $('serie-rodas'), serieV: $('serie-v'), serieW: $('serie-w'), serieErro: $('serie-erro'),
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
  function anguloTexto(th) {
    const g = th * GRAU;
    const r = ((g + 180) % 360 + 360) % 360 - 180;
    return Math.abs(g) > 180 ? `${num(g, 1)}° (≡ ${num(r, 1)}°)` : `${num(g, 1)}°`;
  }

  function lerCores() {
    const s = getComputedStyle(document.documentElement);
    for (const nome of ['texto', 'texto-suave', 'borda', 'painel', 'fundo-cena', 'grade', 'grade-forte',
      'dif', 'dif-corpo', 'inc', 'inc-corpo', 'exato', 'wl', 'wr', 'v', 'w',
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
    const fonte = ctx.font;
    const m = /(\d+(?:\.\d+)?)px/.exec(fonte);
    const tam = m ? parseFloat(m[1]) : 12;
    const fonteIndice = fonte.replace(/(\d+(?:\.\d+)?)px/, `${Math.round(tam * 0.78)}px`);
    const partes = s.split(/(_[A-Za-z0-9])/).filter(Boolean)
      .map((p) => (p.length === 2 && p[0] === '_' ? { t: p[1], indice: true } : { t: p, indice: false }));
    let largura = 0;
    for (const p of partes) {
      ctx.font = p.indice ? fonteIndice : fonte;
      p.w = ctx.measureText(p.t).width;
      largura += p.w;
    }
    if (!soMedir) {
      const alinhamento = ctx.textAlign;
      ctx.textAlign = 'left';
      let px = alinhar === 'center' ? x - largura / 2 : alinhar === 'right' ? x - largura : x;
      for (const p of partes) {
        ctx.font = p.indice ? fonteIndice : fonte;
        ctx.fillText(p.t, px, p.indice ? y + tam * 0.28 : y);
        px += p.w;
      }
      ctx.textAlign = alinhamento;
    }
    ctx.font = fonte;
    return largura;
  }
  const larguraIndice = (ctx, s) => textoIndice(ctx, s, 0, 0, 'left', true);

  // ---------------------------------------------------------------- geometria e comandos

  // Raios de cada roda: o slider de diferença aumenta só a roda do lado para onde é arrastado
  function geometria() {
    const DL = est.D + Math.max(0, -est.dD);
    const DR = est.D + Math.max(0, est.dD);
    return { DL, DR, rl: DL / 2, rr: DR / 2, l: est.L / 2 };
  }

  function limites() {
    const { rl, rr, l } = geometria();
    return {
      vmax: ((rl + rr) / 2) * est.wmax,          // as duas rodas no limite, para a frente
      wmaxRobo: ((rl + rr) / (2 * l)) * est.wmax, // uma roda em +limite e a outra em −limite
    };
  }

  function definirRodas(wl, wr, origem) {
    const s = C.saturar(wl, wr, est.wmax);
    if (s.saturou) est.saturouAte = performance.now() + 2500;
    est.wl = s.wl;
    est.wr = s.wr;
    // o comando é o v e o ω que essas rodas dariam com o raio nominal (rodas iguais)
    const r = est.D / 2;
    const q = C.direta(est.wl, est.wr, r, r, est.L / 2);
    est.vCmd = q.v;
    est.wCmd = q.w;
    recalcularRobo(origem);
  }

  // Inversa com o raio nominal: quem comanda v e ω não sabe da diferença de diâmetro
  function definirRobo(v, w, origem) {
    const r = est.D / 2;
    const q = C.inversa(v, w, r, r, est.L / 2);
    definirRodas(q.wl, q.wr, origem);
  }

  // Depois de mudar diâmetro, distância ou limite: mantém o que o usuário comanda no modo atual
  function reaplicar(origem) {
    if (est.entrada === 'robo') definirRobo(est.vCmd, est.wCmd, origem);
    else definirRodas(est.wl, est.wr, origem);
  }

  // Mantém as rodas e recalcula v e ω (usado também quando mudam diâmetro ou distância)
  function recalcularRobo(origem) {
    const { rl, rr, l } = geometria();
    const r = C.direta(est.wl, est.wr, rl, rr, l);
    est.v = r.v;
    est.w = r.w;
    sujo.planos = true;
    atualizarControles(origem);
  }

  // Escreve um valor num controle, sem atrapalhar o controle que o usuário está mexendo
  function escrever(e, valor, origem, casas) {
    if (e === origem) return;
    if (e.type === 'number' && e === document.activeElement) return;
    e.value = casas === undefined ? String(valor) : valor.toFixed(casas);
  }

  function atualizarControles(origem) {
    const { rl, rr, l } = geometria();
    const { vmax, wmaxRobo } = limites();
    const wmaxG = Math.round(est.wmax * GRAU);
    // faixas múltiplas do passo, para que o zero caia exatamente num ponto do slider
    const vLim = Math.ceil(vmax * 1000) / 1000;
    const wLim = Math.ceil(wmaxRobo * GRAU * 10) / 10;
    for (const s of [el.wl, el.wr]) { s.min = -wmaxG; s.max = wmaxG; }
    el.v.min = -vLim; el.v.max = vLim;
    el.w.min = -wLim; el.w.max = wLim;

    // no modo 2 os controles de v e ω mostram o comando; no modo 1, o que o robô executa
    const comando = est.entrada === 'robo';
    const vMostra = comando ? est.vCmd : est.v, wMostra = comando ? est.wCmd : est.w;
    escrever(el.wl, est.wl * GRAU, origem);
    escrever(el.wr, est.wr * GRAU, origem);
    escrever(el.v, vMostra, origem);
    escrever(el.w, wMostra * GRAU, origem);
    escrever(el.wlNum, est.wl * GRAU, origem, 1);
    escrever(el.wrNum, est.wr * GRAU, origem, 1);
    escrever(el.vNum, vMostra, origem, 3);
    escrever(el.wNum, wMostra * GRAU, origem, 1);

    el.wlSec.innerHTML = `${num(est.wl, 3)} rad/s · v<sub>L</sub> = r<sub>L</sub> φ̇<sub>L</sub> = ${num(rl * est.wl, 3)} m/s`;
    el.wrSec.innerHTML = `${num(est.wr, 3)} rad/s · v<sub>R</sub> = r<sub>R</sub> φ̇<sub>R</sub> = ${num(rr * est.wr, 3)} m/s`;
    if (comando && est.dD !== 0) {
      el.vSec.textContent = `executado: v = ${num(est.v, 3)} m/s (raios diferentes)`;
      el.wSec.textContent = `executado: ω = ${num(est.w * GRAU, 1)} °/s (raios diferentes)`;
    } else {
      el.vSec.textContent = `Δs = v·Δt = ${num(est.v * est.dt * 1000, 1)} mm por passo`;
      el.wSec.textContent = `${num(est.w, 3)} rad/s · Δθ = ω·Δt = ${num(est.w * est.dt * GRAU, 2)}° por passo`;
    }

    const { DL, DR } = geometria();
    const pct = (d) => (d === est.D ? '' : ` (+${num((d - est.D) * 1000, 1)} mm)`);
    let efeito = 'rodas iguais';
    if (DL !== DR) {
      // com φ̇_L = φ̇_R o robô anda em círculo de raio R = l (r_L + r_R) / (r_R − r_L)
      const R = (l * (rl + rr)) / (rr - rl);
      efeito = `com φ̇<sub>L</sub> = φ̇<sub>R</sub> o robô curva para a ${R > 0 ? 'esquerda' : 'direita'}, R = ${num(Math.abs(R), 2)} m`;
    }
    el.difSec.innerHTML = `D<sub>L</sub> = ${num(DL * 1000, 1)} mm${pct(DL)} · D<sub>R</sub> = ${num(DR * 1000, 1)} mm${pct(DR)} · ${efeito}`;
    el.pDerivados.textContent = `r = ${num(est.D * 500, 2)} mm · l = ${num(est.L * 500, 2)} mm · ` +
      `v máx. = ${num(vmax, 3)} m/s · ω máx. = ${num(wmaxRobo * GRAU, 1)} °/s`;

    atualizarMatrizes();
  }

  function atualizarMatrizes() {
    const { rl, rr, l } = geometria();
    const r = est.D / 2;
    // matrizes numéricas têm células de largura mínima, para não "pularem" enquanto os valores mudam
    const mat = (celulas, cols) =>
      `<div class="mat${celulas.every((c) => /^[−\d.,]+$/.test(c)) ? ' num' : ''}" style="--cols:${cols}">` +
      `${celulas.map((c) => `<span>${c}</span>`).join('')}</div>`;
    el.matrizes.innerHTML = `
      <div class="mat-bloco">
        <div class="mat-titulo">direta: rodas → robô</div>
        <div class="mat-eq">${mat(['v', 'ω'], 1)} = ${mat(['r<sub>L</sub>/2', 'r<sub>R</sub>/2', '−r<sub>L</sub>/2l', 'r<sub>R</sub>/2l'], 2)} ${mat(['φ̇<sub>L</sub>', 'φ̇<sub>R</sub>'], 1)}</div>
        <div class="mat-eq">${mat([num(est.v, 3), num(est.w, 3)], 1)} = ${mat([num(rl / 2, 4), num(rr / 2, 4), num(-rl / (2 * l), 4), num(rr / (2 * l), 4)], 2)} ${mat([num(est.wl, 3), num(est.wr, 3)], 1)}</div>
      </div>
      <div class="mat-bloco">
        <div class="mat-titulo">inversa: robô → rodas (raio nominal r)</div>
        <div class="mat-eq">${mat(['φ̇<sub>L</sub>', 'φ̇<sub>R</sub>'], 1)} = ${mat(['1/r', '−l/r', '1/r', 'l/r'], 2)} ${mat(['v', 'ω'], 1)}</div>
        <div class="mat-eq">${mat([num(est.wl, 3), num(est.wr, 3)], 1)} = ${mat([num(1 / r, 3), num(-l / r, 3), num(1 / r, 3), num(l / r, 3)], 2)} ${mat([num(est.vCmd, 3), num(est.wCmd, 3)], 1)}</div>
      </div>
      <div class="mat-titulo">v em m/s; ω e φ̇ em rad/s; r e l em m${est.dD !== 0 ? '<br>raios diferentes: a direta usa r<sub>L</sub> e r<sub>R</sub> reais, a inversa o r nominal' : ''}</div>`;
  }

  // ---------------------------------------------------------------- simulação

  function lerPose0() {
    return [lerNumero(el.pX0) || 0, lerNumero(el.pY0) || 0, (lerNumero(el.pTh0) || 0) * RAD];
  }

  // Duração no mesmo espírito do laço dos slides: round(tmax/Δt) + 1 passos
  function atualizarLimiteTempo() {
    est.tLimite = est.tmax > 0 ? C.passosDoLaco(est.tmax, est.dt) * est.dt : Infinity;
  }

  function reiniciar() {
    const P0 = lerPose0();
    for (const m of ['diferencial', 'incremental', 'exato']) {
      est.pose[m] = P0.slice();
      est.rastro[m] = [[P0[0], P0[1]]];
    }
    est.t = 0;
    est.k = 0;
    est.acumulado = 0;
    est.ang = { e: 0, d: 0 };
    est.hist = [];
    est.fim = false;
    atualizarLimiteTempo();
    registrarHistorico(true);
    vista.roboVisivel = true;
    atualizarBotaoRodar();
  }

  function passo() {
    for (const m of ['diferencial', 'incremental', 'exato']) {
      const anterior = est.pose[m];
      est.pose[m] = C.PASSOS[m](anterior, est.v, est.w, est.dt);
      registrarRastro(m, anterior);
    }
    est.ang.e += est.wl * est.dt;
    est.ang.d += est.wr * est.dt;
    est.k += 1;
    est.t += est.dt;
    registrarHistorico(false);
    if (est.t >= est.tLimite - est.dt / 2) {
      est.rodando = false;
      est.fim = true;
      atualizarBotaoRodar();
    }
  }

  function registrarRastro(m, anterior) {
    const r = est.rastro[m];
    const P = est.pose[m];
    if (m === 'exato') {
      // o arco exato é desenhado curvo: pontos intermediários a cada ~2° de giro
      const n = Math.min(64, Math.ceil(Math.abs(est.w * est.dt) / (2 * RAD)));
      for (let i = 1; i < n; i++) {
        const Q = C.passoExato(anterior, est.v, est.w, (est.dt * i) / n);
        r.push([Q[0], Q[1]]);
      }
    }
    if (est.dt < 0.01) {
      // passos muito curtos: guarda um ponto a cada 2 mm andados
      const u = r[r.length - 1];
      if (u && Math.hypot(P[0] - u[0], P[1] - u[1]) < 0.002) return;
    }
    r.push([P[0], P[1]]);
    if (r.length > LIMITE_RASTRO) r.splice(0, LIMITE_RASTRO / 10); // corta em blocos: splice a cada passo seria O(n)
  }

  function registrarHistorico(forcar) {
    const h = est.hist;
    const u = h[h.length - 1];
    if (!forcar && u && est.t - u.t < 0.02) return;
    const ex = est.pose.exato;
    const desvio = (m) => Math.hypot(est.pose[m][0] - ex[0], est.pose[m][1] - ex[1]);
    h.push({ t: est.t, wl: est.wl, wr: est.wr, v: est.v, w: est.w, eDif: desvio('diferencial'), eInc: desvio('incremental') });
    while (h.length > 2 && h[1].t < est.t - JANELA - 1) h.shift();
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
    if (sujo.planos) { desenharPlanos(); sujo.planos = false; }
    if (sujo.series) { desenharSeries(); sujo.series = false; }
    el.aviso.hidden = performance.now() > est.saturouAte;
    requestAnimationFrame(quadro);
  }

  // ---------------------------------------------------------------- cena (robô no plano)

  const paraTela = (x, y) => [vista.w / 2 + (x - vista.x) * vista.esc, vista.h / 2 - (y - vista.y) * vista.esc];
  const paraMundo = (px, py) => [vista.x + (px - vista.w / 2) / vista.esc, vista.y - (py - vista.h / 2) / vista.esc];

  function enquadrarCaixa(x0, x1, y0, y1) {
    const margem = 0.35;
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
    for (const m of [...MODELOS, 'exato']) {
      if (m === 'exato' ? !est.exibir.exato : !est.ativo[m]) continue;
      for (const p of est.rastro[m]) incluir(p[0], p[1]);
      incluir(est.pose[m][0], est.pose[m][1]);
    }
    if (!Number.isFinite(x0)) incluir(0, 0);
    enquadrarCaixa(x0, x1, y0, y1);
  }

  function modeloPrincipal() {
    return MODELOS.find((m) => est.ativo[m]) || null;
  }

  function roboDentro() {
    const m = modeloPrincipal();
    if (!m) return true;
    const [px, py] = paraTela(est.pose[m][0], est.pose[m][1]);
    return px >= 40 && px <= vista.w - 40 && py >= 40 && py <= vista.h - 40;
  }

  // Câmera: segue o robô, ou só o mantém à vista quando ele sai pela borda
  function ajustarCamera() {
    const m = modeloPrincipal();
    if (!m || arraste) return;
    const P = est.pose[m];
    if (est.exibir.seguir) {
      vista.x += (P[0] - vista.x) * 0.2;
      vista.y += (P[1] - vista.y) * 0.2;
      return;
    }
    if (est.k === vista.ultimoK) return;
    vista.ultimoK = est.k;
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
      enquadrarCaixa(-0.6, 3.0, -0.6, 2.2);
      vista.iniciada = true;
    }
    ajustarCamera();

    desenharGrade(ctx, w, h);
    if (est.exibir.exato) desenharRastro(ctx, est.rastro.exato, cor.exato, [7, 5], false);
    // os pontos de cada passo só fazem sentido enquanto o rastro guarda todos os passos (Δt ≥ 0,01 s)
    const pontos = est.exibir.amostras && est.dt >= 0.01;
    for (const m of MODELOS) if (est.ativo[m]) desenharRastro(ctx, est.rastro[m], cor[COR_MODELO[m]], null, pontos);

    const principal = modeloPrincipal();
    if (principal && est.exibir.icc) desenharPrevisao(ctx, est.pose[principal]);
    if (est.exibir.exato) desenharRobo(ctx, est.pose.exato, 'exato');
    // o diferencial por cima, como nos slides (robô amarelo)
    for (const m of ['incremental', 'diferencial']) if (est.ativo[m]) desenharRobo(ctx, est.pose[m], m);
    if (principal && est.exibir.icc) desenharVetores(ctx, est.pose[principal]);
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
    const eixo = Math.max(0.5, 50 / vista.esc); // passa da borda do robô parado na origem
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

  function desenharRobo(ctx, P, modelo) {
    const { DL, DR, l } = geometria();
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
    const fantasma = modelo === 'exato';
    const corCorpo = modelo === 'incremental' ? cor['inc-corpo'] : cor['dif-corpo'];

    // corpo do P3-DX (slide 26)
    poligono(C.P3DX.corpo[0].map((v) => v / 1000), C.P3DX.corpo[1].map((v) => v / 1000));
    ctx.lineWidth = 1.2;
    if (fantasma) {
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = cor.exato;
    } else {
      ctx.fillStyle = corCorpo;
      ctx.fill();
      ctx.strokeStyle = cor.texto;
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // rodas: comprimento = diâmetro de cada uma, centradas em y = ±l; esquerda e direita nas cores de φ̇_L e φ̇_R
    const lw = C.P3DX.larguraRoda / 1000;
    for (const [lado, D, ang, corRoda] of [[1, DL, est.ang.e, cor.wl], [-1, DR, est.ang.d, cor.wr]]) {
      const yc = lado * l;
      poligono([D / 2, D / 2, -D / 2, -D / 2], [yc - lw / 2, yc + lw / 2, yc + lw / 2, yc - lw / 2]);
      if (fantasma) {
        ctx.strokeStyle = cor.exato;
        ctx.stroke();
        continue;
      }
      ctx.fillStyle = corRoda;
      ctx.fill();
      ctx.strokeStyle = cor.texto;
      ctx.stroke();
      if (vista.esc * D < 28) continue;
      // faixas da banda de rodagem: andam junto com a superfície da roda (r·φ)
      const espaco = D / 4;
      const desl = (((ang * D) / 2) % espaco + espaco) % espaco;
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
    ctx.strokeStyle = fantasma ? cor.exato : cor.orientacao;
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy); ctx.stroke();
    ctx.fillStyle = fantasma ? cor.exato : cor.orientacao;
    ctx.beginPath(); ctx.arc(cx, cy, 3.5, 0, 2 * Math.PI); ctx.fill();
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

  // ICC, eixo das rodas até ele e a circunferência que o robô percorre com os comandos atuais
  function desenharPrevisao(ctx, P) {
    const c = C.icc(P, est.v, est.w);
    if (!c || Math.abs(c.R) > 500) return;
    const { l } = geometria();
    const [ix, iy] = paraTela(c.x, c.y);
    ctx.strokeStyle = cor.w;
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.45;
    ctx.setLineDash([3, 5]);
    ctx.beginPath();
    ctx.arc(ix, iy, Math.abs(c.R) * vista.esc, 0, 2 * Math.PI);
    ctx.stroke();
    // eixo das rodas prolongado até o ICC (as duas rodas e o ponto médio estão nessa reta)
    const lat = Math.abs(c.R - l) > Math.abs(c.R + l) ? l : -l; // roda mais distante do ICC
    const [rx, ry] = paraTela(P[0] - lat * Math.sin(P[2]), P[1] + lat * Math.cos(P[2]));
    ctx.globalAlpha = 0.85;
    ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.moveTo(ix, iy); ctx.lineTo(rx, ry); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.fillStyle = cor.w;
    ctx.beginPath(); ctx.arc(ix, iy, 4.5, 0, 2 * Math.PI); ctx.fill();
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillText('ICC', ix + 7, iy - 6);
  }

  // Vetores: v_L e v_R nas rodas, v no ponto médio (seta = deslocamento em 1 s) e o giro ω
  function desenharVetores(ctx, P) {
    const { rl, rr, l } = geometria();
    const th = P[2], c = Math.cos(th), s = Math.sin(th);
    const naLateral = (lat) => [P[0] - lat * s, P[1] + lat * c];
    const vL = rl * est.wl, vR = rr * est.wr;
    seta(ctx, naLateral(l), [vL * c, vL * s], cor.wl, 2.5);
    seta(ctx, naLateral(-l), [vR * c, vR * s], cor.wr, 2.5);
    seta(ctx, naLateral(0), [est.v * c, est.v * s], cor.v, 3);

    // pelas pontas das setas das rodas passa uma reta que cruza o ICC (triângulos semelhantes)
    const k = C.icc(P, est.v, est.w);
    if (k && Math.abs(k.R) < 500) {
      const lat = Math.abs(k.R - l) > Math.abs(k.R + l) ? l : -l;
      const vlat = lat === l ? vL : vR;
      const [px0, py0] = paraTela(k.x, k.y);
      const ponta = naLateral(lat);
      const [px1, py1] = paraTela(ponta[0] + vlat * c, ponta[1] + vlat * s);
      ctx.strokeStyle = cor['texto-suave'];
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 4]);
      ctx.beginPath(); ctx.moveTo(px0, py0); ctx.lineTo(px1, py1); ctx.stroke();
      ctx.setLineDash([]);
    }

    if (Math.abs(est.w) > 1e-4) {
      // arco em volta do robô, com o sentido de ω (anti-horário = positivo)
      const [cx, cy] = paraTela(P[0], P[1]);
      const raio = Math.max(0.34 * vista.esc, 26);
      const varre = limitar(est.w / 1.5, -1, 1) * Math.PI * 0.85;
      const a0 = -(th + Math.PI / 2); // começa do lado esquerdo do robô
      const a1 = a0 - varre;          // y da tela é invertido
      ctx.strokeStyle = cor.w;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, raio, a0, a1, varre > 0);
      ctx.stroke();
      const ex = cx + raio * Math.cos(a1), ey = cy + raio * Math.sin(a1);
      const tx = varre > 0 ? Math.sin(a1) : -Math.sin(a1);
      const ty = varre > 0 ? -Math.cos(a1) : Math.cos(a1);
      ctx.fillStyle = cor.w;
      ctx.beginPath();
      ctx.moveTo(ex + tx * 8, ey + ty * 8);
      ctx.lineTo(ex - ty * 5, ey + tx * 5);
      ctx.lineTo(ex + ty * 5, ey - tx * 5);
      ctx.closePath();
      ctx.fill();
      ctx.font = `italic 600 14px ${fonte.mat}`;
      ctx.fillText('ω', ex + 8, ey - 6);
    }
  }

  function atualizarHud() {
    const fim = est.fim ? `<span class="alerta">fim (${num(est.tmax, 1)} s)</span>` : '';
    const linhas = [`<div class="linha"><b>t = ${num(est.t, 2)} s</b><span class="suave">passo ${est.k} · Δt = ${num(est.dt, 3)} s</span>${fim}</div>`];
    const linhaPose = (m, nome) => {
      const [x, y, th] = est.pose[m];
      return `<div class="linha"><span class="marca" style="background:${cor[COR_MODELO[m]]}"></span>` +
        `${nome}: x = ${num(x, 3)} m · y = ${num(y, 3)} m · θ = ${anguloTexto(th)}</div>`;
    };
    if (est.ativo.diferencial) linhas.push(linhaPose('diferencial', 'diferencial'));
    if (est.ativo.incremental) linhas.push(linhaPose('incremental', 'incremental'));
    if (est.exibir.exato) linhas.push(linhaPose('exato', 'arco exato'));
    if (!est.ativo.diferencial && !est.ativo.incremental) linhas.push('<div class="linha suave">Marque um dos modelos no topo.</div>');
    let curva;
    if (Math.abs(est.w) < 1e-9) curva = Math.abs(est.v) < 1e-9 ? 'parado' : 'linha reta (ω = 0, R = ∞)';
    else if (Math.abs(est.v) < 1e-9) curva = 'gira no lugar (v = 0, R = 0)';
    else curva = `R = v/ω = ${num(est.v / est.w, 3)} m`;
    if (est.ativo.diferencial && est.ativo.incremental) {
      const a = est.pose.diferencial, b = est.pose.incremental;
      curva += ` · entre os modelos: ${num(Math.hypot(a[0] - b[0], a[1] - b[1]) * 1000, 1)} mm`;
    }
    linhas.push(`<div class="linha suave">${curva}</div>`);
    const html = linhas.join('');
    if (html !== hudAnterior) {
      el.hud.innerHTML = html;
      hudAnterior = html;
    }
  }

  // ---------------------------------------------------------------- gráfico dinâmico: direta ⇄ inversa

  const MARGEM_PLANO = { esq: 54, dir: 12, topo: 14, base: 40 };

  // Sistema de eixos de um plano quadrado; fx e fy são meias-faixas nas unidades internas
  function plano(cv, fx, fy) {
    const c = prepararCanvas(cv);
    if (!c) return null;
    const m = MARGEM_PLANO;
    const W = c.w - m.esq - m.dir, H = c.h - m.topo - m.base;
    return Object.assign(c, {
      m, W, H, fx, fy,
      px: (x) => m.esq + ((x + fx) / (2 * fx)) * W,
      py: (y) => m.topo + ((fy - y) / (2 * fy)) * H,
      vx: (px) => ((px - m.esq) / W) * 2 * fx - fx,
      vy: (py) => fy - ((py - m.topo) / H) * 2 * fy,
    });
  }

  // Trecho da reta a·x + b·y = c dentro da caixa do plano
  function retaNaCaixa(p, a, b, c) {
    const pts = [];
    const dentro = (x, y) => x >= -p.fx - 1e-12 && x <= p.fx + 1e-12 && y >= -p.fy - 1e-12 && y <= p.fy + 1e-12;
    if (Math.abs(b) > 1e-15) for (const x of [-p.fx, p.fx]) { const y = (c - a * x) / b; if (dentro(x, y)) pts.push([x, y]); }
    if (Math.abs(a) > 1e-15) for (const y of [-p.fy, p.fy]) { const x = (c - b * y) / a; if (dentro(x, y)) pts.push([x, y]); }
    const unicos = pts.filter((q, i) => pts.findIndex((r) => Math.abs(r[0] - q[0]) + Math.abs(r[1] - q[1]) < 1e-9) === i);
    return unicos.length >= 2 ? unicos.slice(0, 2) : null;
  }

  function tracarReta(p, a, b, c, corR, tracejado, largura, rotulo) {
    const seg = retaNaCaixa(p, a, b, c);
    if (!seg) return;
    const { ctx } = p;
    ctx.strokeStyle = corR;
    ctx.lineWidth = largura;
    ctx.setLineDash(tracejado || []);
    ctx.beginPath();
    ctx.moveTo(p.px(seg[0][0]), p.py(seg[0][1]));
    ctx.lineTo(p.px(seg[1][0]), p.py(seg[1][1]));
    ctx.stroke();
    ctx.setLineDash([]);
    if (rotulo) {
      // rótulo a 80 % do caminho até a ponta mais alta, do lado em que a reta não passa
      const [s0, s1] = seg[0][1] <= seg[1][1] ? seg : [seg[1], seg[0]];
      const qx = p.px(s0[0] + 0.8 * (s1[0] - s0[0])), qy = p.py(s0[1] + 0.8 * (s1[1] - s0[1]));
      const subindoDireita = p.px(s1[0]) >= p.px(s0[0]);
      ctx.font = `11px ${fonte.ui}`;
      const larg = larguraIndice(ctx, rotulo);
      const x = limitar(subindoDireita ? qx + 8 : qx - 8 - larg, p.m.esq + 3, p.m.esq + p.W - larg - 3);
      const y = limitar(qy + 9, p.m.topo + 12, p.m.topo + p.H - 4);
      ctx.fillStyle = cor['fundo-cena'];
      ctx.globalAlpha = 0.85;
      ctx.fillRect(x - 2, y - 10, larg + 4, 14);
      ctx.globalAlpha = 1;
      ctx.fillStyle = corR;
      textoIndice(ctx, rotulo, x, y, 'left');
    }
  }

  // extremos: textos curtos nas pontas dos eixos ({ xMenos, xMais, yMenos, yMais }), ao lado dos títulos
  function moldura(p, titX, titY, ticksX, ticksY, extremos) {
    const { ctx, m, W, H } = p;
    ctx.font = `11px ${fonte.ui}`;
    ctx.lineWidth = 1;
    // grade
    ctx.strokeStyle = cor.grade;
    ctx.beginPath();
    for (const t of ticksX.valores) { const x = Math.round(p.px(t)) + 0.5; ctx.moveTo(x, m.topo); ctx.lineTo(x, m.topo + H); }
    for (const t of ticksY.valores) { const y = Math.round(p.py(t)) + 0.5; ctx.moveTo(m.esq, y); ctx.lineTo(m.esq + W, y); }
    ctx.stroke();
    // eixos em zero
    ctx.strokeStyle = cor['grade-forte'];
    ctx.beginPath();
    ctx.moveTo(m.esq, Math.round(p.py(0)) + 0.5); ctx.lineTo(m.esq + W, Math.round(p.py(0)) + 0.5);
    ctx.moveTo(Math.round(p.px(0)) + 0.5, m.topo); ctx.lineTo(Math.round(p.px(0)) + 0.5, m.topo + H);
    ctx.stroke();
    ctx.strokeStyle = cor.borda;
    ctx.strokeRect(m.esq + 0.5, m.topo + 0.5, W, H);
    // valores
    ctx.fillStyle = cor['texto-suave'];
    ctx.textAlign = 'center';
    ticksX.valores.forEach((t, i) => ctx.fillText(ticksX.textos[i], p.px(t), m.topo + H + 13));
    ctx.textAlign = 'right';
    ticksY.valores.forEach((t, i) => ctx.fillText(ticksY.textos[i], m.esq - 5, p.py(t) + 4));
    // títulos dos eixos
    ctx.fillStyle = cor.texto;
    ctx.font = `12px ${fonte.ui}`;
    textoIndice(ctx, titX, m.esq + W / 2, m.topo + H + 31, 'center');
    ctx.save();
    ctx.translate(13, m.topo + H / 2);
    ctx.rotate(-Math.PI / 2);
    textoIndice(ctx, titY, 0, 0, 'center');
    if (extremos) {
      ctx.font = `11px ${fonte.ui}`;
      ctx.fillStyle = cor['texto-suave'];
      textoIndice(ctx, extremos.yMenos, -H / 2, 0, 'left');
      textoIndice(ctx, extremos.yMais, H / 2, 0, 'right');
    }
    ctx.restore();
    if (extremos) {
      ctx.font = `11px ${fonte.ui}`;
      ctx.fillStyle = cor['texto-suave'];
      textoIndice(ctx, extremos.xMenos, m.esq, m.topo + H + 31, 'left');
      textoIndice(ctx, extremos.xMais, m.esq + W, m.topo + H + 31, 'right');
    }
    ctx.textAlign = 'left';
  }

  // Marcas de eixo: passo "bonito" na unidade de exibição, convertido para a unidade interna
  function marcas(meiaFaixa, fator, casasMin) {
    const passo = passoBonito((meiaFaixa * fator) / 3);
    const casas = Math.max(casasMin || 0, casasDoPasso(passo));
    const valores = [], textos = [];
    const n = Math.floor((meiaFaixa * fator) / passo + 1e-9);
    for (let i = -n; i <= n; i++) { valores.push((i * passo) / fator); textos.push(num(i * passo, casas)); }
    return { valores, textos };
  }

  function pontoAtual(p, x, y, texto) {
    const { ctx } = p;
    const px = p.px(limitar(x, -p.fx, p.fx)), py = p.py(limitar(y, -p.fy, p.fy));
    ctx.fillStyle = cor.texto;
    ctx.strokeStyle = cor.painel;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, 6.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.font = `600 11px ${fonte.ui}`;
    const larg = larguraIndice(ctx, texto);
    let tx = px + 10, ty = py - 10;
    if (tx + larg > p.w - p.m.dir - 2) tx = px - 10 - larg;
    if (tx < p.m.esq + 2) tx = p.m.esq + 2;
    if (ty < p.m.topo + 12) ty = py + 22;
    ctx.fillStyle = cor.painel;
    ctx.globalAlpha = 0.88;
    ctx.fillRect(tx - 3, ty - 11, larg + 6, 16);
    ctx.globalAlpha = 1;
    ctx.fillStyle = cor.texto;
    textoIndice(ctx, texto, tx, ty, 'left');
  }

  function rotuloCanto(p, x, y, texto, alinhar) {
    const { ctx } = p;
    ctx.font = `11px ${fonte.ui}`;
    ctx.fillStyle = cor['texto-suave'];
    textoIndice(ctx, texto, x, y, alinhar);
  }

  function desenharPlanos() {
    const { rl, rr, l } = geometria();
    const { vmax, wmaxRobo } = limites();

    // --- espaço das rodas: x = φ̇_L, y = φ̇_R (rad/s por dentro, °/s nos rótulos)
    const f = est.wmax * 1.15;
    const a = plano(el.planoRodas, f, f);
    if (a) {
      const t = marcas(f, GRAU);
      moldura(a, 'φ̇_L  (°/s)', 'φ̇_R  (°/s)', t, t);
      const { ctx } = a;
      // região alcançável: |φ̇| ≤ limite
      ctx.fillStyle = cor.destaque;
      ctx.globalAlpha = 0.07;
      ctx.fillRect(a.px(-est.wmax), a.py(est.wmax), a.px(est.wmax) - a.px(-est.wmax), a.py(-est.wmax) - a.py(est.wmax));
      ctx.globalAlpha = 1;
      ctx.strokeStyle = cor.destaque;
      ctx.lineWidth = 1;
      ctx.strokeRect(a.px(-est.wmax), a.py(est.wmax), a.px(est.wmax) - a.px(-est.wmax), a.py(-est.wmax) - a.py(est.wmax));
      // retas especiais: ω = 0 (reta) e v = 0 (gira no lugar)
      tracarReta(a, -rl, rr, 0, cor['texto-suave'], null, 1.2, 'ω = 0: reta');
      tracarReta(a, rl, rr, 0, cor['texto-suave'], null, 1.2, 'v = 0: gira no lugar');
      // pelo ponto atual: v constante e ω constante (as "coordenadas" do robô vistas no plano das rodas)
      tracarReta(a, rl, rr, rl * est.wl + rr * est.wr, cor.v, [6, 4], 1.6);
      tracarReta(a, -rl, rr, -rl * est.wl + rr * est.wr, cor.w, [6, 4], 1.6);
      // nos cantos do quadrado as duas rodas estão no limite; os rótulos ficam por fora, longe das diagonais
      const xe = a.px(-est.wmax), xd = a.px(est.wmax), yc = a.py(est.wmax), yb = a.py(-est.wmax);
      rotuloCanto(a, xd - 6, yc - 6, 'frente', 'right');
      rotuloCanto(a, xe + 6, yb + 15, 'ré', 'left');
      rotuloCanto(a, xe + 6, yc - 6, '↺ gira à esquerda', 'left');
      rotuloCanto(a, xd - 6, yb + 15, 'gira à direita ↻', 'right');
      pontoAtual(a, est.wl, est.wr, `v = ${num(est.v, 3)} m/s · ω = ${num(est.w * GRAU, 1)} °/s`);
    }

    // --- espaço do robô: x = ω, y = v
    const fw = wmaxRobo * 1.15, fv = vmax * 1.15;
    const b = plano(el.planoRobo, fw, fv);
    if (b) {
      moldura(b, 'ω  (°/s)', 'v  (m/s)', marcas(fw, GRAU), marcas(fv, 1, 2),
        { xMenos: '↻ direita', xMais: 'esquerda ↺', yMenos: 'ré', yMais: 'frente' });
      const { ctx } = b;
      // região alcançável: imagem do quadrado das rodas pela cinemática direta (um paralelogramo)
      const cantos = [[1, 1], [-1, 1], [-1, -1], [1, -1]].map(([sl, sr]) => C.direta(sl * est.wmax, sr * est.wmax, rl, rr, l));
      ctx.beginPath();
      cantos.forEach((q, i) => { const x = b.px(q.w), y = b.py(q.v); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.closePath();
      ctx.fillStyle = cor.destaque;
      ctx.globalAlpha = 0.07;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = cor.destaque;
      ctx.lineWidth = 1;
      ctx.stroke();
      // φ̇_L = 0 (v = ωl, pivô na roda esquerda) e φ̇_R = 0 (v = −ωl)
      tracarReta(b, -l, 1, 0, cor['texto-suave'], null, 1.2, 'φ̇_L = 0');
      tracarReta(b, l, 1, 0, cor['texto-suave'], null, 1.2, 'φ̇_R = 0');
      // pelo ponto atual: φ̇_L constante (v − ωl = r_L φ̇_L) e φ̇_R constante (v + ωl = r_R φ̇_R)
      tracarReta(b, -l, 1, rl * est.wl, cor.wl, [6, 4], 1.6);
      tracarReta(b, l, 1, rr * est.wr, cor.wr, [6, 4], 1.6);
      // modo 2 com rodas diferentes: anel vazio no comando, ligado ao ponto que o robô executa
      if (est.entrada === 'robo' && (Math.abs(est.vCmd - est.v) > 1e-9 || Math.abs(est.wCmd - est.w) > 1e-9)) {
        const cx = b.px(limitar(est.wCmd, -fw, fw)), cy = b.py(limitar(est.vCmd, -fv, fv));
        ctx.strokeStyle = cor['texto-suave'];
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(b.px(limitar(est.w, -fw, fw)), b.py(limitar(est.v, -fv, fv))); ctx.stroke();
        ctx.setLineDash([]);
        ctx.strokeStyle = cor.texto;
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(cx, cy, 6.5, 0, 2 * Math.PI); ctx.stroke();
      }
      pontoAtual(b, est.w, est.v, `φ̇_L = ${num(est.wl * GRAU, 1)} · φ̇_R = ${num(est.wr * GRAU, 1)} °/s`);
    }
  }

  // Arrastar o ponto: no plano das rodas é a direta (modo 1), no plano do robô é a inversa (modo 2)
  function ligarPlano(cv, aoMover) {
    let ativo = false;
    const mover = (e) => {
      const r = cv.getBoundingClientRect();
      aoMover(e.clientX - r.left, e.clientY - r.top, cv.clientWidth, cv.clientHeight);
    };
    cv.addEventListener('pointerdown', (e) => { ativo = true; cv.setPointerCapture(e.pointerId); mover(e); });
    cv.addEventListener('pointermove', (e) => { if (ativo) mover(e); });
    const soltar = () => { ativo = false; };
    cv.addEventListener('pointerup', soltar);
    cv.addEventListener('pointercancel', soltar);
  }

  function coordPlano(px, py, w, h, fx, fy) {
    const m = MARGEM_PLANO;
    const W = w - m.esq - m.dir, H = h - m.topo - m.base;
    return [limitar(((px - m.esq) / W) * 2 * fx - fx, -fx, fx), limitar(fy - ((py - m.topo) / H) * 2 * fy, -fy, fy)];
  }

  ligarPlano(el.planoRodas, (px, py, w, h) => {
    if (est.entrada !== 'rodas') return;
    const f = est.wmax * 1.15;
    let [wl, wr] = coordPlano(px, py, w, h, f, f);
    // ímã de 2 % nas retas especiais: fica fácil cair em "reta" ou "gira no lugar"
    const ima = 0.02 * f;
    if (Math.abs(wl - wr) < ima) wl = wr = (wl + wr) / 2;
    else if (Math.abs(wl + wr) < ima) { const m = (wr - wl) / 2; wl = -m; wr = m; }
    if (Math.abs(wl) < ima) wl = 0;
    if (Math.abs(wr) < ima) wr = 0;
    definirRodas(wl, wr, 'plano');
  });

  ligarPlano(el.planoRobo, (px, py, w, h) => {
    if (est.entrada !== 'robo') return;
    const { vmax, wmaxRobo } = limites();
    let [w_, v] = coordPlano(px, py, w, h, wmaxRobo * 1.15, vmax * 1.15);
    if (Math.abs(w_) < 0.02 * wmaxRobo) w_ = 0;
    if (Math.abs(v) < 0.02 * vmax) v = 0;
    definirRobo(v, w_, 'plano');
  });

  // ---------------------------------------------------------------- histórico no tempo

  function serie(cv, titulo, linhas, opcoes) {
    const c = prepararCanvas(cv);
    if (!c) return;
    const { ctx, w, h } = c;
    const m = { esq: 50, dir: 10, topo: 24, base: 20 };
    const W = w - m.esq - m.dir, H = h - m.topo - m.base;
    const t1 = Math.max(est.t, JANELA), t0 = t1 - JANELA;
    const dados = est.hist.filter((p) => p.t >= t0 - 0.05);
    let lo = Infinity, hi = -Infinity;
    for (const L of linhas) for (const p of dados) { const y = L.valor(p); lo = Math.min(lo, y); hi = Math.max(hi, y); }
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
    const px = (t) => m.esq + ((t - t0) / JANELA) * W;
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
    for (let s = Math.ceil(t0 / 5) * 5; s <= t1 + 1e-9; s += 5) {
      const x = Math.round(px(s)) + 0.5;
      ctx.moveTo(x, m.topo); ctx.lineTo(x, m.topo + H);
      ctx.fillText(`${num(s, 0)} s`, x, h - 5);
    }
    ctx.stroke();
    if (lo < 0 && hi > 0) {
      ctx.strokeStyle = cor['grade-forte'];
      ctx.beginPath(); ctx.moveTo(m.esq, Math.round(py(0)) + 0.5); ctx.lineTo(m.esq + W, Math.round(py(0)) + 0.5); ctx.stroke();
    }

    // curvas
    ctx.save();
    ctx.beginPath(); ctx.rect(m.esq, m.topo, W, H); ctx.clip();
    for (const L of linhas) {
      ctx.strokeStyle = L.cor;
      ctx.lineWidth = 2;
      ctx.beginPath();
      dados.forEach((p, i) => { const x = px(p.t), y = py(L.valor(p)); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    }
    ctx.restore();

    // título e legenda com o valor atual
    ctx.textAlign = 'left';
    ctx.font = `600 12px ${fonte.ui}`;
    ctx.fillStyle = cor.texto;
    ctx.fillText(titulo, m.esq, 15);
    let x = m.esq + ctx.measureText(titulo).width + 14;
    ctx.font = `11px ${fonte.ui}`;
    const ultimo = dados[dados.length - 1];
    for (const L of linhas) {
      const texto = `${L.nome}${ultimo ? ' = ' + num(L.valor(ultimo), opcoes.casas) : ''}`;
      ctx.fillStyle = L.cor;
      ctx.fillRect(x, 7, 10, 10);
      ctx.fillStyle = cor.texto;
      x += textoIndice(ctx, texto, x + 14, 15, 'left') + 28;
    }
  }

  function desenharSeries() {
    serie(el.serieRodas, 'Rodas (°/s)', [
      { nome: 'φ̇_L', cor: cor.wl, valor: (p) => p.wl * GRAU },
      { nome: 'φ̇_R', cor: cor.wr, valor: (p) => p.wr * GRAU },
    ], { minFaixa: 20, casas: 1 });
    serie(el.serieV, 'Velocidade linear (m/s)', [
      { nome: 'v', cor: cor.v, valor: (p) => p.v },
    ], { minFaixa: 0.1, casas: 3 });
    serie(el.serieW, 'Velocidade angular (°/s)', [
      { nome: 'ω', cor: cor.w, valor: (p) => p.w * GRAU },
    ], { minFaixa: 10, casas: 1 });
    const erros = [];
    if (est.ativo.diferencial) erros.push({ nome: 'diferencial', cor: cor.dif, valor: (p) => p.eDif * 1000 });
    if (est.ativo.incremental) erros.push({ nome: 'incremental', cor: cor.inc, valor: (p) => p.eInc * 1000 });
    serie(el.serieErro, 'Distância ao arco exato (mm)', erros, { minFaixa: 1, casas: 2, base0: true });
  }

  // ---------------------------------------------------------------- exemplos dos slides

  const EXEMPLOS = {
    slide27: { wl: 60, wr: 70, dt: 0.1, dif: true, inc: false, exato: false },
    slide48: { wl: 50, wr: 80, dt: 0.2, dif: false, inc: true, exato: false },
    slide55: { wl: 50, wr: 80, dt: 1.0, dif: true, inc: true, exato: true },
  };

  function aplicarExemplo(nome) {
    const e = EXEMPLOS[nome];
    restaurarP3DX();
    el.pX0.value = 0; el.pY0.value = 0; el.pTh0.value = 0;
    definirDt(e.dt);
    est.tmax = 30;
    el.pTmax.value = 30;
    est.ativo.diferencial = el.modoDif.checked = e.dif;
    est.ativo.incremental = el.modoInc.checked = e.inc;
    est.exibir.exato = el.exExato.checked = e.exato;
    definirEntrada('rodas'); // os slides comandam as rodas
    reiniciar();
    definirRodas(e.wl * RAD, e.wr * RAD, null);
    // enquadra a trajetória que vai ser percorrida
    const { rl, rr, l } = geometria();
    const n = C.passosDoLaco(est.tmax, est.dt);
    let x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    for (const m of ['diferencial', 'incremental']) {
      for (const P of C.simular(m, [0, 0, 0], est.wl, est.wr, rl, rr, l, est.dt, n).R) {
        x0 = Math.min(x0, P[0]); x1 = Math.max(x1, P[0]); y0 = Math.min(y0, P[1]); y1 = Math.max(y1, P[1]);
      }
    }
    enquadrarCaixa(x0, x1, y0, y1);
    est.rodando = true;
    atualizarBotaoRodar();
    sujo.series = true;
  }

  // ---------------------------------------------------------------- parâmetros

  function restaurarP3DX() {
    est.D = PADRAO.D;
    est.L = PADRAO.L;
    est.wmax = PADRAO.wmax;
    est.dD = 0;
    el.pD.value = PADRAO.D * 1000;
    el.pL.value = PADRAO.L * 1000;
    el.pWmax.value = Math.round(PADRAO.wmax * GRAU);
    el.dif.value = 0;
    reaplicar(null);
  }

  // ---------------------------------------------------------------- modo de entrada

  function definirEntrada(modo) {
    est.entrada = modo;
    const m1 = modo === 'rodas';
    el.entradaRodas.checked = m1;
    el.entradaRobo.checked = !m1;
    // o grupo de saída só mostra o valor calculado
    for (const e of [el.wl, el.wr, el.wlNum, el.wrNum]) e.disabled = !m1;
    for (const e of [el.v, el.w, el.vNum, el.wNum]) e.disabled = m1;
    el.blocoRodas.classList.toggle('saida', !m1);
    el.blocoRobo.classList.toggle('saida', m1);
    el.planoRodas.classList.toggle('saida', !m1);
    el.planoRobo.classList.toggle('saida', m1);
    const papel = (e, ehEntrada) => { e.textContent = ehEntrada ? 'entrada' : 'saída'; e.classList.toggle('e', ehEntrada); };
    papel(el.papelRodas, m1);
    papel(el.papelRobo, !m1);
    papel(el.papelPlanoRodas, m1);
    papel(el.papelPlanoRobo, !m1);
    el.entradaSec.innerHTML = m1
      ? 'Você comanda φ̇<sub>L</sub> e φ̇<sub>R</sub>; v e ω saem da cinemática direta (slides 15 a 17).'
      : 'Você comanda v e ω; φ̇<sub>L</sub> e φ̇<sub>R</sub> saem da cinemática inversa (slide 18).';
    el.planosSub.textContent = m1 ? 'arraste o ponto no espaço das rodas' : 'arraste o ponto no espaço do robô';
    // histórico: o modo 1 mostra v e ω; o modo 2, as rodas; o erro dos modelos fica nos dois
    el.serieRodas.hidden = m1;
    el.serieV.hidden = !m1;
    el.serieW.hidden = !m1;
    el.series.classList.toggle('modo-rodas', m1);
    el.historicoSub.innerHTML = m1 ? 'últimos 20 s · saídas do modo 1: v e ω' : 'últimos 20 s · saídas do modo 2: φ̇<sub>L</sub> e φ̇<sub>R</sub>';
    el.dica.innerHTML = (m1
      ? 'Teclado: <kbd>↑</kbd><kbd>↓</kbd> mudam as duas rodas juntas, <kbd>←</kbd><kbd>→</kbd> uma contra a outra, '
      : 'Teclado: <kbd>↑</kbd><kbd>↓</kbd> mudam v, <kbd>←</kbd><kbd>→</kbd> mudam ω, ') +
      '<kbd>espaço</kbd> pausa, <kbd>0</kbd> para, <kbd>R</kbd> reinicia.';
    atualizarControles(null);
    sujo.planos = sujo.series = true;
  }

  // Δt: slider em escala logarítmica de 0,001 s a 2 s
  const DT_MIN = 0.001, DT_MAX = 2;
  const dtDoSlider = (u) => Number((DT_MIN * Math.pow(DT_MAX / DT_MIN, u / 100)).toPrecision(2));
  const sliderDoDt = (dt) => (100 * Math.log(dt / DT_MIN)) / Math.log(DT_MAX / DT_MIN);
  function definirDt(dt, origem) {
    est.dt = limitar(dt, DT_MIN, DT_MAX);
    if (origem !== el.pDt) el.pDt.value = sliderDoDt(est.dt);
    if (origem !== el.pDtNum) el.pDtNum.value = est.dt;
    atualizarLimiteTempo();
    atualizarControles(null);
  }

  // ---------------------------------------------------------------- eventos

  function ligar() {
    for (const e of [el.entradaRodas, el.entradaRobo]) e.addEventListener('change', () => { if (e.checked) definirEntrada(e.value); });
    el.modoDif.addEventListener('change', () => { est.ativo.diferencial = el.modoDif.checked; sujo.series = true; });
    el.modoInc.addEventListener('change', () => { est.ativo.incremental = el.modoInc.checked; sujo.series = true; });
    el.rodar.addEventListener('click', alternar);
    el.reiniciar.addEventListener('click', () => { reiniciar(); sujo.series = true; });
    el.limpar.addEventListener('click', () => {
      for (const m of ['diferencial', 'incremental', 'exato']) est.rastro[m] = [[est.pose[m][0], est.pose[m][1]]];
    });

    // rodas (direta)
    el.wl.addEventListener('input', () => definirRodas(Number(el.wl.value) * RAD, est.wr, el.wl));
    el.wr.addEventListener('input', () => definirRodas(est.wl, Number(el.wr.value) * RAD, el.wr));
    el.wlNum.addEventListener('input', () => { const x = lerNumero(el.wlNum); if (x !== null) definirRodas(x * RAD, est.wr, el.wlNum); });
    el.wrNum.addEventListener('input', () => { const x = lerNumero(el.wrNum); if (x !== null) definirRodas(est.wl, x * RAD, el.wrNum); });
    // robô (inversa)
    el.v.addEventListener('input', () => definirRobo(Number(el.v.value), est.wCmd, el.v));
    el.w.addEventListener('input', () => definirRobo(est.vCmd, Number(el.w.value) * RAD, el.w));
    el.vNum.addEventListener('input', () => { const x = lerNumero(el.vNum); if (x !== null) definirRobo(x, est.wCmd, el.vNum); });
    el.wNum.addEventListener('input', () => { const x = lerNumero(el.wNum); if (x !== null) definirRobo(est.vCmd, x * RAD, el.wNum); });
    // ao sair de um campo numérico, ele volta a mostrar o valor efetivo (que pode ter saturado)
    for (const e of [el.wlNum, el.wrNum, el.vNum, el.wNum]) e.addEventListener('change', () => atualizarControles(null));
    // idem para os sliders ao soltar (o de v ou ω pode ter pedido mais do que as rodas dão)
    for (const e of [el.wl, el.wr, el.v, el.w]) e.addEventListener('change', () => atualizarControles(null));

    $('btn-parar').addEventListener('click', () => definirRodas(0, 0, null));
    // "reta" iguala as rodas e "girar" as põe opostas (pelo raio nominal), nos dois modos
    $('btn-reta').addEventListener('click', () => definirRobo(Math.abs(est.vCmd) > 1e-6 ? est.vCmd : 0.2, 0, null));
    $('btn-girar').addEventListener('click', () => definirRobo(0, Math.abs(est.wCmd) > 1e-6 ? est.wCmd : 30 * RAD, null));

    // a diferença de diâmetro não muda as rodas comandadas, só o que o robô executa
    el.dif.addEventListener('input', () => { est.dD = Number(el.dif.value) / 1000; recalcularRobo(el.dif); });
    $('btn-dif-zerar').addEventListener('click', () => { el.dif.value = 0; est.dD = 0; recalcularRobo(null); });

    el.pD.addEventListener('input', () => { const x = lerNumero(el.pD); if (x !== null && x >= 20 && x <= 600) { est.D = x / 1000; reaplicar(el.pD); } });
    el.pL.addEventListener('input', () => { const x = lerNumero(el.pL); if (x !== null && x >= 50 && x <= 1500) { est.L = x / 1000; reaplicar(el.pL); } });
    el.pWmax.addEventListener('input', () => { const x = lerNumero(el.pWmax); if (x !== null && x >= 10 && x <= 1440) { est.wmax = x * RAD; reaplicar(el.pWmax); } });
    $('btn-p3dx').addEventListener('click', restaurarP3DX);

    el.pDt.addEventListener('input', () => definirDt(dtDoSlider(Number(el.pDt.value)), el.pDt));
    el.pDtNum.addEventListener('change', () => { const x = lerNumero(el.pDtNum); if (x !== null && x > 0) definirDt(x, el.pDtNum); });
    el.pEscala.addEventListener('change', () => { est.escala = Number(el.pEscala.value); });
    el.pTmax.addEventListener('change', () => {
      const x = lerNumero(el.pTmax);
      est.tmax = x !== null && x > 0 ? x : 0;
      est.fim = false;
      atualizarLimiteTempo();
      atualizarBotaoRodar();
    });

    el.exExato.addEventListener('change', () => { est.exibir.exato = el.exExato.checked; });
    el.exIcc.addEventListener('change', () => { est.exibir.icc = el.exIcc.checked; });
    el.exAmostras.addEventListener('change', () => { est.exibir.amostras = el.exAmostras.checked; });
    el.exSeguir.addEventListener('change', () => { est.exibir.seguir = el.exSeguir.checked; vista.roboVisivel = true; });
    document.querySelectorAll('[data-exemplo]').forEach((b) => b.addEventListener('click', () => aplicarExemplo(b.dataset.exemplo)));

    // vista: arrastar move, roda do mouse dá zoom no ponto do cursor
    el.mundo.addEventListener('pointerdown', (e) => {
      arraste = { x: e.clientX, y: e.clientY, vx: vista.x, vy: vista.y };
      el.mundo.setPointerCapture(e.pointerId);
      el.mundo.classList.add('arrastando');
    });
    el.mundo.addEventListener('pointermove', (e) => {
      if (!arraste) return;
      vista.x = arraste.vx - (e.clientX - arraste.x) / vista.esc;
      vista.y = arraste.vy + (e.clientY - arraste.y) / vista.esc;
    });
    const soltar = () => {
      if (!arraste) return;
      arraste = null;
      el.mundo.classList.remove('arrastando');
      vista.roboVisivel = roboDentro();
    };
    el.mundo.addEventListener('pointerup', soltar);
    el.mundo.addEventListener('pointercancel', soltar);
    el.mundo.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = el.mundo.getBoundingClientRect();
      zoom(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    el.mundo.addEventListener('dblclick', enquadrar);
    $('btn-zoom-mais').addEventListener('click', () => zoom(1.25, vista.w / 2, vista.h / 2));
    $('btn-zoom-menos').addEventListener('click', () => zoom(0.8, vista.w / 2, vista.h / 2));
    $('btn-enquadrar').addEventListener('click', enquadrar);

    document.addEventListener('keydown', (e) => {
      if (e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.target.closest && e.target.closest('input, select, textarea, button, summary')) return;
      // modo 1: as setas mexem nas rodas (juntas ou uma contra a outra); modo 2: em v e ω
      const dv = 0.02, dw = 5 * RAD, dr = 5 * RAD;
      const m1 = est.entrada === 'rodas';
      switch (e.key) {
        case ' ': alternar(); break;
        case 'ArrowUp': if (m1) definirRodas(est.wl + dr, est.wr + dr, null); else definirRobo(est.vCmd + dv, est.wCmd, null); break;
        case 'ArrowDown': if (m1) definirRodas(est.wl - dr, est.wr - dr, null); else definirRobo(est.vCmd - dv, est.wCmd, null); break;
        case 'ArrowLeft': if (m1) definirRodas(est.wl - dr, est.wr + dr, null); else definirRobo(est.vCmd, est.wCmd + dw, null); break;
        case 'ArrowRight': if (m1) definirRodas(est.wl + dr, est.wr - dr, null); else definirRobo(est.vCmd, est.wCmd - dw, null); break;
        case '0': definirRodas(0, 0, null); break;
        case 'r': case 'R': reiniciar(); sujo.series = true; break;
        default: return;
      }
      e.preventDefault();
    });

    // tema: o do sistema ou o escolhido no atributo data-theme da raiz; e as fontes que chegam depois
    const redesenhar = () => { lerCores(); sujo.planos = sujo.series = true; };
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', redesenhar);
    new MutationObserver(redesenhar).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    if (document.fonts) document.fonts.ready.then(redesenhar);
    new ResizeObserver(() => { sujo.planos = sujo.series = true; }).observe(document.body);
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
  definirDt(est.dt);
  reiniciar();
  definirEntrada('rodas');
  definirRodas(60 * RAD, 70 * RAD, null); // abre andando, com as rodas do exemplo do slide 27
  atualizarBotaoRodar();
  requestAnimationFrame(quadro);

  // ganchos para inspeção pelo console (e para os testes no navegador)
  window.simulador = { est, vista, definirRodas, definirRobo, definirEntrada, aplicarExemplo, reiniciar, geometria };
})();
