/*
 * escolha.js: as miniaturas da página de escolha.
 *
 * Cada cartão roda o exemplo do Corke da sua lei (K.EXEMPLOS_CORKE) com os ganhos padrão, no modelo
 * diferencial, pela mesma simular() da simulação, e anima o robô ao longo da trajetória, em laço:
 * cada volta dura uns DURACAO segundos de tela (o fator de tempo aparece no canto) e para no fim
 * por PAUSA segundos. Com "reduzir movimento" no sistema, mostra só a trajetória inteira.
 */
(function () {
  'use strict';

  const C = window.Cinematica;
  const K = window.Controladores;
  const GANHOS = { pose: K.GANHOS_POSE, ponto: K.GANHOS_PONTO, reta: K.GANHOS_RETA, caminho: K.GANHOS_CAMINHO };
  const DURACAO = 6;      // segundos de tela por volta
  const PAUSA = 1.5;      // segundos parado no fim
  const MARGEM = 0.6;     // folga em volta da cena [m]
  const GRAU = 180 / Math.PI;
  const semMovimento = window.matchMedia('(prefers-reduced-motion: reduce)');
  const cor = {};
  let fonte = 'system-ui, sans-serif';

  function lerCores() {
    const s = getComputedStyle(document.documentElement);
    for (const nome of ['texto', 'texto-suave', 'grade', 'grade-forte', 'dif', 'dif-corpo', 'orientacao', 'alvo']) {
      cor[nome] = s.getPropertyValue('--' + nome).trim();
    }
    fonte = s.getPropertyValue('--fonte-ui').trim() || fonte;
  }

  const num = (x, casas) => x.toLocaleString('pt-BR', { minimumFractionDigits: casas, maximumFractionDigits: casas }).replace('-', '−');
  const coord = (x) => Number(x.toFixed(2)).toLocaleString('pt-BR').replace('-', '−');
  const pose = (P) => `(${coord(P[0])}, ${coord(P[1])}, ${coord(P[2] * GRAU)}°)`;

  // O que o exemplo é e como terminou, embaixo da miniatura
  function legenda(c) {
    const { lei, ex, s } = c;
    const t = `${num(s.tChegada, 2)} s`;
    if (lei === 'pose') return `De ${pose(ex.P0)} a ${pose(ex.alvo)}: chega em ${t}${s.estado.sentido < 0 ? ', de ré' : ''}.`;
    if (lei === 'ponto') return `De ${pose(ex.P0)} ao ponto (${coord(ex.alvo[0])}, ${coord(ex.alvo[1])}): chega em ${t}, virado para ${num(K.angulo(s.P[2]) * GRAU, 1)}°.`;
    if (lei === 'reta') return `De ${pose(ex.P0)} até a reta da Fig. 4.10: converge em ${t}.`;
    return `Waypoints da Fig. 4.12 na escala 1:10, ${num(s.estado.cam.total, 2)} m: chega ao fim em ${t}.`;
  }

  // A corrida de cada cartão e a caixa que a miniatura enquadra
  const cartoes = [...document.querySelectorAll('.lei-cartao')].map((el) => {
    const lei = el.dataset.lei, ex = K.EXEMPLOS_CORKE[lei];
    const o = K.opcoes(Object.assign({ lei, modelo: 'diferencial', k: GANHOS[lei] }, ex));
    const s = K.simular(o);
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    const incluir = (p) => { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); };
    s.R.forEach(incluir);
    if (lei === 'caminho') ex.caminho.forEach(incluir); else incluir(ex.alvo);
    const c = {
      el, lei, ex, o, s, cv: el.querySelector('canvas'), i: 0, inicio: null,
      caixa: [x0 - MARGEM, x1 + MARGEM, y0 - MARGEM, y1 + MARGEM],
      // fator de tempo: a volta inteira em uns DURACAO s de tela, nunca mais devagar que o tempo real
      fator: Math.max(1, s.tChegada / DURACAO),
    };
    el.querySelector('.legenda-mini').textContent = legenda(c);
    return c;
  });

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

  function desenhar(c) {
    const p = prepararCanvas(c.cv);
    if (!p) return;
    const { ctx, w, h } = p;
    const [x0, x1, y0, y1] = c.caixa;
    const esc = Math.min(w / (x1 - x0), h / (y1 - y0));
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const T = (x, y) => [w / 2 + (x - cx) * esc, h / 2 - (y - cy) * esc];

    // grade de 1 m (2 m se ficar apertada)
    const g = esc >= 14 ? 1 : 2;
    ctx.strokeStyle = cor.grade;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil((cx - w / 2 / esc) / g) * g; x <= cx + w / 2 / esc; x += g) { const px = Math.round(T(x, 0)[0]) + 0.5; ctx.moveTo(px, 0); ctx.lineTo(px, h); }
    for (let y = Math.ceil((cy - h / 2 / esc) / g) * g; y <= cy + h / 2 / esc; y += g) { const py = Math.round(T(0, y)[1]) + 0.5; ctx.moveTo(0, py); ctx.lineTo(w, py); }
    ctx.stroke();

    desenharAlvo(ctx, c, T, esc, w, h);

    // início
    const [sx, sy] = T(c.ex.P0[0], c.ex.P0[1]);
    ctx.strokeStyle = cor['texto-suave'];
    ctx.lineWidth = 1.8;
    ctx.beginPath(); ctx.arc(sx, sy, 4.5, 0, 2 * Math.PI); ctx.stroke();

    // rastro até o passo atual e o robô
    const R = c.s.R;
    ctx.strokeStyle = cor.dif;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let k = 0; k <= c.i; k++) { const [px, py] = T(R[k][0], R[k][1]); if (k) ctx.lineTo(px, py); else ctx.moveTo(px, py); }
    ctx.stroke();
    robo(ctx, R[c.i], T, cor['dif-corpo'], cor.texto, false);

    // tempo da corrida e o fator de tempo da animação
    ctx.font = `11px ${fonte}`;
    ctx.fillStyle = cor['texto-suave'];
    ctx.textAlign = 'right';
    const fator = c.fator > 1 && !semMovimento.matches ? ` · ×${num(c.fator, 1)}` : '';
    ctx.fillText(`t = ${num(c.i * c.o.dt, 1)} s${fator}`, w - 8, h - 8);
    ctx.textAlign = 'left';
  }

  // contorno do P3-DX (slide 26 da Aula 14) na pose P, cheio ou tracejado, com o traço da orientação
  function robo(ctx, P, T, preenchimento, borda, tracejado) {
    const [x, y, th] = P;
    const co = Math.cos(th), si = Math.sin(th);
    const M = (bx, by) => T(x + co * bx - si * by, y + si * bx + co * by);
    ctx.beginPath();
    C.P3DX.corpo[0].forEach((bx, k) => {
      const [a, b] = M(bx / 1000, C.P3DX.corpo[1][k] / 1000);
      if (k) ctx.lineTo(a, b); else ctx.moveTo(a, b);
    });
    ctx.closePath();
    ctx.lineWidth = 1.2;
    if (tracejado) ctx.setLineDash([3, 2]);
    else { ctx.fillStyle = preenchimento; ctx.fill(); }
    ctx.strokeStyle = borda;
    ctx.stroke();
    ctx.setLineDash([]);
    const [ax, ay] = M(0, 0), [bx, by] = M(0.3, 0);
    ctx.strokeStyle = tracejado ? borda : cor.orientacao;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  }

  // O alvo de cada lei: a pose final tracejada, a mira do ponto, a reta com o sentido, ou o caminho
  function desenharAlvo(ctx, c, T, esc, w, h) {
    const A = c.ex.alvo;
    ctx.strokeStyle = cor.alvo;
    ctx.fillStyle = cor.alvo;
    if (c.lei === 'pose') { robo(ctx, A, T, null, cor.alvo, true); return; }
    if (c.lei === 'ponto') {
      const [px, py] = T(A[0], A[1]);
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 7, 0, 2 * Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.arc(px, py, 2.5, 0, 2 * Math.PI); ctx.fill();
      return;
    }
    if (c.lei === 'reta') {
      const ux = Math.cos(A[2]), uy = Math.sin(A[2]);
      const meia = Math.hypot(w, h) / esc;
      const [ax, ay] = T(A[0] - meia * ux, A[1] - meia * uy), [bx, by] = T(A[0] + meia * ux, A[1] + meia * uy);
      ctx.lineWidth = 2;
      ctx.setLineDash([7, 5]);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.setLineDash([]);
      // setas no sentido do percurso, a cada 1,5 m a partir do ponto marcado
      const ex = ux, ey = -uy;
      for (let t = -Math.ceil(meia / 1.5) * 1.5; t <= meia; t += 1.5) {
        const [px, py] = T(A[0] + t * ux, A[1] + t * uy);
        ctx.beginPath();
        ctx.moveTo(px + 6 * ex, py + 6 * ey);
        ctx.lineTo(px - 4 * ex - 4 * ey, py - 4 * ey + 4 * ex);
        ctx.lineTo(px - 4 * ex + 4 * ey, py - 4 * ey - 4 * ex);
        ctx.closePath();
        ctx.fill();
      }
      return;
    }
    const W = c.ex.caminho.map((p) => T(p[0], p[1]));
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.globalAlpha = 0.5;
    ctx.beginPath();
    W.forEach(([x, y], k) => { if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.lineJoin = 'miter';
    for (const [x, y] of W) { ctx.beginPath(); ctx.arc(x, y, 3, 0, 2 * Math.PI); ctx.fill(); }
    const [fx, fy] = W[W.length - 1];
    ctx.lineWidth = 1.8;
    ctx.beginPath(); ctx.arc(fx, fy, 7, 0, 2 * Math.PI); ctx.stroke();
  }

  // Passo da animação: o tempo de tela vezes o fator dá o tempo da corrida, e dele sai o passo
  function quadro(agora) {
    for (const c of cartoes) {
      const ultimo = c.s.R.length - 1;
      if (semMovimento.matches) c.i = ultimo;
      else {
        if (c.inicio === null) c.inicio = agora;
        let t = ((agora - c.inicio) / 1000) * c.fator;
        if (t > c.s.tChegada + PAUSA * c.fator) { c.inicio = agora; t = 0; }
        c.i = Math.min(ultimo, Math.floor(t / c.o.dt + 1e-9));
      }
      desenhar(c);
    }
    requestAnimationFrame(quadro);
  }

  lerCores();
  const recolorir = () => lerCores();
  semMovimento.addEventListener('change', () => { for (const c of cartoes) c.inicio = null; });
  new MutationObserver(recolorir).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  if (document.fonts) document.fonts.ready.then(recolorir);
  requestAnimationFrame(quadro);

  // para inspeção pelo console e para os testes no navegador
  window.escolha = { cartoes };
})();
