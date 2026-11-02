/*
 * controladores.js: leis de controle do Corke aplicadas ao Pioneer P3-DX diferencial
 * ENE122 Robótica Móvel, Mini Desafio 01b (Corke, Robotics, Vision and Control, Seção 4.1.1)
 *
 * Quatro leis, com a mesma malha fechada: ir a uma pose (4.1.1.4, a principal), ir a um ponto
 * (4.1.1.1), seguir uma reta (4.1.1.2) e seguir um caminho por pure pursuit (4.1.1.3).
 *
 * Notação (a mesma de cinematica.js e do Corke):
 *   P = (x, y, θ)          pose do ponto médio do eixo das rodas
 *   alvo = (x*, y*, θ*)    pose que se quer alcançar; no ponto, θ* não conta; na reta, é um ponto
 *                          dela e a direção do percurso
 *   ρ, α, β                coordenadas polares do alvo vistas pelo robô (Fig. 4.13 do Corke)
 *   d                      distância com sinal até a reta, positiva com o robô à direita dela
 *   v, ω                   comandos do controlador; as rodas saem da cinemática inversa
 *
 * O Corke escreve as leis para o modelo de bicicleta, com o esterçamento γ = atan(ωL/v)
 * (eq. 4.8). No diferencial o comando já é (v, ω), então a lei entra direto em ω.
 *
 * Como cinematica.js, o arquivo não depende da interface: roda no navegador
 * (window.Controladores, depois de cinematica.js) e no Node (require), que é como os testes rodam.
 */
(function (raiz, fabrica) {
  if (typeof module === 'object' && module.exports) module.exports = fabrica(require('./cinematica.js'));
  else raiz.Controladores = fabrica(raiz.Cinematica);
})(typeof self !== 'undefined' ? self : this, function (C) {
  'use strict';

  // Operador ⊖ do Corke: diferença de ângulos levada a (−π, π]
  function angulo(a) {
    return Math.atan2(Math.sin(a), Math.cos(a));
  }

  // ---------------------------------------------------------------- driving to a configuration

  // Coordenadas polares (Fig. 4.13), já com a troca de coordenadas para um alvo qualquer:
  // ρ é a distância até o alvo, α o ângulo do vetor até o alvo medido a partir da frente do robô
  // e β = θ* − θ − α (com o alvo na origem e θ* = 0, é o β = −θ − α do livro)
  function polares(P, alvo) {
    const dx = alvo[0] - P[0], dy = alvo[1] - P[1];
    const alpha = angulo(Math.atan2(dy, dx) - P[2]);
    return { rho: Math.hypot(dx, dy), alpha, beta: angulo(alvo[2] - P[2] - alpha) };
  }

  // Estabilidade da malha fechada linear: kρ > 0, kβ < 0 e kα − kρ > 0
  function condicoesPose(k) {
    return [
      { texto: 'kρ > 0', ok: k.rho > 0 },
      { texto: 'kβ < 0', ok: k.beta < 0 },
      { texto: 'kα − kρ > 0', ok: k.alpha - k.rho > 0 },
    ];
  }

  // Condição "forte" de Siegwart e Nourbakhsh (Introduction to Autonomous Mobile Robots, 2004,
  // seção 3.6.2, eq. 3.65): com kρ > 0 e kβ < 0, kα + 5/3·kβ − 2/π·kρ > 0 garante que α não sai de
  // (−π/2, π/2] na aproximação, ou seja, que o robô não troca de sentido. Não decide a chegada: só se
  // o alvo pode passar para trás do robô no caminho. A margem é o lado esquerdo da desigualdade.
  function condicaoFortePose(k) {
    const margem = k.alpha + (5 / 3) * k.beta - (2 / Math.PI) * k.rho;
    return { texto: 'kα + 5/3 kβ − 2/π kρ > 0', margem, ok: k.rho > 0 && k.beta < 0 && margem > 0 };
  }

  // As equações polares supõem o alvo à frente, α ∈ (−π/2, π/2]. Com o alvo atrás, o Corke
  // dá ré negando v e γ, com o sentido escolhido pelo α inicial e mantido até o fim.
  function sentidoPose(P, alvo) {
    const { alpha } = polares(P, alvo);
    return alpha > -Math.PI / 2 && alpha <= Math.PI / 2 ? 1 : -1;
  }

  // Lei de controle v = kρ·ρ, ω = kα·α + kβ·β. De ré, é a mesma lei aplicada ao robô "virado"
  // (frente em θ + π, alvo em θ* + π) com v trocado de sinal: como γ = atan(ωL/v), negar v e γ
  // no modelo de bicicleta é o mesmo que negar v e manter ω no diferencial.
  function comandoPose(P, alvo, k, sentido) {
    const re = sentido < 0;
    const { rho, alpha, beta } = re
      ? polares([P[0], P[1], P[2] + Math.PI], [alvo[0], alvo[1], alvo[2] + Math.PI])
      : polares(P, alvo);
    return { v: sentido * k.rho * rho, w: k.alpha * alpha + k.beta * beta, rho, alpha, beta };
  }

  // Ganhos com que a interface abre (os testes conferem que chegam de todas as poses da grade)
  const GANHOS_POSE = { rho: 0.5, alpha: 1.5, beta: -0.5 };

  // ---------------------------------------------------------------- driving to a point (4.1.1.1)

  // v = Kv·ρ e γ = Kh(θ* ⊖ θ), com θ* = atan2(Δy, Δx). No diferencial o esterço γ vira ω, e
  // θ* ⊖ θ é o próprio α das coordenadas polares. Como v/ρ = Kv, a malha da direção fica
  // α̇ = Kv·sin α − Kh·α, sem ρ: α vai a zero se Kh > Kv. É a lei da pose com kβ = 0.
  function comandoPonto(P, alvo, k) {
    const { rho, alpha } = polares(P, alvo);
    return { v: k.v * rho, w: k.h * alpha, rho, alpha };
  }

  function condicoesPonto(k) {
    return [
      { texto: 'Kv > 0', ok: k.v > 0 },
      { texto: 'Kh > Kv', ok: k.h > k.v },
    ];
  }

  // Com 0 < Kh < Kv, α para onde Kv·sin α = Kh·α: o robô se aproxima em espiral, sem nunca apontar
  // para o alvo; abaixo de Kh/Kv = 2/π, o equilíbrio passa de 90° e a espiral é para fora.
  // Devolve esse α de equilíbrio (0 se Kh ≥ Kv; null fora de 0 < Kh, 0 < Kv).
  function equilibrioPonto(k) {
    if (!(k.v > 0 && k.h > 0)) return null;
    if (k.h >= k.v) return 0;
    let a = 1e-9, b = Math.PI;   // sin(α)/α cai de 1 a 0 em (0, π]: bisseção
    for (let i = 0; i < 60; i++) {
      const m = (a + b) / 2;
      if (Math.sin(m) / m > k.h / k.v) a = m; else b = m;
    }
    return (a + b) / 2;
  }

  const GANHOS_PONTO = { v: 0.5, h: 1.5 };   // os da Fig. 4.8

  // ---------------------------------------------------------------- driving along a line (4.1.1.2)

  // Reta ax + by + c = 0. O Corke toma a direção do percurso θ* = atan2(a, −b) e a distância
  // d = (a, b, c)·(x, y, 1)/√(a² + b²), positiva com o robô à direita do percurso; a lei
  // γ = Kd·d + Kh(θ* ⊖ θ) vira para a esquerda quando ele está à direita (Fig. 4.9; a Fig. 4.10
  // confirma o sentido). No diferencial, ω = Kd·d + Kh(θ* ⊖ θ), com v constante.
  // A interface marca a reta como uma pose: um ponto dela e a direção do percurso.
  function retaDaPose(A) {
    const a = Math.sin(A[2]), b = -Math.cos(A[2]);
    return [a, b, -(a * A[0] + b * A[1])];
  }

  function distanciaReta(P, L) {
    return (L[0] * P[0] + L[1] * P[1] + L[2]) / Math.hypot(L[0], L[1]);
  }

  const direcaoReta = (L) => Math.atan2(L[0], -L[1]);

  // k: { d: Kd, h: Kh, v: velocidade constante }
  function comandoReta(P, L, k) {
    const d = distanciaReta(P, L);
    const erro = angulo(direcaoReta(L) - P[2]);
    return { v: k.v, w: k.d * d + k.h * erro, d, erro };
  }

  function condicoesReta(k) {
    return [
      { texto: 'Kd > 0', ok: k.d > 0 },
      { texto: 'Kh > 0', ok: k.h > 0 },
      { texto: 'v > 0', ok: k.v > 0 },
    ];
  }

  // Perto da reta (sin e ≈ e), d̈ + Kh·ḋ + v·Kd·d = 0: um segundo grau com ωn = √(v·Kd) e
  // ζ = Kh / (2√(v·Kd)). Com ζ < 1 o robô passa da reta antes de assentar.
  function amortecimentoReta(k) {
    const wn = Math.sqrt(k.v * k.d);
    return { wn, zeta: k.h / (2 * wn) };
  }

  const GANHOS_RETA = { d: 0.5, h: 1, v: 0.5 };   // Kd e Kh da Fig. 4.10; com v = 0,5 m/s, ζ = 1

  // ---------------------------------------------------------------- driving along a path (4.1.1.3)

  // Pure pursuit: o caminho são N pontos p_i (aqui, os waypoints ligados por retas e amostrados a
  // cada 1 cm). j é o ponto do caminho mais perto do robô e k, o primeiro depois de j a pelo menos
  // d de p_j: o "ponto de perseguição", que anda junto com o robô. A lei é a do ponto com p_k como
  // alvo: v constante e ω = Kh(θ* ⊖ θ), θ* = atan2 até p_k. Quanto maior d, mais o robô corta as quinas.
  const PASSO_CAMINHO = 0.01;

  // W: waypoints [[x, y], ...]. Devolve os pontos P, o comprimento acumulado s e o total.
  function amostrarCaminho(W) {
    const P = [W[0].slice()], s = [0];
    for (let i = 1; i < W.length; i++) {
      const [x0, y0] = W[i - 1], [x1, y1] = W[i];
      const L = Math.hypot(x1 - x0, y1 - y0);
      const n = Math.max(1, Math.ceil(L / PASSO_CAMINHO));
      for (let m = 1; m <= n; m++) {
        P.push([x0 + ((x1 - x0) * m) / n, y0 + ((y1 - y0) * m) / n]);
        s.push(s[s.length - 1] + L / n);
      }
    }
    return { P, s, total: s[s.length - 1], n: P.length };
  }

  // O Corke procura j no caminho inteiro. Aqui, por padrão, a busca começa no j anterior e vai até
  // max(1 m, 2d) adiante: j nunca volta e não pula para outro trecho quando o caminho se cruza ou
  // passa perto de si mesmo (no cruzamento do "8", a busca global faz j saltar de ramo).
  function maisPerto(Pr, cam, jAnterior, d, global) {
    let i0 = 0, i1 = cam.n - 1;
    if (!global) {
      i0 = jAnterior;
      const limite = cam.s[jAnterior] + Math.max(1, 2 * d);
      while (i1 > i0 && cam.s[i1] > limite) i1--;
    }
    let j = i0, melhor = Infinity;
    for (let i = i0; i <= i1; i++) {
      const e = (cam.P[i][0] - Pr[0]) ** 2 + (cam.P[i][1] - Pr[1]) ** 2;
      if (e < melhor) { melhor = e; j = i; }
    }
    return j;
  }

  // k: o primeiro ponto depois de j a pelo menos d de p_j; no fim do caminho, o último ponto
  function perseguido(cam, j, d) {
    const [xj, yj] = cam.P[j];
    for (let i = j + 1; i < cam.n; i++) {
      if (Math.hypot(cam.P[i][0] - xj, cam.P[i][1] - yj) >= d) return i;
    }
    return cam.n - 1;
  }

  // k: { d: distância de perseguição, h: Kh, v: velocidade }. Quando o ponto perseguido chega ao fim,
  // v cai na proporção da distância ao fim, v·ρ/d: a chegada é a lei do ponto com Kv = v/d.
  function comandoCaminho(P, cam, j, k) {
    const kk = perseguido(cam, j, k.d);
    const [xk, yk] = cam.P[kk];
    const alpha = angulo(Math.atan2(yk - P[1], xk - P[0]) - P[2]);
    const fim = cam.P[cam.n - 1];
    const rhoFim = Math.hypot(fim[0] - P[0], fim[1] - P[1]);
    const v = kk === cam.n - 1 ? k.v * Math.min(1, rhoFim / k.d) : k.v;
    const desvio = Math.hypot(cam.P[j][0] - P[0], cam.P[j][1] - P[1]);
    return { v, w: k.h * alpha, alpha, j, k: kk, desvio, rhoFim, s: cam.s[j] };
  }

  function condicoesCaminho(k) {
    return [
      { texto: 'd > 0', ok: k.d > 0 },
      { texto: 'Kh > 0', ok: k.h > 0 },
      { texto: 'v > 0', ok: k.v > 0 },
    ];
  }

  const GANHOS_CAMINHO = { d: 0.5, h: 2, v: 0.4 };   // d da Fig. 4.12 na escala 1:10

  // Caminhos prontos (waypoints e pose inicial), os mesmos na página e nos testes
  const arred = (x) => Math.round(x * 100) / 100;
  const CAMINHOS = {
    // Fig. 4.12 na escala 1:10: waypoints (10, 10), (10, 60), (80, 80), (50, 10) e partida na origem
    corke: { W: [[1, 1], [1, 6], [8, 8], [5, 1]], P0: [0, 0, 0] },
    zigue: { W: [[0, 0], [1, 1], [2, 0], [3, 1], [4, 0], [5, 1]], P0: [0, 0, Math.PI / 4] },
    quadrado: { W: [[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]], P0: [0, 0, 0] },
    // lemniscata x = 2 sin t, y = sin 2t: começa e termina no cruzamento
    oito: { W: Array.from({ length: 25 }, (_, i) => { const t = (2 * Math.PI * i) / 24; return [arred(2 * Math.sin(t)), arred(Math.sin(2 * t))]; }), P0: [0, 0, Math.PI / 4] },
    esse: { W: Array.from({ length: 13 }, (_, i) => { const x = (2 * Math.PI * i) / 12; return [arred(0.8 * x), arred(Math.sin(x))]; }), P0: [0, 0, (50 * Math.PI) / 180] },
  };

  // O exemplo do Corke de cada lei: a página da simulação abre com ele e a página de escolha o mostra
  const GRAU = Math.PI / 180;
  const EXEMPLOS_CORKE = {
    pose: { P0: [9, 5, 0], alvo: [5, 5, 90 * GRAU] },             // alcançado de ré
    ponto: { P0: [8, 5, 90 * GRAU], alvo: [5, 5, 0] },            // uma das partidas da Fig. 4.8
    reta: { P0: [8, 5, 90 * GRAU], alvo: [4, 4, Math.atan2(1, 2)] },   // a reta (1, −2, 4) da Fig. 4.10
    caminho: { P0: CAMINHOS.corke.P0, caminho: CAMINHOS.corke.W },     // Fig. 4.12 na escala 1:10
  };

  // ---------------------------------------------------------------- as leis para a malha

  // Cada lei: iniciar (estado fixo da corrida), comando, erro (o que falta: d e θ, este null quando
  // a orientação não conta) e chegou. o.re === false proíbe a ré na pose.
  const LEIS = {
    pose: {
      iniciar: (P0, o) => ({ sentido: o.re === false ? 1 : sentidoPose(P0, o.alvo) }),
      comando: (P, o, e) => comandoPose(P, o.alvo, o.k, e.sentido),
      erro: (P, o) => erroAoAlvo(P, o.alvo),
      chegou: (P, o) => Math.hypot(o.alvo[0] - P[0], o.alvo[1] - P[1]) < o.tolRho &&
        Math.abs(angulo(o.alvo[2] - P[2])) < o.tolTh,
    },
    ponto: {
      iniciar: () => ({ sentido: 1 }),
      comando: (P, o) => comandoPonto(P, o.alvo, o.k),
      erro: (P, o) => ({ d: Math.hypot(o.alvo[0] - P[0], o.alvo[1] - P[1]), th: null }),
      chegou: (P, o) => Math.hypot(o.alvo[0] - P[0], o.alvo[1] - P[1]) < o.tolRho,
    },
    // a reta não tem fim: "chegar" é convergir, com |d| e o erro de direção dentro das tolerâncias
    reta: {
      iniciar: (P0, o) => ({ sentido: 1, reta: o.reta || retaDaPose(o.alvo) }),
      comando: (P, o, e) => comandoReta(P, e.reta, o.k),
      erro: (P, o, e) => ({ d: distanciaReta(P, e.reta), th: angulo(direcaoReta(e.reta) - P[2]) }),
      chegou: (P, o, e) => Math.abs(distanciaReta(P, e.reta)) < o.tolRho &&
        Math.abs(angulo(direcaoReta(e.reta) - P[2])) < o.tolTh,
    },
    // o caminho guarda j entre os passos: avancar() o atualiza para a pose do passo, antes do comando.
    // Chegar é estar a menos da tolerância do último ponto com j já no último trecho (a d do fim), para
    // um caminho fechado não "chegar" logo na partida.
    caminho: {
      iniciar: (P0, o) => {
        const cam = amostrarCaminho(o.caminho);
        return { sentido: 1, cam, j: maisPerto(P0, cam, 0, o.k.d, o.buscaGlobal) };
      },
      avancar: (P, o, e) => { e.j = maisPerto(P, e.cam, e.j, o.k.d, o.buscaGlobal); },
      comando: (P, o, e) => comandoCaminho(P, e.cam, e.j, o.k),
      erro: (P, o, e) => {
        const fim = e.cam.P[e.cam.n - 1];
        return { d: Math.hypot(fim[0] - P[0], fim[1] - P[1]), th: null };
      },
      chegou: (P, o, e) => {
        const fim = e.cam.P[e.cam.n - 1];
        return e.cam.s[e.j] >= e.cam.total - o.k.d && Math.hypot(fim[0] - P[0], fim[1] - P[1]) < o.tolRho;
      },
    },
  };

  // ---------------------------------------------------------------- malha fechada

  // Geometria nominal do P3-DX (cinematica.js) e o limite de roda com que o simulador da Aula 14 abre
  const PADRAO = {
    r: C.P3DX.diametroRoda / 2000,
    l: C.P3DX.distanciaRodas / 2000,
    wmax: (360 * Math.PI) / 180,
  };

  // o: { lei, alvo, reta, caminho, buscaGlobal, k, P0, re, modelo, dt, tmax, r, rl, rr, l, wmax, escL, escR, pulsos, tolRho, tolTh }
  // reta: (a, b, c), só para a lei da reta; sem ela, a reta sai do alvo (ponto e direção do percurso).
  // caminho: os waypoints, só para a lei do caminho; buscaGlobal: procurar j no caminho inteiro, como o Corke.
  // r é o raio nominal, o único que o controlador conhece; rl e rr são os raios reais (iguais a r, se
  // não forem dados); escL e escR, a fração do giro de cada roda que escorrega; pulsos, por volta do encoder.
  function opcoes(o) {
    return Object.assign({ modelo: 'diferencial', dt: 0.05, tmax: 60, tolRho: 0.01, tolTh: Math.PI / 180, escL: 0, escR: 0, pulsos: 500 },
      PADRAO, o);
  }

  // Da pose que o controlador vê até o movimento: a lei dá (v, ω); a inversa com o raio nominal dá as
  // rodas, que saturam em ±wmax mantendo o raio de curva; o chão recebe (1 − escorregamento) do giro
  // de cada roda, com os raios reais. vReal e wReal são o que o robô de fato executa.
  function rodas(P, o, estado) {
    const u = LEIS[o.lei].comando(P, o, estado);
    const ideal = C.inversa(u.v, u.w, o.r, o.r, o.l);
    const s = C.saturar(ideal.wl, ideal.wr, o.wmax);
    const real = C.direta(s.wl * (1 - o.escL), s.wr * (1 - o.escR), o.rl ?? o.r, o.rr ?? o.r, o.l);
    return Object.assign({ wl: s.wl, wr: s.wr, saturou: s.saturou, vReal: real.v, wReal: real.w }, u);
  }

  // Um passo da malha com a pose verdadeira: o controlador lê a pose do próprio modelo, que anda
  // pelo modelo escolhido. O comando fica constante durante o passo.
  // A interface chama esta função a cada Δt; simular() a repete até chegar ou acabar o tempo.
  function passoMalha(P, o, estado) {
    if (LEIS[o.lei].avancar) LEIS[o.lei].avancar(P, o, estado);
    const amostra = rodas(P, o, estado);
    return { P: C.PASSOS[o.modelo](P, amostra.vReal, amostra.wReal, o.dt), amostra };
  }

  function simular(o) {
    o = opcoes(o);
    const lei = LEIS[o.lei];
    const estado = lei.iniciar(o.P0, o);
    const passos = Math.round(o.tmax / o.dt);
    let P = o.P0.slice();
    const R = [P.slice()], amostras = [];
    let chegou = false, tChegada = null;
    for (let i = 0; i < passos; i++) {
      if (lei.chegou(P, o, estado)) { chegou = true; tChegada = i * o.dt; break; }
      const p = passoMalha(P, o, estado);
      amostras.push(Object.assign({ t: i * o.dt }, p.amostra));
      P = p.P;
      R.push(P.slice());
    }
    if (!chegou && lei.chegou(P, o, estado)) { chegou = true; tChegada = passos * o.dt; }
    return { R, amostras, chegou, tChegada, estado, P };
  }

  // ---------------------------------------------------------------- malha fechada pela odometria

  // Como no slide 43 da Aula 14: o robô real anda pelo arco exato com os raios reais e o
  // escorregamento; os encoders contam pulsos inteiros do giro de cada roda (escorregando ou não);
  // a odometria converte pulsos em distância pelo raio nominal e integra pelo modelo incremental.
  // O controlador só vê a estimativa, então "chega" quando a estimativa chega ao alvo.
  function inicioOdometria(P0) {
    return { real: P0.slice(), odo: P0.slice(), angL: 0, angR: 0, pL: 0, pR: 0 };
  }

  function passoMalhaOdometria(s, o, estado) {
    if (LEIS[o.lei].avancar) LEIS[o.lei].avancar(s.odo, o, estado);
    const amostra = rodas(s.odo, o, estado);
    const N = o.pulsos, porPulso = (2 * Math.PI / N) * o.r;
    const angL = s.angL + amostra.wl * o.dt, angR = s.angR + amostra.wr * o.dt;
    const nL = C.pulsos(angL, N), nR = C.pulsos(angR, N);
    return {
      s: {
        real: C.passoExato(s.real, amostra.vReal, amostra.wReal, o.dt),
        odo: C.passoOdometria(s.odo, (nL - s.pL) * porPulso, (nR - s.pR) * porPulso, o.l),
        angL, angR, pL: nL, pR: nR,
      },
      amostra,
    };
  }

  function simularOdometria(o) {
    o = opcoes(o);
    const lei = LEIS[o.lei];
    const estado = lei.iniciar(o.P0, o);
    const passos = Math.round(o.tmax / o.dt);
    let s = inicioOdometria(o.P0);
    const R = [s.real.slice()], O = [s.odo.slice()];
    let chegou = false, tChegada = null;
    for (let i = 0; i < passos; i++) {
      if (lei.chegou(s.odo, o, estado)) { chegou = true; tChegada = i * o.dt; break; }
      s = passoMalhaOdometria(s, o, estado).s;
      R.push(s.real.slice());
      O.push(s.odo.slice());
    }
    if (!chegou && lei.chegou(s.odo, o, estado)) { chegou = true; tChegada = passos * o.dt; }
    return { R, O, real: s.real, odo: s.odo, chegou, tChegada, estado };
  }

  // Quanto uma pose está longe do alvo: distância e orientação (entre −180° e 180°)
  function erroAoAlvo(P, alvo) {
    return { d: Math.hypot(alvo[0] - P[0], alvo[1] - P[1]), th: angulo(alvo[2] - P[2]) };
  }

  return {
    angulo, polares, condicoesPose, condicaoFortePose, sentidoPose, comandoPose, GANHOS_POSE,
    comandoPonto, condicoesPonto, equilibrioPonto, GANHOS_PONTO,
    retaDaPose, distanciaReta, direcaoReta, comandoReta, condicoesReta, amortecimentoReta, GANHOS_RETA,
    amostrarCaminho, maisPerto, perseguido, comandoCaminho, condicoesCaminho, GANHOS_CAMINHO, CAMINHOS,
    EXEMPLOS_CORKE, LEIS, PADRAO, opcoes, rodas, passoMalha, simular,
    inicioOdometria, passoMalhaOdometria, simularOdometria, erroAoAlvo,
  };
});
