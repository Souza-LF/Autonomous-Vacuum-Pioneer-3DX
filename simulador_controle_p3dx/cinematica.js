/*
 * cinematica.js: modelo cinemático do robô diferencial (Pioneer P3-DX)
 * ENE122 Robótica Móvel, Cinemática Partes A e B (Aulas 13 e 14, Prof. André Marcato)
 *
 * Notação dos slides:
 *   P = (x, y, θ)        pose do ponto médio do eixo das rodas no frame global
 *   r                    raio da roda
 *   l                    metade da distância entre as rodas (L = 2l)
 *   φ̇_l, φ̇_r (wl, wr)    velocidades angulares das rodas esquerda e direita [rad/s]
 *   v, ω (v, w)          velocidades linear e angular do ponto médio [m/s, rad/s]
 *
 * Generalização usada pelo slider de diferença de diâmetro: cada roda tem o seu
 * raio (rl, rr). Com rl = rr = r, todas as fórmulas voltam às dos slides 15 a 18.
 *
 * O arquivo não depende da interface: roda no navegador (window.Cinematica) e no
 * Node (require), que é como os testes o comparam com o código Python dos slides.
 */
(function (raiz, fabrica) {
  if (typeof module === 'object' && module.exports) module.exports = fabrica();
  else raiz.Cinematica = fabrica();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // Geometria do P3-DX copiada dos slides 26 e 27 (mm; x para a frente, y para a esquerda)
  const P3DX = {
    diametroRoda: 195.0,   // r = 0,5 · 195 mm
    distanciaRodas: 381.0, // l = 0,5 · 381 mm ("metade do eixo das rodas")
    larguraRoda: 40.0,     // as rodas do slide 26 vão de 170,5 a 210,5 mm em y
    corpo: [
      [100.0, 227.5, 227.5, 100.0, -200.0, -227.5, -227.5, -200.0],
      [-190.5, -50.0, 50.0, 190.5, 190.5, 163.0, -163.0, -190.5],
    ],
  };

  // Cinemática direta (slides 15 a 17): velocidades das rodas -> ponto médio
  function direta(wl, wr, rl, rr, l) {
    if (rl === rr) {
      // mesma ordem de operações do código dos slides (v = r/2·(wr+wl), w = r/(2l)·(wr−wl))
      const r = rl;
      return { v: r / 2 * (wr + wl), w: r / (2 * l) * (wr - wl) };
    }
    return { v: (rr * wr + rl * wl) / 2, w: (rr * wr - rl * wl) / (2 * l) };
  }

  // Cinemática inversa (slide 18): ponto médio -> velocidades das rodas
  function inversa(v, w, rl, rr, l) {
    return { wl: (v - w * l) / rl, wr: (v + w * l) / rr };
  }

  // Limita as rodas a ±wmax mantendo a razão wl/wr, de modo que o raio de curva R = v/ω não muda
  function saturar(wl, wr, wmax) {
    const m = Math.max(Math.abs(wl), Math.abs(wr));
    if (m <= wmax || m === 0) return { wl, wr, saturou: false };
    const k = wmax / m;
    return { wl: wl * k, wr: wr * k, saturou: true };
  }

  // Modelo diferencial integrado por Euler (slides 22 e 29)
  function passoDiferencial(P, v, w, dt) {
    const th = P[2];
    return [P[0] + v * Math.cos(th) * dt, P[1] + v * Math.sin(th) * dt, th + w * dt];
  }

  // Modelo incremental (slides 39 e 50): desloca Δs na orientação média θ + Δθ/2
  function passoIncremental(P, v, w, dt) {
    const th = P[2];
    const delta_s = v * dt;
    const delta_th = w * dt;
    return [
      P[0] + delta_s * Math.cos(th + delta_th / 2.0),
      P[1] + delta_s * Math.sin(th + delta_th / 2.0),
      th + delta_th,
    ];
  }

  // Referência que não está nos slides: com v e ω constantes no passo, o ponto médio
  // anda exatamente sobre um arco em torno do ICC, para qualquer Δt
  function passoExato(P, v, w, dt) {
    const th = P[2];
    const dth = w * dt;
    if (Math.abs(dth) < 1e-12) return passoDiferencial(P, v, w, dt);
    const R = v / w;
    return [
      P[0] + R * (Math.sin(th + dth) - Math.sin(th)),
      P[1] - R * (Math.cos(th + dth) - Math.cos(th)),
      th + dth,
    ];
  }

  const PASSOS = { diferencial: passoDiferencial, incremental: passoIncremental, exato: passoExato };

  // Encoder incremental: pulsos inteiros contados até o ângulo acumulado da roda, com N pulsos por volta.
  // A folga de 1e-9 evita que uma volta inteira calculada como 0,999999… perca o último pulso.
  function pulsos(angulo, N) {
    return Math.floor((angulo * N) / (2 * Math.PI) + 1e-9);
  }

  // Odometria (slide 43): distâncias percorridas pelas rodas -> Δs e Δθ -> pose pelo modelo incremental
  function passoOdometria(P, dsl, dsr, l) {
    const delta_s = (dsr + dsl) / 2;
    const delta_th = (dsr - dsl) / (2 * l);
    return [
      P[0] + delta_s * Math.cos(P[2] + delta_th / 2),
      P[1] + delta_s * Math.sin(P[2] + delta_th / 2),
      P[2] + delta_th,
    ];
  }

  // Centro instantâneo de curvatura (slides 10 e 11): a R = v/ω do ponto médio, à esquerda se ω > 0
  function icc(P, v, w) {
    if (Math.abs(w) < 1e-9) return null;
    const R = v / w;
    return { x: P[0] - R * Math.sin(P[2]), y: P[1] + R * Math.cos(P[2]), R };
  }

  // Iterações do laço `for t in np.arange(0.0, tmax + dt, dt)` dos slides 29 e 50
  function passosDoLaco(tmax, dt) {
    return Math.round(tmax / dt) + 1;
  }

  // Simulação em lote igual aos programas dos slides (usada nos testes e nos exemplos)
  function simular(modelo, P0, wl, wr, rl, rr, l, dt, passos) {
    const passo = PASSOS[modelo];
    const { v, w } = direta(wl, wr, rl, rr, l);
    let P = P0.slice();
    const R = [P.slice()];
    for (let k = 0; k < passos; k++) {
      P = passo(P, v, w, dt);
      R.push(P);
    }
    return { P, R, v, w };
  }

  return {
    P3DX, direta, inversa, saturar, icc, passosDoLaco, simular,
    passoDiferencial, passoIncremental, passoExato, PASSOS, pulsos, passoOdometria,
  };
});
