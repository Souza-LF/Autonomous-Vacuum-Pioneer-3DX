/*
 * vista3d.js: a vista 3D da simulação de controle (módulo ES, carregado depois de simulacao.js).
 *
 * Só desenha: a cada quadro lê o estado de window.simulador (pose e giro das rodas de cada malha,
 * rastros, início, alvo, reta e caminho) e não escreve nada nele, então a corrida é a mesma com a vista
 * 3D aberta ou fechada. A troca entre o mapa 2D e a vista 3D, a dica e o aviso de quando este módulo
 * não carrega ficam em simulacao.js, que chama ativar(). O mundo é o do mapa: x e y no chão, em
 * metros, e z para cima. As cores são as do mapa, lidas das mesmas variáveis do CSS.
 *
 * O robô é o contorno do P3-DX do mapa (C.P3DX.corpo) em duas camadas, o chassi e a tampa, com as
 * alturas do URDF do P3-DX (p3dx_description, no repositório do aspirador): chassi de 0,075 m até a
 * tampa, tampa de 1 cm em 0,229 m e roda louca de 37,5 mm de raio atrás. As rodas têm os diâmetros e a
 * distância entre elas da simulação e giram com o ângulo acumulado de cada uma.
 */
import * as THREE from './vendor/three/three.module.min.js';
import { OrbitControls } from './vendor/three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from './vendor/three/addons/renderers/CSS2DRenderer.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);   // z para cima, com x e y no chão, como no mapa

const S = window.simulador;
const C = window.Cinematica;
const est = S.est;
const caixa = document.getElementById('cena3d');
const RAD = Math.PI / 180;

// medidas do P3-DX [m]: as alturas vêm do URDF; o contorno, as rodas e a bitola, da simulação
const CHASSI = { base: 0.075, topo: 0.229, escalaX: 0.35 / 0.455, escalaY: 0.25 / 0.381 };
const TAMPA = 0.01;
const ALTO = CHASSI.topo + TAMPA;
const LOUCA = { x: -0.2145, r: 0.0375, largura: 0.025 };
const LARGURA_RODA = C.P3DX.larguraRoda / 1000;
// o que fica no chão (grade, caminho, reta, rastros, marcas) é desenhado nesta altura e sem escrever
// profundidade; a ordem de desenho decide quem fica por cima, e o chão é empurrado para trás
const Z_CHAO = 0.001;
const ORDEM = { grade: 1, caminho: 2, marca: 3, sombra: 4, rastro: 5 };

// ---------------------------------------------------------------- cores

const cor = {};
const NOMES_COR = ['fundo-cena', 'grade', 'grade-forte', 'texto', 'texto-suave', 'dif', 'dif-corpo', 'inc', 'inc-corpo',
  'exato', 'real', 'real-corpo', 'odo', 'wl', 'wr', 'alvo', 'eixo-x', 'eixo-y', 'orientacao'];
function lerCores() {
  const s = getComputedStyle(document.documentElement);
  for (const n of NOMES_COR) cor[n] = s.getPropertyValue('--' + n).trim();
}

// cada material pintado com uma cor do tema (vezes um fator) é lembrado, para trocar junto com o tema
const pintados = new Map();
function pintar(mat, nome, fator = 1) {
  pintados.set(mat, [nome, fator]);
  mat.color.set(cor[nome]).multiplyScalar(fator);
  return mat;
}
function aplicarCores() {
  for (const [mat, [nome, fator]] of pintados) mat.color.set(cor[nome]).multiplyScalar(fator);
  // o céu é o fundo da cena puxado um pouco para o texto: o chão some nele, na névoa, no horizonte
  cena.background.set(cor['fundo-cena']).lerp(new THREE.Color(cor['texto-suave']), 0.1);
  cena.fog.color.copy(cena.background);
}
const solido = (nome, fator) => pintar(new THREE.MeshLambertMaterial(), nome, fator);
const fantasma = (nome) => pintar(new THREE.MeshLambertMaterial({ transparent: true, opacity: 0.3, depthWrite: false }), nome);
const noChao = (nome, opacidade = 1) =>
  pintar(new THREE.MeshBasicMaterial({ transparent: true, opacity: opacidade, depthWrite: false, side: THREE.DoubleSide }), nome);
const traco = (nome, opacidade = 1) => pintar(new THREE.LineBasicMaterial({ transparent: true, opacity: opacidade, depthWrite: false }), nome);
const tracejado = (nome, cheio, vazio) =>
  pintar(new THREE.LineDashedMaterial({ dashSize: cheio, gapSize: vazio, transparent: true, depthWrite: false }), nome);

// ---------------------------------------------------------------- fitas no chão

// Uma linha larga deitada no chão (o WebGL só desenha linhas de 1 pixel): um retângulo por trecho.
// Os rastros só crescem durante a corrida, então só os trechos novos entram a cada quadro; um rastro
// novo (a corrida recomeçou) refaz a fita. Tracejada, cada trecho entra ou não pelo meio dele.
class Fita {
  constructor(largura, nomeCor, ordem, traco = null, opacidade = 1) {
    this.largura = largura;
    this.traco = traco;
    this.malha = new THREE.Mesh(new THREE.BufferGeometry(), noChao(nomeCor, opacidade));
    this.malha.renderOrder = ordem;
    this.malha.frustumCulled = false;
    this.pts = null;
    this.usados = 0;   // pontos do rastro já na fita
    this.trechos = 0;  // retângulos desenhados
    this.cap = 0;      // retângulos que cabem no buffer
    this.s = 0;        // comprimento andado, para o tracejado
  }

  sincronizar(pts) {
    if (pts !== this.pts || pts.length < this.usados) {
      this.pts = pts;
      this.usados = Math.min(1, pts.length);
      this.trechos = 0;
      this.s = 0;
    }
    if (pts.length <= this.usados) return;
    this.garantir(this.trechos + pts.length - this.usados);
    const pos = this.malha.geometry.attributes.position.array;
    const w = this.largura / 2, z = Z_CHAO;
    for (let i = this.usados; i < pts.length; i++) {
      const a = pts[i - 1], b = pts[i];
      const dx = b[0] - a[0], dy = b[1] - a[1], c = Math.hypot(dx, dy);
      const meio = this.s + c / 2;
      this.s += c;
      if (c < 1e-9) continue;
      if (this.traco && meio % (this.traco[0] + this.traco[1]) > this.traco[0]) continue;
      const nx = (-dy / c) * w, ny = (dx / c) * w;
      pos.set([
        a[0] + nx, a[1] + ny, z, a[0] - nx, a[1] - ny, z, b[0] + nx, b[1] + ny, z,
        b[0] + nx, b[1] + ny, z, a[0] - nx, a[1] - ny, z, b[0] - nx, b[1] - ny, z,
      ], this.trechos * 18);
      this.trechos++;
    }
    this.usados = pts.length;
    this.malha.geometry.attributes.position.needsUpdate = true;
    this.malha.geometry.setDrawRange(0, this.trechos * 6);
  }

  // um buffer não cresce depois de desenhado: um maior, com o que já havia, a cada vez que dobra
  garantir(trechos) {
    if (trechos <= this.cap) return;
    let cap = Math.max(256, this.cap);
    while (cap < trechos) cap *= 2;
    const novo = new Float32Array(cap * 18);
    const velho = this.malha.geometry.attributes.position;
    if (velho) novo.set(velho.array.subarray(0, this.trechos * 18));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(novo, 3).setUsage(THREE.DynamicDrawUsage));
    this.malha.geometry.dispose();
    this.malha.geometry = geo;
    this.cap = cap;
  }
}

// ---------------------------------------------------------------- peças

function formaCorpo(ex = 1, ey = 1) {
  const [xs, ys] = C.P3DX.corpo;
  const f = new THREE.Shape();
  xs.forEach((x, i) => {
    const px = (x / 1000) * ex, py = (ys[i] / 1000) * ey;
    if (i) f.lineTo(px, py); else f.moveTo(px, py);
  });
  return f;
}

function noChaoEm(malha, x, y, ang = 0, ordem = ORDEM.marca) {
  malha.position.set(x, y, Z_CHAO);
  malha.rotation.z = ang;
  malha.renderOrder = ordem;
  return malha;
}
const anel = (x, y, r0, r1, nome) => noChaoEm(new THREE.Mesh(new THREE.RingGeometry(r0, r1, 48), noChao(nome)), x, y);
const disco = (x, y, r, nome, opacidade = 1) => noChaoEm(new THREE.Mesh(new THREE.CircleGeometry(r, 32), noChao(nome, opacidade)), x, y);
const retangulo = (x, y, ang, comp, larg, nome) => noChaoEm(new THREE.Mesh(new THREE.PlaneGeometry(comp, larg), noChao(nome)), x, y, ang);

// seta deitada no chão, de o na direção ang
function setaChao(o, ang, comp, larg, nome) {
  const cab = Math.min(comp * 0.4, larg * 4.5), w = larg / 2, hw = larg * 2;
  const v = (x, y) => new THREE.Vector2(x, y);
  const f = new THREE.Shape([v(0, -w), v(comp - cab, -w), v(comp - cab, -hw), v(comp, 0), v(comp - cab, hw), v(comp - cab, w), v(0, w)]);
  return noChaoEm(new THREE.Mesh(new THREE.ShapeGeometry(f), noChao(nome)), o[0], o[1], ang);
}

// ponta de seta (triângulo) no chão, centrada em (x, y), apontando para ang
function ponta(x, y, ang, tam, nome, opacidade = 1) {
  const v = (a, b) => new THREE.Vector2(a, b);
  const f = new THREE.Shape([v(0.6 * tam, 0), v(-0.4 * tam, -0.45 * tam), v(-0.4 * tam, 0.45 * tam)]);
  return noChaoEm(new THREE.Mesh(new THREE.ShapeGeometry(f), noChao(nome, opacidade)), x, y, ang);
}

// poste com uma bola no alto, para o alvo se achar de longe, em perspectiva
function poste(x, y, nome) {
  const mat = solido(nome, 1);
  const haste = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.34, 10), mat);
  haste.rotation.x = Math.PI / 2;
  haste.position.set(x, y, 0.17);
  const bola = new THREE.Mesh(new THREE.SphereGeometry(0.028, 18, 12), mat);
  bola.position.set(x, y, 0.35);
  return [haste, bola];
}

function rotuloEm(texto, nome, x, y, z) {
  const div = document.createElement('div');
  div.className = 'rotulo-3d';
  div.textContent = texto;
  div.style.color = `var(--${nome})`;
  const r = new CSS2DObject(div);
  r.position.set(x, y, z);
  return r;
}

// Libera geometrias e materiais de um grupo que sai da cena (os compartilhados ficam)
const GEO_FAIXA = new THREE.BoxGeometry(0.008, LARGURA_RODA + 0.002, 0.003);
const MAT_FAIXA = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.8 });
const MAT_LOUCA = new THREE.MeshLambertMaterial({ color: 0x3b4048 });
function descartar(obj) {
  obj.traverse((o) => {
    if (o.isCSS2DObject) o.element.remove();
    if (o.geometry && o.geometry !== GEO_FAIXA) o.geometry.dispose();
    if (o.material && o.material !== MAT_FAIXA && o.material !== MAT_LOUCA) {
      pintados.delete(o.material);
      o.material.dispose();
    }
  });
  obj.removeFromParent();
}

// ---------------------------------------------------------------- robô

// 'diferencial', 'incremental' e 'real' são robôs; 'exato' e 'odo' são fantasmas translúcidos, como os
// contornos tracejados do mapa, porque não são um robô de verdade. Os dois modelos andam quase juntos:
// o incremental fica um pouco atrás na profundidade, para o diferencial aparecer por cima, como no mapa.
const TIPOS = {
  diferencial: { corpo: 'dif-corpo' },
  incremental: { corpo: 'inc-corpo', atras: true },
  real: { corpo: 'real-corpo' },
  exato: { corpo: 'exato', fantasma: true },
  odo: { corpo: 'odo', fantasma: true },
};

function criarRobo(tipo) {
  const t = TIPOS[tipo];
  const mat = (nome, fator) => {
    if (t.fantasma) return fantasma(t.corpo);
    const m = solido(nome, fator);
    if (t.atras) Object.assign(m, { polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 8 });
    return m;
  };
  const extrudar = (f, h) => new THREE.ExtrudeGeometry(f, { depth: h, bevelEnabled: false });
  const g = new THREE.Group();

  const chassi = new THREE.Mesh(extrudar(formaCorpo(CHASSI.escalaX, CHASSI.escalaY), CHASSI.topo - CHASSI.base), mat(t.corpo, 0.7));
  chassi.position.z = CHASSI.base;
  const tampa = new THREE.Mesh(extrudar(formaCorpo(), TAMPA), mat(t.corpo, 1));
  tampa.position.z = CHASSI.topo;
  const borda = new THREE.LineSegments(new THREE.EdgesGeometry(tampa.geometry), traco(t.fantasma ? t.corpo : 'texto', t.fantasma ? 0.9 : 0.5));
  borda.position.z = CHASSI.topo;
  g.add(chassi, tampa, borda);

  // rodas nas cores de φ̇_L e φ̇_R, com os diâmetros reais (o controlador só conhece o nominal)
  const DL = est.D + est.odo.dDL, DR = est.D + est.odo.dDR, l = est.L / 2;
  const rodas = [];
  for (const [lado, D, nome] of [[1, DL, 'wl'], [-1, DR, 'wr']]) {
    const roda = new THREE.Group();
    roda.position.set(0, lado * l, D / 2);
    roda.add(new THREE.Mesh(new THREE.CylinderGeometry(D / 2, D / 2, LARGURA_RODA, 36), mat(nome, 1)));
    if (!t.fantasma) {
      // faixas brancas na banda, que andam com a superfície da roda, como as do mapa
      for (let i = 0; i < 8; i++) {
        const a = (i * Math.PI) / 4;
        const faixa = new THREE.Mesh(GEO_FAIXA, MAT_FAIXA);
        faixa.position.set(Math.sin(a) * (D / 2 + 0.001), 0, Math.cos(a) * (D / 2 + 0.001));
        faixa.rotation.y = a;
        roda.add(faixa);
      }
    }
    g.add(roda);
    rodas.push(roda);
  }

  // ponto médio e orientação em cima da tampa (o ponto vermelho e o traço de 0,12 m do mapa)
  const matFrente = t.fantasma ? traco(t.corpo) : null;
  if (t.fantasma) {
    const frente = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, ALTO), new THREE.Vector3(0.12, 0, ALTO)]), matFrente);
    g.add(frente);
  } else {
    const matOri = solido('orientacao', 1);
    const frente = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.012, 0.003), matOri);
    frente.position.set(0.06, 0, ALTO + 0.0015);
    const centro = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.003, 20), matOri);
    centro.rotation.x = Math.PI / 2;
    centro.position.set(0, 0, ALTO + 0.0015);
    // roda louca atrás, com o suporte até o chassi
    const louca = new THREE.Mesh(new THREE.CylinderGeometry(LOUCA.r, LOUCA.r, LOUCA.largura, 20), MAT_LOUCA);
    louca.position.set(LOUCA.x, 0, LOUCA.r);
    const suporte = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.035, CHASSI.base - LOUCA.r + 0.01), MAT_LOUCA);
    suporte.position.set(LOUCA.x + 0.012, 0, (LOUCA.r + CHASSI.base) / 2 + 0.005);
    // sombra simples, para o robô parecer apoiado no chão
    const sombra = new THREE.Mesh(new THREE.CircleGeometry(0.3, 40),
      new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.12, depthWrite: false }));
    sombra.scale.set(1, 0.9, 1);
    sombra.position.z = Z_CHAO;
    sombra.renderOrder = ORDEM.sombra;
    g.add(frente, centro, louca, suporte, sombra);
  }
  g.visible = false;
  return { grupo: g, rodas };
}

// ---------------------------------------------------------------- cena

let renderer = null, rotulos = null, cena = null, camera = null, controles = null;
let ativa = false, erro = null, quadros = 0, pedido = 0, tamanho = [0, 0];
let robos = {}, fitas = {}, grade = null, marcas = null, desvio = null;
let chaveCena = '', chaveRobo = '', mexeu = false;

function avisar(texto) {
  const p = document.createElement('p');
  p.className = 'aviso-3d';
  p.textContent = texto;
  caixa.append(p);
}

function montar() {
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (e) {
    erro = 'sem WebGL';
    avisar('A vista 3D precisa de WebGL, e este navegador não o ofereceu. O mapa 2D continua funcionando.');
    return false;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.domElement.addEventListener('webglcontextlost', (e) => {
    e.preventDefault();
    erro = 'contexto WebGL perdido';
    avisar('O navegador desligou o WebGL desta página. Recarregue a página para voltar à vista 3D.');
  });
  caixa.append(renderer.domElement);
  rotulos = new CSS2DRenderer();
  rotulos.domElement.className = 'rotulos-3d';
  caixa.append(rotulos.domElement);

  cena = new THREE.Scene();
  cena.background = new THREE.Color();
  cena.fog = new THREE.Fog(0xffffff, 10, 40);
  camera = new THREE.PerspectiveCamera(40, 1, 0.02, 400);
  cena.add(new THREE.HemisphereLight(0xffffff, 0x6b7280, 2.3));
  const sol = new THREE.DirectionalLight(0xffffff, 1.1);
  sol.position.set(-4, -6, 10);
  cena.add(sol);

  controles = new OrbitControls(camera, renderer.domElement);
  controles.enableDamping = true;
  controles.dampingFactor = 0.12;
  controles.maxPolarAngle = 88 * RAD;   // não passa para baixo do chão
  controles.minDistance = 0.3;
  controles.maxDistance = 150;
  controles.screenSpacePanning = false; // deslocar anda no plano do chão
  controles.zoomToCursor = true;
  controles.addEventListener('start', () => { mexeu = true; });

  // rastros: os de referência por baixo, o diferencial por cima, como no mapa
  fitas = {
    exato: new Fita(0.022, 'exato', ORDEM.rastro, [0.07, 0.05]),
    real: new Fita(0.025, 'real', ORDEM.rastro + 1),
    odo: new Fita(0.025, 'odo', ORDEM.rastro + 2, [0.08, 0.05]),
    incremental: new Fita(0.025, 'inc', ORDEM.rastro + 3),
    diferencial: new Fita(0.025, 'dif', ORDEM.rastro + 4),
  };
  for (const f of Object.values(fitas)) cena.add(f.malha);
  desvio = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), tracejado('odo', 0.02, 0.025));
  desvio.frustumCulled = false;
  cena.add(desvio);

  aplicarCores();
  const retingir = () => { lerCores(); aplicarCores(); };
  new MutationObserver(retingir).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  return true;
}

// Os robôs são refeitos quando mudam os diâmetros das rodas ou a distância entre elas
function montarRobos() {
  for (const r of Object.values(robos)) descartar(r.grupo);
  robos = {};
  for (const tipo of Object.keys(TIPOS)) {
    robos[tipo] = criarRobo(tipo);
    cena.add(robos[tipo].grupo);
  }
}

// o mesmo passo "redondo" da grade do mapa (1, 2, 2,5 ou 5 vezes uma potência de 10)
function passoBonito(x) {
  const p = Math.pow(10, Math.floor(Math.log10(x)));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= x) return m * p;
  return 10 * p;
}

// Grade em volta da cena, com o passo pelo tamanho dela (0,5 m numa cena de 4 m, 1 m numa de 8 m), os
// eixos na origem e o chão. A grade vai longe, para a borda dela ficar na névoa.
function montarGrade() {
  if (grade) descartar(grade);
  grade = new THREE.Group();
  const [a0, a1, b0, b1] = S.caixaCena();
  const g = passoBonito(Math.max(a1 - a0, b1 - b0, 2) / 8);
  const m = 30;
  const x0 = Math.floor((a0 - m) / g) * g, x1 = Math.ceil((a1 + m) / g) * g;
  const y0 = Math.floor((b0 - m) / g) * g, y1 = Math.ceil((b1 + m) / g) * g;
  const pos = [];
  for (let x = x0; x <= x1 + g / 2; x += g) pos.push(x, y0, 0, x, y1, 0);
  for (let y = y0; y <= y1 + g / 2; y += g) pos.push(x0, y, 0, x1, y, 0);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  const linhas = new THREE.LineSegments(geo, traco('grade-forte', 0.7));
  linhas.position.z = Z_CHAO / 2;
  linhas.renderOrder = ORDEM.grade;
  grade.add(linhas);
  const lado = 3 * Math.max(x1 - x0, y1 - y0);
  const chao = new THREE.Mesh(new THREE.PlaneGeometry(lado, lado),
    pintar(new THREE.MeshBasicMaterial({ polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 4 }), 'fundo-cena'));
  chao.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0);
  grade.add(chao);
  grade.add(setaChao([0, 0], 0, 0.5, 0.014, 'eixo-x'), setaChao([0, 0], Math.PI / 2, 0.5, 0.014, 'eixo-y'));
  const letras = [rotuloEm('x', 'eixo-x', 0.6, -0.06, 0.02), rotuloEm('y', 'eixo-y', -0.07, 0.6, 0.02)];
  grade.add(...letras);
  grade.userData.letras = letras.map((r) => r.element);
  // a reta vai de borda a borda da cena com uma folga, não da grade inteira
  grade.userData.caixa = [a0 - 8, a1 + 8, b0 - 8, b1 + 8];
  grade.userData.passo = g;
  cena.add(grade);
}

// Início, alvo, reta ou caminho, como no mapa (com um poste no alvo, para achá-lo em perspectiva)
function montarMarcas() {
  if (marcas) descartar(marcas);
  marcas = new THREE.Group();
  const P0 = est.P0, A = est.alvo;
  marcas.add(anel(P0[0], P0[1], 0.05, 0.066, 'texto-suave'), setaChao(P0, P0[2], 0.3, 0.014, 'texto-suave'));
  marcas.add(rotuloEm('início', 'texto-suave', P0[0], P0[1], 0.32));
  if (est.lei === 'caminho') montarCaminho(est.caminho);
  else if (est.lei === 'reta') montarReta(A);
  else if (est.lei === 'ponto') montarPonto(A);
  else montarPoseFinal(A);
  cena.add(marcas);
}

// pose final: o contorno tracejado do robô no chão e a seta de θ*
function montarPoseFinal(A) {
  const [xs, ys] = C.P3DX.corpo;
  const c = Math.cos(A[2]), s = Math.sin(A[2]);
  const pts = xs.map((x, i) => new THREE.Vector3(A[0] + (c * x - s * ys[i]) / 1000, A[1] + (s * x + c * ys[i]) / 1000, Z_CHAO));
  const contorno = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), tracejado('alvo', 0.035, 0.025));
  contorno.computeLineDistances();
  contorno.renderOrder = ORDEM.marca;
  marcas.add(contorno, setaChao(A, A[2], 0.4, 0.018, 'alvo'), disco(A[0], A[1], 0.025, 'alvo'), ...poste(A[0], A[1], 'alvo'));
  marcas.add(rotuloEm('alvo', 'alvo', A[0], A[1], 0.45));
}

// ponto: mira (anel, cruz e ponto); a orientação não conta
function montarPonto(A) {
  marcas.add(anel(A[0], A[1], 0.12, 0.14, 'alvo'), disco(A[0], A[1], 0.03, 'alvo'));
  for (let k = 0; k < 4; k++) {
    const a = (k * Math.PI) / 2;
    marcas.add(retangulo(A[0] + 0.15 * Math.cos(a), A[1] + 0.15 * Math.sin(a), a, 0.11, 0.014, 'alvo'));
  }
  marcas.add(...poste(A[0], A[1], 'alvo'), rotuloEm('alvo', 'alvo', A[0], A[1], 0.45));
}

// reta: tracejada de borda a borda da grade, com setas a cada metro no sentido do percurso
function montarReta(A) {
  const [x0, x1, y0, y1] = grade.userData.caixa;
  const ux = Math.cos(A[2]), uy = Math.sin(A[2]);
  const ts = [[x0, y0], [x1, y0], [x0, y1], [x1, y1]].map(([x, y]) => (x - A[0]) * ux + (y - A[1]) * uy);
  const t0 = Math.min(...ts), t1 = Math.max(...ts);
  const pts = [];
  for (let t = t0; t <= t1; t += 0.02) pts.push([A[0] + t * ux, A[1] + t * uy]);
  const fita = new Fita(0.022, 'alvo', ORDEM.caminho, [0.12, 0.08]);
  fita.sincronizar(pts);
  marcas.add(fita.malha);
  for (let i = Math.ceil(t0); i <= Math.floor(t1); i++) if (i) marcas.add(ponta(A[0] + i * ux, A[1] + i * uy, A[2], 0.1, 'alvo'));
  marcas.add(disco(A[0], A[1], 0.03, 'alvo'));
  marcas.add(rotuloEm('reta', 'alvo', A[0] + 0.3 * ux - 0.2 * uy, A[1] + 0.3 * uy + 0.2 * ux, 0.05));
}

// caminho: os waypoints ligados por uma fita larga e meio transparente, uma seta no meio de cada
// trecho, os waypoints numerados e o anel do fim
function montarCaminho(W) {
  const pts = [];
  for (let i = 1; i < W.length; i++) {
    const a = W[i - 1], b = W[i];
    const n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02));
    for (let k = i === 1 ? 0 : 1; k <= n; k++) pts.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]);
  }
  const fita = new Fita(0.05, 'alvo', ORDEM.caminho, null, 0.5);
  fita.sincronizar(pts);
  marcas.add(fita.malha);
  for (let i = 1; i < W.length; i++) {
    const a = W[i - 1], b = W[i];
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.4) continue;
    marcas.add(ponta((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Math.atan2(b[1] - a[1], b[0] - a[0]), 0.1, 'alvo'));
  }
  const n = W.length;
  const fechado = n > 2 && Math.hypot(W[n - 1][0] - W[0][0], W[n - 1][1] - W[0][1]) < 0.01;
  W.forEach((p, i) => {
    if (i === n - 1) {
      marcas.add(anel(p[0], p[1], 0.06, 0.076, 'alvo'), rotuloEm('fim', 'alvo', p[0], p[1], fechado ? 0.26 : 0.14));
      return;
    }
    marcas.add(disco(p[0], p[1], 0.035, 'alvo'), rotuloEm(String(i + 1), 'alvo', p[0], p[1], 0.14));
  });
}

// ---------------------------------------------------------------- câmera

// A pose do robô principal (o das leituras): a estimativa, quando só a odometria está à vista
function posePrincipal() {
  const m = S.principal();
  return est.m[m].P;
}

// Enquadra o início, o alvo (ou o caminho) e os rastros, em perspectiva ou de cima: a caixa da cena, da
// altura do robô, tem de caber na tela abaixo do quadro de leitura, que cobre o alto dela (como no
// mapa). Em perspectiva a conta não é linear, então a distância e o centro se acertam em poucas voltas.
function enquadrar(deCima = false) {
  const [x0, x1, y0, y1] = S.caixaCena();
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const lx = Math.max(x1 - x0, 1.5) + 0.6, ly = Math.max(y1 - y0, 1.5) + 0.6;
  const el = (deCima ? 89.9 : 38) * RAD;
  const az = (deCima ? -90 : -100) * RAD;
  const cantos = [];
  for (const sx of [-0.5, 0.5]) for (const sy of [-0.5, 0.5]) for (const z of [0, ALTO]) cantos.push(new THREE.Vector3(cx + sx * lx, cy + sy * ly, z));
  const hud = document.getElementById('hud');
  const h = tamanho[1] || 1;
  const reservado = Math.min(h * 0.3, (hud ? hud.offsetHeight : 0) + 12);
  const topo = 1 - (2 * reservado) / h;   // o alto da área livre, em coordenadas da tela (−1 a 1)
  const meio = (topo - 1) / 2;
  let R = Math.hypot(lx, ly), frente = 0;
  const v = new THREE.Vector3();
  for (let volta = 0; volta < 8; volta++) {
    const tx = cx - Math.cos(az) * frente, ty = cy - Math.sin(az) * frente;
    controles.target.set(tx, ty, 0);
    camera.position.set(tx + R * Math.cos(el) * Math.cos(az), ty + R * Math.cos(el) * Math.sin(az), R * Math.sin(el));
    camera.lookAt(controles.target);
    camera.updateMatrixWorld();
    let xa = Infinity, xb = -Infinity, ya = Infinity, yb = -Infinity;
    for (const p of cantos) {
      v.copy(p).project(camera);
      xa = Math.min(xa, v.x); xb = Math.max(xb, v.x); ya = Math.min(ya, v.y); yb = Math.max(yb, v.y);
    }
    // o centro da caixa vai para o meio da área livre; um metro no chão, para a frente, sobe isto na tela
    frente += (((ya + yb) / 2 - meio) * R * Math.tan((camera.fov * RAD) / 2)) / Math.sin(el);
    R *= Math.max((xb - xa) / (2 * 0.9), (yb - ya) / ((topo + 1) * 0.88));
  }
  controles.update();
  mexeu = false;
}

function deslocar(dx, dy) {
  controles.target.x += dx;
  controles.target.y += dy;
  camera.position.x += dx;
  camera.position.y += dy;
}

// "Câmera segue o robô" (a mesma opção do mapa) leva a câmera junto; sem ela, a câmera só vai atrás do
// robô quando ele sai da tela, e só enquanto quem usa não mexeu nela
function acompanhar() {
  const P = posePrincipal();
  if (est.exibir.seguir) {
    deslocar((P[0] - controles.target.x) * 0.2, (P[1] - controles.target.y) * 0.2);
    return;
  }
  if (mexeu) return;
  const v = new THREE.Vector3(P[0], P[1], ALTO / 2).project(camera);
  if (Math.abs(v.x) < 0.85 && Math.abs(v.y) < 0.85 && v.z < 1) return;
  deslocar((P[0] - controles.target.x) * 0.05, (P[1] - controles.target.y) * 0.05);
}

// ---------------------------------------------------------------- quadro

// A caixa do texto de um rótulo na tela, sem o espaço que o desloca do ponto (ver .rotulo-3d no CSS)
function caixaTexto(e) {
  const r = document.createRange();
  r.selectNodeContents(e);
  return r.getBoundingClientRect();
}
const cruzam = (a, b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;

// As letras dos eixos saem quando caem em cima de outro rótulo na tela (o "início" na origem, por
// exemplo): os eixos continuam pelas cores das setas
function liberarLetras() {
  const letras = grade.userData.letras;
  const outros = [...rotulos.domElement.querySelectorAll('.rotulo-3d')]
    .filter((e) => !letras.includes(e) && e.style.display !== 'none').map(caixaTexto);
  for (const e of letras) e.style.visibility = outros.some((b) => cruzam(caixaTexto(e), b)) ? 'hidden' : '';
}

function mostrar(nome, visivel, P, ang, rastro) {
  const r = robos[nome], f = fitas[nome];
  r.grupo.visible = f.malha.visible = visivel;
  if (!visivel) return;
  r.grupo.position.set(P[0], P[1], 0);
  r.grupo.rotation.z = P[2];
  r.rodas[0].rotation.y = ang.e;
  r.rodas[1].rotation.y = ang.d;
  f.sincronizar(rastro);
}

function medir() {
  const w = caixa.clientWidth, h = caixa.clientHeight;
  if (!w || !h || (w === tamanho[0] && h === tamanho[1])) return;
  tamanho = [w, h];
  renderer.setSize(w, h, false);
  rotulos.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function quadro() {
  pedido = 0;
  if (!ativa) return;
  medir();
  const chaveR = [est.D, est.L, est.odo.dDL, est.odo.dDR].join();
  if (chaveR !== chaveRobo) { chaveRobo = chaveR; montarRobos(); }
  const chaveC = JSON.stringify([est.lei, est.P0, est.lei === 'caminho' ? est.caminho : est.alvo]);
  if (chaveC !== chaveCena) {
    chaveCena = chaveC;
    montarGrade();
    montarMarcas();
    enquadrar();
  }

  const so = est.m.odometria;
  for (const m of ['diferencial', 'incremental']) mostrar(m, est.ativo[m], est.m[m].P, est.m[m].ang, est.m[m].rastro);
  mostrar('exato', est.exibir.exato, est.m.exato.P, est.m.exato.ang, est.m.exato.rastro);
  mostrar('real', est.odo.ativo, so.real, so.ang, so.rastroReal);
  mostrar('odo', est.odo.ativo, so.P, so.ang, so.rastro);
  // o desvio entre o robô real e a estimativa, tracejado por cima dos dois
  desvio.visible = est.odo.ativo;
  if (desvio.visible) {
    const pos = desvio.geometry.attributes.position;
    pos.setXYZ(0, so.real[0], so.real[1], ALTO + 0.01);
    pos.setXYZ(1, so.P[0], so.P[1], ALTO + 0.01);
    pos.needsUpdate = true;
    desvio.computeLineDistances();
  }

  acompanhar();
  controles.update();
  // névoa a partir de um pouco além do ponto olhado: a grade some longe, e o perto fica nítido
  const d = camera.position.distanceTo(controles.target);
  cena.fog.near = 1.2 * d + 2;
  cena.fog.far = 3.5 * d + 6;
  renderer.render(cena, camera);
  rotulos.render(cena, camera);
  liberarLetras();
  quadros++;
  pedido = requestAnimationFrame(quadro);
}

// Liga ou desliga a vista 3D. Só desenha enquanto está à vista; a primeira vez monta a cena.
function ativar(ligar) {
  ativa = !!ligar;
  if (!ativa) return;
  if (!renderer) {
    if (erro) return;
    lerCores();
    if (!montar()) { ativa = false; return; }
  }
  if (!pedido) pedido = requestAnimationFrame(quadro);
}

// para os testes no navegador: o que está desenhado, sem mexer em nada
function estado() {
  const r = {}, f = {};
  for (const [nome, robo] of Object.entries(robos)) {
    const g = robo.grupo;
    r[nome] = {
      visivel: g.visible, x: g.position.x, y: g.position.y, th: g.rotation.z,
      rodaE: robo.rodas[0].rotation.y, rodaD: robo.rodas[1].rotation.y, raios: [robo.rodas[0].position.z, robo.rodas[1].position.z],
    };
  }
  for (const [nome, fita] of Object.entries(fitas)) f[nome] = { visivel: fita.malha.visible, pontos: fita.usados, trechos: fita.trechos };
  return {
    ativa, erro, quadros, robos: r, rastros: f,
    camera: camera ? { posicao: camera.position.toArray(), alvo: controles.target.toArray() } : null,
    rotulos: [...caixa.querySelectorAll('.rotulo-3d')].map((e) => e.textContent),
    letras: grade ? grade.userData.letras.map((e) => e.style.display !== 'none' && e.style.visibility !== 'hidden') : null,
    ceu: cena ? '#' + cena.background.getHexString() : null,
    tela: renderer ? [renderer.domElement.width, renderer.domElement.height] : null,
    grade: grade ? grade.userData.passo : null,
  };
}

// onde um ponto do mundo cai na tela, em pixels a partir do canto da vista 3D (para os testes)
function naTela(x, y, z = 0) {
  const v = new THREE.Vector3(x, y, z).project(camera);
  return [((v.x + 1) / 2) * tamanho[0], ((1 - v.y) / 2) * tamanho[1], v.z < 1];
}

window.vista3d = { ativar, estado, naTela, enquadrar: () => enquadrar(false), deCima: () => enquadrar(true) };
document.getElementById('btn-3d-enquadrar').addEventListener('click', () => { if (ativa) enquadrar(false); });
document.getElementById('btn-3d-cima').addEventListener('click', () => { if (ativa) enquadrar(true); });
// a página pode ter aberto já na vista 3D (#3d na URL), antes de este módulo carregar
if (S.vista.modo === '3d') ativar(true);
