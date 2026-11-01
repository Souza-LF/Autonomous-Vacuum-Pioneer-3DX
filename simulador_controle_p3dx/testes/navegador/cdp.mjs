// Cliente mínimo do Chrome DevTools Protocol para o Edge headless (Node 22+, WebSocket global).
import { mkdirSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

// A página servida por `python servidor.py` (porta PORTA, padrão 8000) e o navegador sem janela com a
// depuração remota (porta PORTA_CDP, padrão 9333). As fotos dos testes vão para fotos/, fora do git.
export const BASE = `http://127.0.0.1:${process.env.PORTA || 8000}/`;
export const FOTOS = fileURLToPath(new URL('./fotos/', import.meta.url));
mkdirSync(FOTOS, { recursive: true });

export async function conectar(porta = Number(process.env.PORTA_CDP) || 9333) {
  const base = `http://127.0.0.1:${porta}`;
  let lista;
  for (let i = 0; i < 40; i++) {
    try { lista = await (await fetch(base + '/json/list')).json(); break; } catch { await dormir(250); }
  }
  if (!lista) throw new Error('Edge não respondeu na porta ' + porta);
  let alvo = lista.find((t) => t.type === 'page');
  if (!alvo) alvo = await (await fetch(base + '/json/new?about:blank', { method: 'PUT' })).json();
  const ws = new WebSocket(alvo.webSocketDebuggerUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  let id = 0;
  const pendentes = new Map();
  const eventos = [];
  const ouvintes = [];
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.id && pendentes.has(msg.id)) {
      const { res, rej } = pendentes.get(msg.id);
      pendentes.delete(msg.id);
      if (msg.error) rej(new Error(JSON.stringify(msg.error))); else res(msg.result);
    } else if (msg.method) {
      eventos.push(msg);
      for (const f of ouvintes) f(msg);
    }
  };
  const enviar = (method, params = {}) => new Promise((res, rej) => {
    const i = ++id;
    pendentes.set(i, { res, rej });
    ws.send(JSON.stringify({ id: i, method, params }));
  });
  const avaliar = async (expressao) => {
    const r = await enviar('Runtime.evaluate', { expression: expressao, awaitPromise: true, returnByValue: true, userGesture: true });
    if (r.exceptionDetails) throw new Error('JS: ' + JSON.stringify(r.exceptionDetails.exception?.description || r.exceptionDetails.text));
    return r.result.value;
  };
  const esperarEvento = (nome, ms = 10000) => new Promise((res, rej) => {
    const t = setTimeout(() => rej(new Error('tempo esgotado: ' + nome)), ms);
    ouvintes.push(function f(msg) {
      if (msg.method === nome) { clearTimeout(t); ouvintes.splice(ouvintes.indexOf(f), 1); res(msg); }
    });
  });
  const tela = async (largura, altura, escuro = false, movel = false) => {
    await enviar('Emulation.setDeviceMetricsOverride', { width: largura, height: altura, deviceScaleFactor: 1, mobile: movel });
    await enviar('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: escuro ? 'dark' : 'light' }] });
  };
  const abrir = async (url) => {
    // a mesma URL com #fragmento só rolaria a página, sem carregá-la de novo: antes, about:blank
    if (url.includes('#')) {
      const limpou = esperarEvento('Page.loadEventFired', 15000);
      await enviar('Page.navigate', { url: 'about:blank' });
      await limpou;
    }
    const carregou = esperarEvento('Page.loadEventFired', 15000);
    await enviar('Page.navigate', { url });
    await carregou;
    await dormir(600);
  };
  const foto = async (arquivo, clip) => {
    const r = await enviar('Page.captureScreenshot', { format: 'png', ...(clip ? { clip: { ...clip, scale: 1 } } : {}) });
    writeFileSync(arquivo, Buffer.from(r.data, 'base64'));
    return arquivo;
  };
  // eventos de mouse "de verdade" (geram pointer events com pointerId válido)
  const mouse = async (tipo, x, y, botoes = 1) => enviar('Input.dispatchMouseEvent', {
    type: tipo, x, y, button: tipo === 'mouseMoved' && !botoes ? 'none' : 'left', buttons: botoes, clickCount: 1,
  });
  const arrastar = async (x0, y0, x1, y1, passos = 8) => {
    await mouse('mouseMoved', x0, y0, 0);
    await mouse('mousePressed', x0, y0);
    for (let i = 1; i <= passos; i++) await mouse('mouseMoved', x0 + ((x1 - x0) * i) / passos, y0 + ((y1 - y0) * i) / passos, 1);
    await mouse('mouseReleased', x1, y1, 0);
  };
  const tecla = async (key, code, keyCode) => {
    await enviar('Input.dispatchKeyEvent', { type: 'rawKeyDown', key, code, windowsVirtualKeyCode: keyCode });
    await enviar('Input.dispatchKeyEvent', { type: 'keyUp', key, code, windowsVirtualKeyCode: keyCode });
  };
  await enviar('Page.enable');
  await enviar('Runtime.enable');
  await enviar('Log.enable');
  // ao ligar, Runtime e Log repetem o que a aba já tinha registrado (a página do teste anterior): fora
  eventos.length = 0;
  return { enviar, avaliar, abrir, tela, foto, mouse, arrastar, tecla, eventos, fechar: () => ws.close() };
}

export const dormir = (ms) => new Promise((r) => setTimeout(r, ms));

// erros de console e exceções capturados até agora; o favicon.ico, que o navegador recém-aberto pede
// sozinho e o servidor não tem, não é erro da página
export function erros(eventos) {
  return eventos.filter((e) =>
    e.method === 'Runtime.exceptionThrown' ||
    (e.method === 'Runtime.consoleAPICalled' && ['error', 'warning', 'assert'].includes(e.params.type)) ||
    (e.method === 'Log.entryAdded' && ['error', 'warning'].includes(e.params.entry.level) && !/\/favicon\.ico$/.test(e.params.entry.url || '')))
    .map((e) => JSON.stringify(e.params).slice(0, 400));
}
