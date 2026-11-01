// Roda os testes de navegador em sequência e resume. Antes, nesta pasta do simulador:
//   python servidor.py --sem-navegador          (ou outra porta, passada em PORTA)
//   msedge --headless=new --remote-debugging-port=9333 --user-data-dir=<pasta temporária>
// Uso: node testes/navegador/rodar.mjs [parte do nome...], por exemplo `rodar.mjs caminho 3d`.
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const TESTES = ['teste_pose.mjs', 'teste_odometria.mjs', 'teste_ponto_reta.mjs', 'teste_caminho.mjs', 'teste_escolha.mjs', 'teste_3d.mjs'];
const filtro = process.argv.slice(2);
const resultado = [];
for (const t of TESTES) {
  if (filtro.length && !filtro.some((f) => t.includes(f))) continue;
  console.log(`\n=== ${t}`);
  const r = spawnSync(process.execPath, [fileURLToPath(new URL(t, import.meta.url))], { stdio: 'inherit' });
  resultado.push([t, r.status === 0]);
}
console.log('\n' + resultado.map(([t, ok]) => `${ok ? 'ok   ' : 'FALHA'} ${t}`).join('\n'));
process.exit(resultado.length && resultado.every(([, ok]) => ok) ? 0 : 1);
