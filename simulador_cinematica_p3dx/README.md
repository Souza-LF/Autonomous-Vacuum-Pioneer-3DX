# Simulador cinemático do Pioneer P3-DX

Trabalho da Aula 14 de ENE122 Robótica Móvel (Cinemática, Parte B, Prof. André Marcato): uma
interface gráfica que mostra o Pioneer P3-DX andando, com os modelos diferencial e incremental,
a cinemática direta e a inversa e o efeito de rodas com diâmetros diferentes. Roda no navegador,
em localhost. Só precisa do Python (biblioteca padrão) e de um navegador.

O enunciado pede dois modos de entrada, e a tela tem um seletor para isso:

- **Modo 1 · Rodas → robô.** O usuário dá as velocidades das rodas. O simulador mostra os
  gráficos de v e ω.
- **Modo 2 · Robô → rodas.** O usuário dá v e ω. O simulador calcula as rodas e mostra os
  gráficos de φ̇<sub>L</sub> e φ̇<sub>R</sub>.

## Como rodar

```bash
python servidor.py
```

O servidor abre `http://localhost:8000` no navegador (se a porta estiver ocupada, usa a próxima
livre). `Ctrl+C` encerra. Outras formas:

- `python servidor.py 8080` escolhe a porta; `python servidor.py --sem-navegador` só sobe o servidor.
- `python -m http.server 8000` nesta pasta, abrindo o endereço à mão.

## A tela

| Região | O que tem |
| --- | --- |
| Topo | Os dois modelos da pose (diferencial e incremental), que podem rodar juntos para comparar; Pausar, Reiniciar, Limpar rastro. |
| Esquerda | O seletor de entrada, φ̇<sub>L</sub> e φ̇<sub>R</sub> (velocidades angulares das rodas, o "PL/PR" do quadro) e v, ω. No modo 1 as rodas são a entrada, e v e ω saem da cinemática direta. No modo 2, v e ω são a entrada, e as rodas saem da inversa. O grupo de saída fica marcado "saída" e os controles dele só mostram o valor calculado. |
| Centro | O robô no plano com a geometria dos slides, o rastro de cada modelo com um ponto por passo Δt, o ICC, os vetores v<sub>L</sub>, v<sub>R</sub>, v e o sentido de ω. Embaixo, o slider da diferença de diâmetro: para a esquerda aumenta a roda esquerda, para a direita aumenta a direita. |
| Direita | Diâmetro da roda, distância entre rodas, velocidade máxima da roda, Δt, velocidade da simulação, duração, pose inicial, o que exibir e os exemplos dos slides. |
| Direta ⇄ inversa | Os espaços das rodas (φ̇<sub>L</sub>, φ̇<sub>R</sub>) e do robô (ω, v), com as matrizes das duas cinemáticas entre eles. Arrastar o ponto no gráfico de entrada (o das rodas no modo 1, o do robô no modo 2) move o outro, os sliders e o robô. As retas tracejadas pelo ponto mostram v e ω constantes no espaço das rodas e φ̇<sub>L</sub>, φ̇<sub>R</sub> constantes no espaço do robô; a região sombreada é o que as rodas alcançam. |
| Histórico | No modo 1, v e ω; no modo 2, φ̇<sub>L</sub> e φ̇<sub>R</sub>. Nos dois, a distância de cada modelo ao arco exato. Janela dos últimos 20 s. |

Teclado (com o foco fora dos campos):
- No modo 1, ↑ e ↓ somam ±5 °/s às duas rodas, e ← e → tiram 5 °/s de uma roda e põem na outra.
- No modo 2, ↑ e ↓ mudam v em 0,02 m/s, e ← e → mudam ω em 5 °/s.
- Nos dois modos: espaço pausa, 0 para as rodas, R reinicia.

Na cena: arrastar move a vista, a roda do mouse dá zoom, duplo clique enquadra.

## Equações

Notação dos slides: pose P = (x, y, θ), r raio da roda, l metade da distância entre as rodas.

- Direta: v = (r<sub>R</sub> φ̇<sub>R</sub> + r<sub>L</sub> φ̇<sub>L</sub>) / 2 e ω = (r<sub>R</sub> φ̇<sub>R</sub> − r<sub>L</sub> φ̇<sub>L</sub>) / (2l).
  Com as rodas iguais fica v = (r/2)(φ̇<sub>R</sub> + φ̇<sub>L</sub>) e ω = (r/2l)(φ̇<sub>R</sub> − φ̇<sub>L</sub>), como nos slides.
- Inversa: φ̇<sub>L</sub> = (v − ωl) / r e φ̇<sub>R</sub> = (v + ωl) / r, com o raio nominal r (veja abaixo).
- Modelo diferencial (Euler, slides 22 e 29): x += v cos θ Δt, y += v sin θ Δt, θ += ω Δt.
- Modelo incremental (slides 39 e 50): Δs = v Δt, Δθ = ω Δt, x += Δs cos(θ + Δθ/2),
  y += Δs sin(θ + Δθ/2), θ += Δθ.
- Arco exato (referência, fora dos slides): com v e ω constantes no passo, o robô anda sobre um
  arco de raio R = v/ω em volta do ICC. Serve para medir o erro dos dois modelos: o Euler erra
  na ordem de Δt e o incremental na ordem de Δt².

Parâmetros do P3-DX tirados do código dos slides 26 e 27: diâmetro da roda 195 mm (r = 97,5 mm)
e distância entre rodas 381 mm (l = 190,5 mm). O limite de 360 °/s por roda não vem dos slides:
é uma escolha do simulador para a faixa dos sliders (dá v máx. de 0,613 m/s) e pode ser mudado.

## Escolhas de interpretação

- As velocidades das rodas são a fonte da verdade, porque são elas que os motores comandam.
  Pedir v e ω calcula as rodas pela inversa; se uma roda passar do limite, as duas são reduzidas
  na mesma proporção, o que mantém o raio de curva R = v/ω, e aparece um aviso.
- O slider de diferença de diâmetro simula um erro que quem comanda não conhece. A inversa usa o
  raio nominal (o do campo "Diâmetro da roda"), e a direta usa os raios reais r<sub>L</sub> e r<sub>R</sub>.
  Mexer no slider mantém φ̇<sub>L</sub> e φ̇<sub>R</sub> e recalcula o v e o ω que o robô executa.
  - No modo 1, com as rodas girando igual, o robô passa a curvar para o lado da roda menor.
  - No modo 2, o comando continua o mesmo, mas o robô executa outro v e outro ω. A linha embaixo
    de v e de ω mostra o valor executado. No espaço do robô, um anel vazio marca o comando, e o
    ponto cheio marca o que o robô faz.
- Mudar o diâmetro nominal, a distância entre rodas ou o limite mantém o que o modo atual
  comanda: as rodas no modo 1, v e ω no modo 2.
- Os exemplos dos slides comandam as rodas, por isso passam para o modo 1.
- Os exemplos dos slides rodam 30 s com o mesmo número de passos do laço
  `np.arange(0, tmax + dt, dt)`, isto é, round(tmax/Δt) + 1 passos. Por isso a pose final na tela
  é a do programa em Python.
- O código do slide 48 (incremental, 50 e 80 °/s, Δt = 0,2 s) termina em (−0,649; 1,335) m e
  231,9°. A figura do slide 53 não mostra esse resultado: ela repete a trajetória do exemplo do
  slide 27.

## Validação

```bash
python testes/referencia_professor.py
node testes/teste_cinematica.js
```

O primeiro roda os dois programas dos slides (27 e 29; 48 e 50) sem a parte gráfica e grava a
trajetória em `testes/referencia.json`. O segundo refaz as duas trajetórias com `cinematica.js` e
compara ponto a ponto: as 906 e as 456 coordenadas saem idênticas bit a bit, com 301 e 151 passos.
Ele também confere a ida e volta direta → inversa, o sinal de ω com rodas diferentes, a saturação
e a ordem de convergência contra o arco exato (razões 2,000 e 4,000 ao dividir Δt por dois).

A interface foi testada no navegador. Os três exemplos, rodados pela tela, chegam à mesma pose
final do Python. Também foram conferidos o arraste nos dois gráficos, o teclado, o zoom, a página
no celular (375 e 390 px) e o tema escuro.

O seletor de entrada foi testado nos dois modos:
- quais controles e quais gráficos aparecem em cada modo;
- as rodas pela inversa no modo 2 e a saturação mantendo R;
- o teclado e o arraste só no gráfico de entrada;
- o erro de raio, que no modo 2 mantém o comando e muda o executado.

## Arquivos

| Arquivo | Conteúdo |
| --- | --- |
| `index.html` | Estrutura da página. |
| `estilo.css` | Aparência, temas claro e escuro e as cores usadas no desenho. |
| `cinematica.js` | Só a física: direta, inversa, saturação, ICC e os três passos de integração. Roda no navegador e no Node. |
| `interface.js` | Estado, laço de tempo, desenho da cena e dos gráficos, eventos. |
| `servidor.py` | Servidor local (biblioteca padrão do Python). |
| `testes/` | Referência em Python a partir do código dos slides e teste em Node. |
| `COLCON_IGNORE` | Faz o colcon pular esta pasta quando o workspace ROS 2 é compilado. |
