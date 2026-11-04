# Controle do Pioneer P3-DX

Mini Desafio 01b de ENE122 Robótica Móvel: ler a Seção 4.1.1 do Corke (*Robotics, Vision and
Control*) e implementar um dos algoritmos de controle com o modelo diferencial e/ou o incremental.
O grupo escolheu **ir a uma pose** (*driving to a configuration*, 4.1.1.4) como algoritmo
principal, com todas as medições. Os outros três (ir a um ponto, seguir uma reta e seguir um
caminho) vêm como extras, na mesma malha fechada, para comparar.

A simulação roda no navegador, em localhost, e usa os modelos diferencial e incremental do
[simulador cinemático da Aula 14](../simulador_cinematica_p3dx/) (`cinematica.js` é uma cópia do arquivo de lá). A malha também pode
ser fechada pela odometria dos encoders. Só precisa do Python (biblioteca padrão) e de um navegador.

## Como rodar

```bash
python servidor.py
```

O servidor abre `http://localhost:8000` no navegador, na página de escolha (se a porta estiver
ocupada, usa a próxima livre). `Ctrl+C` encerra. Outras formas:

- `python servidor.py 8080` escolhe a porta; `python servidor.py --sem-navegador` só sobe o servidor.
- `python -m http.server 8000` nesta pasta, abrindo o endereço à mão. No Windows, a vista 3D pode
  não abrir assim: em algumas máquinas o Python tira do registro o tipo `text/plain` para `.js`, e o
  navegador recusa o módulo. O `servidor.py` corrige isso.
- Abrir `simulacao.html` direto do disco (`file://`) também roda a simulação, mas sem a vista 3D: o
  navegador não carrega módulos de arquivos, e a área da vista 3D avisa.

### Executável para Windows (sem precisar de Python)

```bash
python -m PyInstaller ControleP3DX.spec
```

Gera `dist/ControleP3DX.exe` (cerca de 7 MB, um arquivo só). Ele leva o Python, o `servidor.py`, as
duas páginas e o three.js da vista 3D; ao abrir, serve em localhost e abre o navegador, igual ao `python servidor.py`, e
aceita os mesmos argumentos. Fechar a janela de console encerra a simulação. `build/` e `dist/` não
vão para o git.

## As duas páginas

- **`index.html`, a escolha.** Um cartão por algoritmo, com a lei, o que se marca no mapa e uma
  miniatura que roda o exemplo do livro no modelo diferencial, com as mesmas funções da simulação.
  A animação é acelerada para cada volta durar uns 6 s (o fator aparece no canto). Com "reduzir
  movimento" ligado no sistema, a miniatura mostra a trajetória inteira, parada.
- **`simulacao.html`, a simulação.** Abre no algoritmo pedido na URL
  (`simulacao.html?algoritmo=pose`, `ponto`, `reta` ou `caminho`). Trocar o algoritmo no seletor do
  topo abre o exemplo do Corke dele e atualiza a URL.

## A tela da simulação

| Região | O que tem |
| --- | --- |
| Topo | O link de volta à escolha, o seletor do algoritmo, os dois modelos da pose (diferencial e incremental), que rodam juntos para comparar, Pausar/Continuar e Reiniciar. |
| Esquerda | A lei do algoritmo escolhido, com um slider e um campo para cada ganho (a marca na trilha é o zero, porque o sinal decide a estabilidade). As condições ficam em verde ou vermelho, com um aviso quando alguma não vale. Na pose, uma quarta etiqueta, só contornada, mostra a condição forte de Siegwart e Nourbakhsh, que não decide a chegada. Um texto diz o que os ganhos fazem: a condição forte da pose, a espiral do ponto, ζ e ω<sub>n</sub> da reta, a chegada do caminho. Embaixo, "Agora": as grandezas do controlador, v e ω comandados e executados, as rodas e o aviso de saturação. |
| Centro | O mapa, com três ferramentas: marcar o alvo (a reta, ou editar o caminho), marcar o início e mover a vista. Mostra o rastro de cada modelo com um ponto por passo Δt e as grandezas do controlador desenhadas: ρ, α e β da Fig. 4.13 na pose; d e θ* ⊖ θ na reta; p<sub>j</sub>, p<sub>k</sub>, o círculo de raio d e α no caminho. O quadro de leitura diz quando cada modelo chegou e com que erro. Logo abaixo, o histórico da corrida inteira: distância, ângulos, v, ω (comando tracejado e executado), as rodas com o limite e, com a odometria, o desvio entre o robô real e a estimativa. Acima do mapa, a aba "Mapa 2D \| Vista 3D" troca o mapa pela vista 3D (seção seguinte). |
| Direita | Poses de início e alvo (na reta, também em (a, b, c)), o caminho (número de waypoints, comprimento e o tempo mínimo para percorrê-lo), os exemplos de cada algoritmo, as tolerâncias de chegada e o tempo máximo, os parâmetros do robô, o robô real e a odometria (diâmetro a mais de cada roda, escorregamento, pulsos por volta, malha pela odometria), Δt, a velocidade da simulação e o que exibir. |
| Embaixo | "Equações usadas", com as leis, as condições e as escolhas de cada algoritmo. |

No mapa, com "Marcar alvo", clicar põe o alvo e arrastar a partir dele dá a orientação; na reta,
arrastar dá o sentido do percurso. No caminho, a mesma ferramenta vira "Editar caminho":

- clicar longe de tudo acrescenta um waypoint no fim;
- arrastar um waypoint o move;
- arrastar a partir de um trecho insere um waypoint nele;
- Shift+clique ou o botão direito apaga um waypoint, deixando no mínimo dois.

A roda do mouse dá zoom. Com "Mover vista", arrastar move o mapa e o duplo clique enquadra. Com o
foco fora dos campos, espaço pausa e R reinicia. Mudar ganhos, poses, parâmetros ou Δt recomeça a
corrida da pose inicial.

O fundo é sempre claro, mesmo com o Windows ou o navegador no modo escuro.

## A vista 3D

A aba "Vista 3D" mostra a mesma corrida em 3D, desenhada com o three.js. A vista só lê o estado da
simulação e não muda nada nele: com ela aberta, a corrida continua bit a bit a de `simular()`.

- O P3-DX é o contorno do mapa em duas camadas, o chassi e a tampa, com as alturas do URDF do
  repositório do grupo (`p3dx_description/urdf/p3dx.xacro`), a roda louca atrás e as duas rodas, que
  giram com o ângulo de cada uma. As rodas têm os diâmetros reais, com o diâmetro a mais de cada roda
  da coluna da direita.
- Os robôs são os do mapa, nas mesmas cores: os modelos ligados no topo, o arco exato quando
  exibido (transparente) e, com a odometria, o robô real e a estimativa (transparente), ligados por
  uma linha tracejada. Os rastros ficam no chão; o do arco exato e o da estimativa, tracejados.
- No chão também ficam o início, o alvo (com um poste e o nome), a reta com o sentido do percurso e
  o caminho com os waypoints numerados, sobre uma grade cujo passo acompanha o tamanho da cena. As
  letras x e y dos eixos saem quando cairiam em cima de outro rótulo, como o do início na origem.
- Arrastar gira a câmera; com o botão direito (ou dois dedos), desloca; a roda do mouse aproxima,
  na direção do cursor. "De cima" olha o chão de cima e "Enquadrar" volta à vista inicial. Enquanto
  a câmera não é mexida, ela acompanha o robô quando ele chega perto da borda; "Câmera segue o robô",
  nas opções de exibição, também vale aqui.
- Alvo, reta e caminho se marcam no mapa 2D; a vista 3D acompanha, com as mesmas cores.
- `#3d` no fim da URL abre direto na vista 3D (`simulacao.html?algoritmo=reta#3d`); trocar de aba
  atualiza a URL.
- O three.js 0.160.0 (licença MIT) vem junto, em `vendor/three/`: a vista 3D funciona sem internet,
  também no executável. A origem e os sha256 estão em `vendor/three/LEIAME.md`.
- A vista 3D precisa da página servida (`python servidor.py` ou o executável). Aberta como arquivo,
  a área avisa isso e o mapa 2D continua funcionando. Se o módulo falhar por outro motivo, o aviso
  diz qual: um arquivo que não chegou ou o erro que ele lançou.

## A malha fechada

A cada Δt, para cada modelo:

1. o controlador lê a pose daquele modelo e dá (v, ω);
2. a inversa, com o raio **nominal** (o único que o controlador conhece), dá as rodas;
3. se uma roda passar de ±360°/s, as duas são reduzidas na mesma proporção, o que mantém o raio de
   curva R = v/ω;
4. a direta, com os raios **reais** e a fração de cada roda que não escorrega, dá o v e o ω que o
   robô executa;
5. o modelo (diferencial, incremental ou o arco exato de referência) anda um passo com o comando
   constante.

Os três modelos partem da mesma pose, cada um com a própria malha, como se houvesse um sensor
perfeito de posição. A chegada é conferida depois de cada passo, do mesmo jeito que na função
`simular()` dos testes. Por isso o tempo de chegada na tela é bit a bit o do Node.

Na **malha pela odometria**, o robô real anda pelo arco exato com os raios reais e o escorregamento.
Os encoders contam pulsos inteiros do giro de cada roda, e a estimativa é integrada pelo modelo
incremental com o raio nominal (slide 43 da Aula 14). O controlador só vê a estimativa, então para
quando a *estimativa* chega.

## Os quatro algoritmos

### Ir a uma pose (4.1.1.4), o principal

ρ, α e β são as coordenadas polares do alvo vistas pelo robô (Fig. 4.13), para um alvo qualquer:
ρ é a distância, α o ângulo do alvo visto da frente do robô e β = θ* ⊖ θ ⊖ α o que falta girar.

- v = k<sub>ρ</sub> ρ e ω = k<sub>α</sub> α + k<sub>β</sub> β, estável com k<sub>ρ</sub> > 0, k<sub>β</sub> < 0
  e k<sub>α</sub> − k<sub>ρ</sub> > 0. Os ganhos padrão são (0,5; 1,5; −0,5).
- Ré: se o alvo começa atrás (α fora de (−90°, 90°]), o controlador trata o robô como "virado"
  (frente em θ + 180°, alvo em θ* + 180°) e nega v, mantendo ω. É o "negar v e γ" do Corke, porque
  γ = atan(ωL/v). O sentido é decidido no início e mantido até o fim; a caixa "Dar ré" o proíbe.

Medido com uma grade de 96 poses iniciais (1, 2 e 4 m do alvo, 8 direções, 4 orientações, alvo em
(0, 0, 90°)) e os ganhos padrão:

- As 96 chegam nos três modelos (ρ < 1 cm e erro de orientação < 1°); a mais lenta em 22,7 s no
  diferencial e 23,4 s nos outros dois. As rodas saturam em 84 das 96.
- Com uma condição violada, em 120 s: k<sub>β</sub> > 0, 0 de 96; k<sub>ρ</sub> < 0, 0 de 96;
  k<sub>α</sub> < k<sub>ρ</sub>, 4 de 96.
- (1, 2, −1) cumpre as três condições e chega nas 96, mas em 12 delas α sai de (−90°, 90°].
- Esses ganhos violam a **condição forte** de Siegwart e Nourbakhsh (*Introduction to Autonomous Mobile
  Robots*, 2004, seção 3.6.2, eq. 3.65): com k<sub>ρ</sub> > 0 e k<sub>β</sub> < 0,
  k<sub>α</sub> + 5/3 k<sub>β</sub> − 2/π k<sub>ρ</sub> > 0 garante que α não sai de (−90°, 90°], ou
  seja, que o robô não troca de sentido na aproximação. Os ganhos padrão a cumprem (margem 0,35);
  (1, 2, −1), não (−0,30).
  - Numa varredura de 144 conjuntos de ganhos que cumprem as três condições, nos 92 que cumprem a forte α
    nunca saiu do intervalo nas 96 poses, e nos 52 que não cumprem saiu em pelo menos uma.
  - Os testes conferem os seis conjuntos mais perto da fronteira de cada lado: com margens de +0,008 a
    +0,073, nenhuma pose tira α do intervalo; com margens de −0,034 a −0,137, de 6 a 12 poses tiram.
- As 48 com o alvo atrás chegam de ré em 15,3 s e 2,93 m de percurso, em média; sem ré, em 17,5 s e
  4,04 m.
- Exemplo do Corke, de (9, 5, 0°) a (5, 5, 90°): chega de ré em 16,55 s no diferencial e em 17,25 s
  no incremental e no arco exato.
- Com k<sub>β</sub> = +0,5, o robô chega à posição, mas fica girando com β perto de ±177°.
- Δt: contra o arco exato, o diferencial tem erro de ordem 1 (dobrar Δt multiplica o desvio por
  2,03 a 2,14) e o incremental, de ordem 2 (4,05 a 4,21).

### Ir a um ponto (4.1.1.1)

- θ* = atan2(y* − y, x* − x), v = K<sub>v</sub> ρ e ω = K<sub>h</sub> (θ* ⊖ θ), com os ganhos da Fig. 4.8
  (0,5; 1,5). θ* ⊖ θ é o α da pose, e α̇ = K<sub>v</sub> sin α − K<sub>h</sub> α: α vai a zero se
  K<sub>h</sub> > K<sub>v</sub> > 0.
- É a lei da pose sem o termo de β, então a orientação final fica livre. Não dá ré, porque v ≥ 0.
- Exemplo do Corke, de (8, 5, 90°) a (5, 5): chega em 14,20 s, virado para 217,7°. As 8 partidas da
  Fig. 4.8 chegam cada uma girada de 45° da anterior.
- A grade de 96 poses chega nos três modelos.
- Com K<sub>h</sub>/K<sub>v</sub> = 0,8, α para no equilíbrio K<sub>v</sub> sin α = K<sub>h</sub> α, em 64,8°, e o
  robô chega em espiral (|α| na chegada com mediana de 65,1°, com Δt = 0,01 s). Com 0,6, abaixo de
  2/π, o equilíbrio passa de 90° e nenhuma das 96 chega: todas se afastam.

### Seguir uma reta (4.1.1.2)

- A reta ax + by + c = 0, d = (ax + by + c)/√(a² + b²), positivo com o robô à direita do percurso,
  e θ* = atan2(a, −b), a direção do percurso.
- ω = K<sub>d</sub> d + K<sub>h</sub> (θ* ⊖ θ), com v constante. O sinal de K<sub>d</sub> d foi conferido na
  página do livro e na Fig. 4.10 (a extração de texto do PDF perde o sinal).
- Perto da reta, d̈ + K<sub>h</sub> ḋ + v K<sub>d</sub> d = 0: ω<sub>n</sub> = √(v K<sub>d</sub>) e
  ζ = K<sub>h</sub> / (2ω<sub>n</sub>). Os ganhos padrão (0,5; 1; v = 0,5 m/s) dão ζ = 1.
- No mapa, a reta é marcada por um ponto dela e pelo sentido do percurso. A reta não tem fim:
  a corrida para quando |d| < 1 cm e o erro de direção < 1° ("convergiu").

Medido:

- Fig. 4.10, reta (1, −2, 4): as 4 partidas convergem sem cruzar a reta, em 7,65 a 19,15 s.
- A grade de 96 poses converge nos três modelos, a mais lenta em 20,9 s.
- Sobressinal: ζ = 0,3 dá 37,35% (teoria 37,23%); ζ = 0,5 dá 16,34% (16,30%).
- K<sub>d</sub> < 0, K<sub>h</sub> < 0 ou K<sub>h</sub> = 0: nenhuma das 96 converge.
- **Erro estacionário com uma roda maior.** Andar reto pede ω ≠ 0, e a lei proporcional só o dá
  com d ≠ 0. Sobra d = ω/K<sub>d</sub>, com ω = −v (r<sub>r</sub> − r) / (l (r<sub>r</sub> + r)). Com a
  direita 1, 2 e 4 mm maior, o robô assenta a −13,43, −26,78 e −53,29 mm da reta, como previsto,
  nos três modelos. Na pose e no ponto isso não acontece, porque v → 0 no alvo.

### Seguir um caminho: pure pursuit (4.1.1.3)

- O caminho são os waypoints ligados por retas e amostrados a cada 1 cm.
- p<sub>j</sub> é o ponto do caminho mais perto do robô, e p<sub>k</sub>, o ponto perseguido, é o
  primeiro depois de j a pelo menos d de p<sub>j</sub>.
- ω = K<sub>h</sub> (θ* ⊖ θ), com θ* rumo a p<sub>k</sub>, e v constante. É a lei do ponto com o alvo
  andando à frente do robô. Ganhos padrão: d = 0,5 m, K<sub>h</sub> = 2, v = 0,4 m/s.
- No fim, quando p<sub>k</sub> é o último ponto, v cai para v ρ/d: a chegada é a lei do ponto com
  K<sub>v</sub> = v/d. Chegar é estar a menos de 1 cm do fim com p<sub>j</sub> já no último trecho, para
  um caminho fechado não "chegar" na partida.
- O Corke procura j no caminho inteiro. Aqui a busca começa no j anterior e vai até max(1 m, 2d)
  adiante, para j não saltar de trecho quando o caminho se cruza.

Medido:

- Os cinco caminhos prontos chegam ao fim nos três modelos: o da Fig. 4.12 na escala 1:10
  (19,90 m, 55,4 s), zigue-zague, quadrado, oito e S.
- O robô corta as quinas. Numa quina de 90°, com d = 0,125, 0,25, 0,5 e 1 m: com K<sub>h</sub> = 20
  (v/K<sub>h</sub> = 2 cm), o corte é de 67, 143, 294 e 594 mm, e dobrar d dobra o corte; com
  K<sub>h</sub> = 2 (v/K<sub>h</sub> = 20 cm), é de 24, 76, 209 e 507 mm, e cresce mais que isso. O corte
  só é proporcional a d quando d ≫ v/K<sub>h</sub>.
- No cruzamento do oito, a busca no caminho inteiro faz j saltar de ramo uma vez e voltar; a
  busca a partir do j anterior nunca salta. Nos dois casos o robô termina.
- Com K<sub>h</sub> ≤ 0, o robô não vira para o caminho e não avança nele.

## Malha pela odometria

Medido no exemplo do Corke da pose, com 500 pulsos por volta:

- Sem erros físicos, o robô real termina a 7,5 mm do alvo. Com 10⁶ pulsos, a 3,3 mm, o mesmo que com
  a pose verdadeira.
- **O robô real não entra na malha.** A estimativa sai bit a bit igual com ou sem erro de roda: os
  pulsos dependem só dos comandos, e os comandos só da estimativa. Fechar a malha pela odometria é
  malha aberta em relação ao mundo.
- Com a roda direita 2 mm maior, a estimativa chega e o robô real fica a 240 mm e 7,2° do alvo;
  com a pose verdadeira, a malha corrige e o robô fica a 4,1 mm. Com 0,5, 1, 2 e 4 mm, o robô real
  fica a 65, 123, 240 e 477 mm.
- Com 5% de escorregamento na roda esquerda, o robô real fica a 1.116 mm e 36° do alvo.
- Na grade de 96 poses, com a direita 2 mm maior, a estimativa sempre chega e o robô real fica em
  média a 117 mm (no máximo 424 mm); com a pose verdadeira, no máximo 10 mm.
- No caminho do Corke, com a direita 2 mm maior, o robô real termina a 2,29 m do fim.

## Escolhas de interpretação

- O Corke escreve as leis para o modelo de bicicleta, com o esterçamento γ = atan(ωL/v). No
  diferencial o comando já é (v, ω), então cada lei entra direto em ω.
- Os exemplos são os das figuras do livro: a pose de (9, 5, 0°) a (5, 5, 90°); o ponto e a reta das
  Figs. 4.8 e 4.10; o caminho da Fig. 4.12, com os waypoints (10, 10), (10, 60), (80, 80) e
  (50, 10) divididos por 10, para caber na escala do P3-DX.
- O limite de 360°/s por roda vem do simulador da Aula 14 (dá v máx. de 0,61 m/s); não é do P3-DX
  real. Longe do alvo, v = k<sub>ρ</sub> ρ satura as rodas, e a saturação mantém o raio de curva.
- As tolerâncias de chegada (1 cm e 1°) e o tempo máximo (60 s) são da simulação e podem ser
  mudados na coluna da direita.
- No caminho, a condição exibida é só K<sub>h</sub> > 0. K<sub>h</sub> > v/d seria a condição da lei do
  ponto na chegada, mas não é necessária: com K<sub>h</sub> = 0,5 e v/d = 0,8, o caminho do Corke ainda
  chega ao fim (em 58 s, com α de 5,9°), porque α começa pequeno. O texto da página avisa que, abaixo
  disso, o robô chega torto.

## Validação

```bash
node testes/teste_controladores.js
```

São 62 conferências, em 11 seções:

1. coordenadas polares: casos feitos à mão, o β = −θ − α do livro e a invariância a girar a cena;
2. condições de estabilidade e escolha da ré;
3. convergência da pose nas 96 poses e nos três modelos, a ré e o exemplo do Corke;
4. cada condição violada faz a malha falhar, e a condição forte separa os ganhos em que α sai de
   (−90°, 90°];
5. saturação das rodas: o limite é respeitado e o raio de curva do comando, também;
6. Δt: ordem 1 do diferencial e ordem 2 do incremental contra o arco exato;
7. malha pela odometria;
8. ir a um ponto: exemplo do Corke, Fig. 4.8, grade e a espiral de K<sub>h</sub> < K<sub>v</sub>;
9. seguir uma reta: o sinal, a Fig. 4.10, a grade, o sobressinal e o erro estacionário;
10. seguir um caminho: amostragem, caminhos prontos, corte das quinas, o oito e a odometria;
11. o exemplo do Corke de cada lei, o mesmo da página de escolha, chega nos três modelos.

As duas páginas também são testadas no navegador, pelos testes de `testes/navegador/` (Node 22+, sem
dependências), que controlam um Edge sem janela pelo DevTools Protocol. Nesta pasta, com o servidor e
o Edge rodando, cada um no seu terminal:

```bash
python servidor.py --sem-navegador
msedge --headless=new --remote-debugging-port=9333 --user-data-dir=<pasta vazia>
node testes/navegador/rodar.mjs
```

O `msedge.exe` fica em `C:\Program Files (x86)\Microsoft\Edge\Application`. O `rodar.mjs` roda os
seis testes em sequência (143 conferências) e sai com erro se algum falhar; `rodar.mjs 3d` roda só
os que têm "3d" no nome. `PORTA` e `PORTA_CDP` mudam as portas do servidor (8000) e do Edge (9333).
As fotos dos testes vão para `testes/navegador/fotos/`, fora do git.

- Em cada algoritmo, a corrida na tela é bit a bit a de `simular()` nos três modelos, e a da
  odometria é a de `simularOdometria()`; com a vista 3D aberta, também.
- Também são conferidos:
  - os ganhos e as condições, também a forte, com os ganhos padrão e com (1, 2, −1);
  - os números dos eixos com passo de 2,5 (no histórico e no mapa), as legendas dos gráficos dentro
    da largura e o botão "Continuar";
  - a edição do alvo, da reta e do caminho no mapa (acrescentar, arrastar, inserir, apagar);
  - os exemplos e o seletor;
  - a URL do algoritmo;
  - as legendas e a animação da página de escolha;
  - na vista 3D: robôs, giro e diâmetro das rodas e rastros iguais aos do estado, os rótulos de
    cada algoritmo, as letras dos eixos que saem quando cobririam outro rótulo, "De cima", "Enquadrar", "Câmera segue o robô", o `#3d` da URL e o aviso quando a
    página é aberta como arquivo ou quando o three.js não chega;
  - o fundo claro com o sistema no modo escuro e a página a 390 px, nas duas vistas;
  - o console sem erros.
- O executável serve os mesmos arquivos da pasta (md5 iguais), e os mesmos testes passaram contra ele.

## Arquivos

| Arquivo | Conteúdo |
| --- | --- |
| `index.html`, `escolha.js` | A página de escolha e as miniaturas. |
| `simulacao.html`, `simulacao.js` | A página da simulação: estado, tempo, desenho do mapa e do histórico, eventos. |
| `vista3d.js` | A vista 3D (módulo ES): o P3-DX, os rastros e as marcas desenhados com o three.js a partir do estado da simulação, sem mudá-lo. |
| `vendor/three/` | three.js 0.160.0 (MIT), com o `OrbitControls` e o `CSS2DRenderer`; origem e sha256 em `LEIAME.md`. |
| `estilo.css` | Aparência das duas páginas (fundo sempre claro; o tema escuro só entra com `data-theme="dark"` na raiz) e as cores usadas no desenho (no mapa e na vista 3D). |
| `controladores.js` | As quatro leis, a malha fechada, a malha pela odometria e os exemplos do Corke. Não depende da página: roda no navegador e no Node. |
| `cinematica.js` | Cópia do simulador da Aula 14: direta, inversa, saturação, os três passos de integração, os pulsos do encoder e o passo da odometria. |
| `servidor.py` | Servidor local (biblioteca padrão do Python). |
| `ControleP3DX.spec` | Receita do PyInstaller para o executável do Windows. |
| `testes/teste_controladores.js` | Os testes em Node. |
| `testes/navegador/` | Os testes no navegador: `cdp.mjs` (o cliente do DevTools Protocol), um teste por parte da página e o `rodar.mjs`, que roda todos. |
| `COLCON_IGNORE` | Faz o colcon pular esta pasta quando o workspace ROS 2 é compilado. |
