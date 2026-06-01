# quilojaules

Calculadora de quilojaules — estima o trabalho mecânico (kJ) e as calorias para
pedalar uma rota, com cálculo **por trecho** (parâmetros próprios por segmento).

Aplicativo estático (HTML + JS + CSS, sem build). Abra `index.html` num
navegador, ou sirva a pasta (`python -m http.server`). Algumas funções precisam
de rede (tiles, elevação FABDEM, roteamento OSRM); a física roda local.

Layout de coluna única: não há perfil global — **todos os parâmetros são por
trecho**, editados na tabela de trechos.

## Como usar

1. **Rota** — arraste um `.gpx`, clique em *rota de exemplo (SP)*, ou
   *desenhar no mapa* (clique para adicionar pontos; *seguir vias* usa o OSRM).
   POIs (`<wpt>`) do GPX aparecem com ícones padrão e têm um botão liga/desliga
   no controle de camadas (*POIs (GPX)*).
2. **Recorte** — o slider duplo isola um sub-trecho da rota.
3. **Trechos** — no modo *dividir*, clique no gráfico de elevação ou na linha do
   mapa para dividir a rota; edite os parâmetros de cada trecho na tabela
   (célula vazia herda o padrão). Cada trecho mostra seu **tempo em movimento**
   estimado e tem um botão **✕** para removê-lo (funde no vizinho). A energia
   total soma os trechos. Os rótulos aparecem no mapa e no perfil; edite-os na
   coluna *trecho*.
4. **Zoom no perfil** — alterne o gráfico para o modo *zoom* e arraste para
   ampliar uma faixa; *reset zoom* volta à vista cheia.
5. **Camadas** — OpenStreetMap, Satélite, ou **Topográfica colorida**
   (renderizada no cliente a partir do FABDEM): elevação em `cmocean.phase`
   com declividade em *multiply* por cima. No painel **Topografia colorida**
   (abaixo do mapa) ajuste `elevação mín/máx`, `declividade máx` e
   `amostragem máx. (MB / mapa)`. A camada **amostra** do FABDEM dentro desse
   limite **por visualização do mapa** — o orçamento é dividido entre os tiles à
   vista (via overviews do COG), então um render completo baixa ≤ o limite, e
   nunca os tiles inteiros. A declividade é calculada na resolução amostrada
   (sem efeito quadriculado).

## Como os quilojoules são calculados

O número grande (kJ) é o **trabalho mecânico no pedal** necessário para vencer
a rota — não as calorias do corpo (essas vêm depois). O cálculo é uma soma
sobre pequenos segmentos: a rota (do GPX, do desenho ou recortada) é uma
sequência de pontos `{lat, lon, elevação}`, e cada par de pontos consecutivos
é um segmento.

### 1. Geometria do segmento

Para cada segmento entre os pontos *a* e *b*:

```text
horiz = distância de Haversine(a, b)        # no plano, em metros
ΔH    = elev(b) − elev(a)                    # ganho/perda de elevação
L     = √(horiz² + ΔH²)                       # comprimento 3D do segmento
sinθ  = ΔH / L                                # inclinação (seno)
cosθ  = √(1 − sin²θ)
```

A elevação vem do FABDEM (rotas desenhadas e GPX sem `ele`) ou do próprio GPX.

### 2. Forças (equação de potência de Martin et al., 1998)

Com massa total `m = ciclista + bike`, gravidade `g = 9,80665` e os parâmetros
do trecho:

```text
F_roll = Crr · m · g · cosθ                  # rolamento
F_grav = m · g · sinθ                         # gravidade (− na descida)
v_eff  = v + vento_contra
F_aero = ½ · ρ · CdA · v_eff · |v_eff|        # arrasto (sinalizado)
P_roda = (F_roll + F_grav + F_aero) · v       # potência na roda
```

A potência no **pedal** divide pela eficiência da transmissão:
`P_pedal = P_roda / η`. **Potência negativa é zerada** — descer ou frear não
recupera energia (`E = max(P_roda, 0) / η · Δt`).

### 3. De onde vem a velocidade *v*

- **Rota gravada (com timestamps no GPX):** `v = L / Δt` a partir dos horários.
  Segmentos abaixo de 0,5 m/s contam como parada e são ignorados — a duração é
  *tempo em movimento*, não tempo total.
- **Rota planejada (sem timestamps):** assume-se **esforço constante por tipo
  de terreno**. Pela inclinação, escolhe-se uma potência-alvo (subida / plano /
  descida, classificadas pelo limiar `grau±`) e resolve-se por bissecção a
  velocidade `v` em que `P_pedal(v)` iguala esse alvo, limitada pela velocidade
  máxima de descida (freio). Na descida com alvo 0, usa-se a velocidade de
  equilíbrio (gravidade vs. rolamento + arrasto), também limitada pelo freio.

### 4. Soma por trecho e total

Cada **trecho** usa seus próprios parâmetros (os da tabela sobrepõem os padrões:
`{...padrão, ...overrides do trecho}`). A energia de cada trecho é a integral
`Σ P_pedal · Δt` sobre seus segmentos, e o total é a soma dos trechos:

```text
E_total (J) = Σ_trechos Σ_segmentos max(P_roda, 0)/η · Δt
kJ          = E_total / 1000
```

A repartição "onde vão os watts" separa essa energia em rolamento, aero e
subida (`F_roll`, `F_aero`, `F_grav`, cada um zerado quando negativo).

### 5. Calorias e potência média

As **calorias** assumem ~24% de eficiência muscular bruta:

```text
kcal = (kJ / 0,24) / 4,184
```

A **potência média** é `E_total / duração` (W); `W/kg` divide pela massa do
ciclista. O `recorte` e os `trechos` recalculam tudo só sobre a porção ativa.

## Estado salvo (RDF / SHACL)

`shapes.ttl` define o vocabulário `qj:` e as formas **SHACL** que especificam
toda a entrada do usuário — `qj:TopographyConfig`, `qj:Profile`, `qj:Segment`
(configuração por trecho), `qj:Route` (rota de origem, pontos em
`qj:coordinates`) e o `qj:AppState` que os agrega. O botão **exportar .ttl.gz**
serializa o estado atual nesse formato (um grafo conforme às formas) comprimido
com gzip; **importar** restaura tudo, incluindo a rota. N3.js (parser Turtle) é
carregado sob demanda do CDN só na importação.

## Perfil de potência

A linha de potência usa no máximo **200 pontos**, cada um a **média de potência**
no seu intervalo de tempo (tempo total ÷ 200) — média no tempo, não medida
instantânea — plotada na distância atingida no meio do intervalo.

## Arquivos

- `index.html` — marcação + CDNs (Leaflet, geotiff.js; N3.js sob demanda).
- `style.css` — estilos.
- `app.js` — toda a lógica (física, FABDEM, camada topográfica, rota, trechos,
  POIs, export/import RDF).
- `shapes.ttl` — vocabulário + formas SHACL do grafo de estado.

Elevação: FABDEM via `telhas.pedalhidrografi.co` (fallback Open-Meteo).
As pastas `pedalhidrografico/` e `old-applet/` são referência, não fazem parte
do app.
