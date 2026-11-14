# Wizualizacja 3D i Optymalizacja Trajektorii Wyścigowej (`racing_line_3d`)

Aplikacja napisana w języku **Rust** do symulacji fizycznej, optymalizacji trajektorii przejazdu (*racing line*) przez trójwymiarowy zakręt toru wyścigowego oraz interaktywnej wizualizacji 3D (`three-d` + `egui` + `egui_plot` + `glam`).

---

## Model fizyczny i parametry pojazdu

### 1. Domyślne parametry pojazdu (`VehicleParams`)
Symulacja odwzorowuje samochód wyścigowy klasy **GT3 / Formula** o następujących parametrach bazowych (wszystkie kluczowe parametry można płynnie zmieniać suwakami na żywo w panelu bocznym):

| Parametr | Symbol | Wartość domyślna | Zakres w UI | Znaczenie fizyczne |
| :--- | :---: | :---: | :---: | :--- |
| Współczynnik przyczepności opon | $\mu$ | **1.40** | $0.70 \text{ -- } 2.20$ | Stosunek maksymalnej siły tarcia opon (slick) do siły nacisku normalnego |
| Maks. przyspieszenie silnika | $a_{\text{acc}}$ | **8.0 m/s²** (~0.82 G) | $3.0 \text{ -- } 15.0\text{ m/s}^2$ | Limit przyspieszenia wzdłużnego wynikający z mocy silnika i napędu |
| Maks. opóźnienie hamowania | $a_{\text{brake}}$ | **14.0 m/s²** (~1.43 G) | $6.0 \text{ -- } 22.0\text{ m/s}^2$ | Limit opóźnienia układu hamulcowego wraz z oporem aerodynamicznym |
| Prędkość maksymalna na prostej | $v_{\text{top}}$ | **78.0 m/s** (~280 km/h) | stała | Prędkość graniczna na długich odcinkach prostych |
| Przyspieszenie ziemskie | $g$ | **9.81 m/s²** | stała | Stała grawitacyjna |
| Szerokość toru | $W$ | **12.0 m** | $8.0 \text{ -- } 20.0\text{ m}$ | Odległość między lewą a prawą krawędzią asfaltu |
| Kąt pochylenia zakrętu (banking) | $\beta$ | **8.0°** | $0.0^\circ \text{ -- } 15.0^\circ$ | Nachylenie poprzeczne jezdni do wnętrza łuku |
| Margines bezpieczeństwa | $m$ | **0.8 m** | $0.3 \text{ -- } 2.5\text{ m}$ | Minimalny odstęp środka pojazdu od krawędzi toru |

---

### 2. Model punktowy z kołem tarcia (*Friction Circle*)
W symulacji pojazd traktowany jest jako punkt materialny poruszający się po powierzchni toru 3D. Opony posiadają **skończony budżet przyczepności**, który można przedstawić geometrycznie jako koło o promieniu równym maksymalnemu przyspieszeniu poziomemu $a_{\max}$:

$$\sqrt{a_{\text{lat}}^2 + a_{\text{long}}^2} \le a_{\max}$$

gdzie:
- $a_{\text{lat}} = \kappa v^2 = \frac{v^2}{R}$ to **przyspieszenie boczne (dośrodkowe)** wynikające z jazdy po łuku o promieniu $R = \frac{1}{\kappa}$,
- $a_{\text{long}} = \frac{dv}{dt} = v \frac{dv}{ds}$ to **przyspieszenie wzdłużne** (dodatnie podczas przyspieszania, ujemne podczas hamowania).

**Co to oznacza w praktyce?**
- Gdy kierowca skręca na pełnym limicie przyczepności ($a_{\text{lat}} = a_{\max}$, sam szczyt zakrętu – *apex*), dostępny zapas na przyspieszanie lub hamowanie wynosi zero ($a_{\text{long}} = 0$).
- Aby mocno zahamować przed zakrętem lub mocno przyspieszyć na wyjściu, tor jazdy musi mieć w tych miejscach **mniejszą krzywiznę $\kappa$ (większy promień $R$)**, co zwalnia część budżetu koła tarcia:
  $$a_{\text{long, avail}} = a_{\max} \cdot \sqrt{1 - \left(\frac{a_{\text{lat}}}{a_{\max}}\right)^2}$$

---

### 3. Wpływ kąta pochylenia zakrętu ($\beta$) oraz przyczepności ($\mu$)
Gdy zakręt jest wyprofilowany do wewnątrz pod kątem $\beta$ (*banking*), pojazdowi pomagają dwa zjawiska:
1. Składowa siły ciężkości $\sin\beta$ bezpośrednio ściąga auto do środka łuku (przeciwdziałając sile odśrodkowej),
2. Siła odśrodkowa na pochylonej nawierzchni dodatkowo **dociska opony prostopadle do asfaltu**, zwiększając siłę normalną $N$, a tym samym maksymalną siłę tarcia $\mu N$.

Z równowagi sił w przekroju poprzecznym wynika dokładny wzór na maksymalne przyspieszenie poziome:

$$a_{\max}(\beta) = g \cdot \frac{\mu \cos\beta + \sin\beta}{\cos\beta - \mu \sin\beta}$$

Zwiększenie kąta $\beta$ lub współczynnika tarcia $\mu$ podnosi $a_{\max}(\beta)$, co bezpośrednio zwiększa **prędkość graniczną w zakręcie**:

$$v_{\text{lat}} = \sqrt{a_{\max}(\beta) \cdot R} = \sqrt{\frac{a_{\max}(\beta)}{\kappa}}$$

---

### 4. Algorytm *Forward-Backward Pass* i tajemnica *Late Apex* (późnego wierzchołka)
Dla dowolnego kształtu ścieżki (opisanego parametrami poprzecznymi $\alpha_i \in [0, 1]$ na kolejnych przekrojach toru) profil prędkości $v(s)$ wyznaczany jest w trzech krokach:

1. **Ograniczenie boczne (Cornering Limit):**
   W każdym punkcie $i$ wyliczana jest krzywizna $\kappa_i$ (z trzech sąsiednich punktów 3D) i przypisywana maksymalna prędkość w łuku: $v_i \leftarrow \min(v_{\text{top}}, \sqrt{a_{\max, i} / \kappa_i})$.
2. **Przebieg wsteczny (*Backward Pass* – strefa hamowania):**
   Algorytm idzie **od końca toru do początku** ($i = N-2 \dots 0$). Jeśli w punkcie $i+1$ prędkość w łuku jest niska, to w punkcie $i$ auto nie może jechać szybciej niż prędkość, z której zdąży wyhamować na odcinku $\Delta s_i$:
   $$v_i \le \sqrt{v_{i+1}^2 + 2 \cdot \min(a_{\text{brake}}, a_{\text{long, avail}}) \cdot \Delta s_i}$$
   Ten krok automatycznie wyznacza **punkt rozpoczęcia hamowania** przed zakrętem (kolor czerwony).
3. **Przebieg w przód (*Forward Pass* – strefa przyspieszania):**
   Algorytm idzie **od początku do końca toru** ($i = 0 \dots N-2$). Od najwolniejszego punktu w szczycie zakrętu (*apex*) prędkość w kolejnym punkcie $i+1$ rośnie tylko o tyle, na ile pozwala moc silnika i zapas przyczepności opon:
   $$v_{i+1} \le \sqrt{v_i^2 + 2 \cdot \min(a_{\text{acc}}, a_{\text{long, avail}}) \cdot \Delta s_i}$$
   Ten krok wyznacza profil wyjścia z zakrętu na pełnym gazie (kolor zielony).

**Dlaczego linia wyścigowa wybiera późny wierzchołek (*Late Apex*)?**
Samochód hamuje znacznie gwałtowniej ($a_{\text{brake}} = 14\text{ m/s}^2$) niż przyspiesza ($a_{\text{acc}} = 8\text{ m/s}^2$). Dlatego:
- **Linia geometryczna** (cyjanowa) minimalizuje samą krzywiznę symetrycznie pośrodku zakrętu, ale zmusza kierowcę do długiego skręcania na wyjściu, co opóźnia wciśnięcie gazu.
- **Linia wyścigowa** (czerwono-żółto-zielona) poświęca nieco prędkości na wejściu (gdzie hamulce i tak są bardzo skuteczne), przesuwa wierzchołek zakrętu dalej w stronę wyjścia (*Late Apex*) i **wcześniej prostuje tor jazdy**. Dzięki temu kierowca może wcześniej otworzyć pełny gaz i osiąga wyższą prędkość na całej prostej wyjściowej, skracając całkowity czas przejazdu $T = \sum \frac{\Delta s_i}{v_i}$.

---

## Struktura projektu

- `src/track.rs` – generowanie geometrii zakrętu 3D (presety: *Parabolica*, *Szykana S*, *Nawrót 167°*), obliczanie przekrojów poprzecznych, kąta bankingu $\beta$, krawężników i poprzecznych żeberek siatki.
- `src/physics.rs` – model fizyczny pojazdu, koło tarcia (*Friction Circle*), wpływ pochylenia zakrętu $\beta$ oraz dwukierunkowy algorytm *Forward-Backward Pass*.
- `src/optimizer.rs` – wyznaczanie i porównywanie 3 trajektorii (środkowej, geometrycznej i czasowo-optymalnej z efektem *Late Apex*) oraz budowanie kolorowanych wstęg 3D.
- `src/ui.rs` – panel sterowania `egui`, zwijana sekcja z opisem fizyki oraz wykresy telemetryczne `egui_plot` ($v(s)$ i diagram G-G).
- `src/main.rs` – inicjalizacja okna `winit` + `three-d`, kamery orbitalnej i głównej pętli renderowania 3D.
- `tests/trajectory_tests.rs` – testy jednostkowe i integracyjne weryfikujące geometrię, granice toru oraz limity fizyczne.

---

## Uruchomienie i testy

### 1. Uruchomienie aplikacji 3D
```powershell
cargo run --release
```

### 2. Sterowanie w oknie 3D
- **Lewy przycisk myszy (przeciąganie):** obrót kamery orbitalnej wokół zakrętu,
- **Prawy przycisk myszy (przeciąganie):** przesuwanie punktu obserwacji (pan),
- **Rolka myszy:** przybliżanie i oddalanie widoku (zoom),
- **Panel boczny po lewej:** zmiana kształtu zakrętu, zakładka *„Model fizyczny i parametry”*, suwaki parametrów ($\mu$, kąt bankingu $\beta$, szerokość $W$, limity hamowania i przyspieszania) oraz wykresy $v(s)$ i G-G.

### 3. Uruchomienie testów jednostkowych
```powershell
cargo test
```
