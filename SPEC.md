Jesteś doświadczonym programistą Rusta (Senior Rust Engineer) specjalizującym się w symulacjach fizycznych, matematyce obliczeniowej i grafice 3D. Twoim zadaniem jest stworzenie projektu: **Wizualizacja 3D oraz algorytmiczna optymalizacja trajektorii przejazdu (racing line) przez zakręt toru wyścigowego**.

### ZASADY WSPÓŁPRACY I STYLU KODU (BARDZO WAŻNE):
1. **Tryb konsultacji (Checkpoints):** Nie buduj wszystkiego naraz bez mojej wiedzy. Po każdym etapie ZATRZYMAJ SIĘ, przedstaw swoje propozycje i zapytaj mnie o zdanie, co dodać, co zmienić, a z czego zrezygnować.
2. **Proste komentarze:** Pisz w kodzie krótkie, proste i zrozumiałe komentarze po polsku (wyjaśniające *co* robi dany fragment, bez akademickiego żargonu).
3. **Samodzielny dobór bibliotek:** Sam dobierz najlepsze i stabilne crate'y z ekosystemu Rusta (np. do grafiki 3D/okna – jak `bevy`, `three-d` lub `kiss3d` + `egui` do suwaków parametrów; oraz do matematyki/splajnów/optymalizacji – jak `nalgebra`, `glam`, `splines` itp.) i pobierz je za pomocą `cargo add`.
4. **Bezpieczeństwo i limity:** Unikaj nadmiernego komplikowania typów i lifetimes. Jeśli napotkasz błąd kompilacji, którego nie rozwiążesz w 3 próbach – zatrzymaj się i opisz mi problem.

---

### ZAŁOŻENIA GEOMETRYCZNE I FIZYCZNE MODELU:
1. **Geometria zakrętu 3D:**
   - Domyślny kształt: zakręt typu **„parabolica” lub szykana 90°–120°** o zmiennym promieniu, szerokości toru W = 12 m oraz regulowanym **pochyleniu poprzecznym (kąt bankingu beta, np. 0°–15°)** i lekkiej zmianie wysokości (spadek/podjazd na wyjściu).
   - Reprezentacja toru jako dyskretna seria przekrojów poprzecznych (np. 100–200 punktów wzdłuż linii środkowej z lewą i prawą krawędzią oraz wektorem normalnym powierzchni).
2. **Model fizyczny pojazdu (Uproszczony model punktowy z kołem tarcia):**
   - **Przyczepność (Friction Circle):** Maksymalne wypadkowe przyspieszenie poziome jest ograniczone współczynnikiem tarcia opon mi (np. mi = 1.4 dla auta wyścigowego) oraz składową grawitacji wynikającą z pochylenia zakrętu beta:
     `a_max = g * (mi * cos(beta) + sin(beta)) / (cos(beta) - mi * sin(beta))`
   - **Prędkość graniczna w zakręcie:** Dla lokalnego promienia krzywizny `R = 1 / krzywizna` maksymalna prędkość boczna wynosi `v_lat = sqrt(a_max * R)`.
   - **Dynamika wzdłużna (Forward-Backward Pass):** Uwzględnij osobne limity maksymalnego przyspieszenia silnika (`a_acc`, np. 8 m/s²) oraz maksymalnego hamowania (`a_brake`, np. 14 m/s²), aby trajektoria naturalnie wyznaczała punkt hamowania przed zakrętem, wierzchołek (*apex*) i wyjście z pełnym gazem.
3. **Cel optymalizacji:**
   - Znalezienie ciągłej krzywej (np. parametryzowanej współczynnikiem `alpha` z przedziału `[0, 1]` między lewą a prawą krawędzią toru na każdym przekroju), która **minimalizuje całkowity czas przejazdu** `T = suma(ds_i / v_i)` przy zachowaniu marginesu bezpieczeństwa od krawędzi toru.

---

### ETAP 1: Propozycja architektury i bibliotek (STOP PO TYM ETAPIE)
1. Zaproponuj 2 opcje podejścia do silnika 3D i interfejsu (np. lekki renderer typu `three-d` + `egui` vs. pełny silnik `bevy`) wraz z krótkim wyjaśnieniem plusów i minusów każdej z nich dla tego konkretnego zadania.
2. Wypisz listę bibliotek (crate'ów), które proponujesz pobrać do obsługi geometrii 3D, obliczeń wektorowych, optymalizacji numerycznej (np. gradient descent / L-BFGS / metoda relaksacji krzywizny) oraz interfejsu UI.
3. Zaproponuj listę opcjonalnych funkcji (np. porównanie linii geometrycznej vs. wyścigowej, wykres prędkości i przeciążeń G, suwak współczynnika przyczepności mi i kąta bankingu beta, animacja przejazdu pojazdu).
4. **ZATRZYMAJ SIĘ** i zapytaj mnie:
   - Który zestaw bibliotek wybieram?
   - Które z zaproponowanych funkcji dodać, a które pominąć?

---

### ETAP 2: Inicjalizacja projektu i geometria toru 3D (STOP PO TYM ETAPIE)
1. Po mojej akceptacji zainicjuj projekt (`cargo init`), pobierz zatwierdzone biblioteki przez `cargo add` i stwórz czytelną strukturę plików w `src/` (np. `track.rs`, `physics.rs`, `optimizer.rs`, `ui.rs`, `main.rs`).
2. Zaimplementuj generowanie siatki (mesh) zakrętu 3D zgodnie z założeniami geometrycznymi (z krawężnikami, pochyleniem beta i zmianą wysokości).
3. Uruchom `cargo check`, upewnij się, że nie ma błędów kompilacji, i **ZATRZYMAJ SIĘ**, pytając mnie o akceptację przed przejściem do algorytmu optymalizacji.

---

### ETAP 3: Algorytm optymalizacji, profil prędkości i wizualizacja 3D
1. Zaimplementuj moduł fizyki (`physics.rs` – obliczanie krzywizny, prędkości granicznej w zakręcie oraz dwukierunkowy przebieg *forward-backward* dla hamowania i przyspieszania) oraz optymalizator trajektorii (`optimizer.rs`).
2. Wyrenderuj w oknie 3D:
   - powierzchnię zakrętu z siatką ułatwiającą ocenę pochylenia,
   - wyliczoną optymalną trajektorię pokolorowaną według fazy jazdy (czerwony = hamowanie, żółty = szczyt zakrętu / limit przyczepności, zielony = przyspieszanie),
   - panel UI z suwakami na żywo (mi, kąt pochylenia beta, szerokość toru, limit hamowania) oraz wyświetlaniem wyliczonego czasu przejazdu w sekundach.
3. Dodaj testy jednostkowe w `tests/` sprawdzające, czy trajektoria mieści się w granicach toru oraz czy profil prędkości nie przekracza zadanego limitu przyczepności.
4. Uruchom `cargo build` oraz `cargo test`, a na końcu wygeneruj prosty plik `README.md` z instrukcją obsługi.