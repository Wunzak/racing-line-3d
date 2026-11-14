use glam::{Mat3, Vec3};
use three_d::{CpuMesh, Indices, Positions, Srgba};

/// Dostępne kształty zakrętu (presety geometrii)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackPreset {
    /// Zakręt typu Parabolica – ciasne wejście i stopniowo rosnący promień na wyjściu
    Parabolica,
    /// Szykana S (90°–120°) – szybka zmiana kierunku prawo-lewo
    ChicaneS,
    /// Nawrót o 165°–170° (Hairpin) o bezpiecznym promieniu R ~ 20 m
    Hairpin,
}

impl TrackPreset {
    /// Czytelna nazwa presetu wyświetlana w interfejsie UI
    pub fn label(&self) -> &'static str {
        match self {
            TrackPreset::Parabolica => "Parabolica (zmienny promień)",
            TrackPreset::ChicaneS => "Szykana S (90°–120°)",
            TrackPreset::Hairpin => "Nawrót 167° (Hairpin, R=20m)",
        }
    }
}

/// Parametry geometryczne toru sterowane z poziomu UI
#[derive(Debug, Clone)]
pub struct TrackParams {
    /// Wybrany kształt zakrętu
    pub preset: TrackPreset,
    /// Liczba przekrojów poprzecznych wzdłuż toru (100–200)
    pub num_sections: usize,
    /// Szerokość toru W [m] (domyślnie 12.0 m)
    pub width: f32,
    /// Maksymalny kąt pochylenia poprzecznego (banking beta) w stopniach (0°–15°)
    pub banking_deg: f32,
    /// Maksymalna zmiana wysokości (podjazd/spadek na wyjściu) [m]
    pub elevation_change: f32,
    /// Margines bezpieczeństwa od krawędzi toru [m]
    pub safety_margin: f32,
}

impl Default for TrackParams {
    fn default() -> Self {
        Self {
            preset: TrackPreset::Parabolica,
            num_sections: 160,
            width: 12.0,
            banking_deg: 8.0,
            elevation_change: 3.0,
            safety_margin: 0.8,
        }
    }
}

/// Pojedynczy przekrój poprzeczny toru 3D
#[derive(Debug, Clone)]
pub struct CrossSection {
    /// Dystans od początku toru wzdłuż linii środkowej [m]
    pub s: f32,
    /// Punkt na linii środkowej 3D (oś obrotu dla kąta bankingu)
    pub center: Vec3,
    /// Znormalizowany wektor styczny (kierunek jazdy wzdłuż toru)
    pub tangent: Vec3,
    /// Znormalizowany wektor poprzeczny (od lewej do prawej krawędzi, obrócony o banking wokół stycznej)
    pub lateral: Vec3,
    /// Znormalizowany wektor normalny powierzchni toru (prostopadły do asfaltu)
    pub normal: Vec3,
    /// Współrzędne 3D lewej krawędzi toru (alpha = 0.0)
    pub left_edge: Vec3,
    /// Współrzędne 3D prawej krawędzi toru (alpha = 1.0)
    pub right_edge: Vec3,
    /// Lokalny efektywny kąt pochylenia zakrętu beta [rad] (wartość bezwzględna w łuku)
    pub banking_rad: f32,
    /// Znak kierunku zakrętu w płaszczyźnie poziomej (+1 = w prawo, -1 = w lewo, 0 = prosta)
    pub turn_sign: f32,
}

/// Pełna dyskretna reprezentacja zakrętu 3D
#[derive(Debug, Clone)]
pub struct Track3D {
    pub params: TrackParams,
    pub sections: Vec<CrossSection>,
    pub total_length: f32,
}

impl Track3D {
    /// Generuje serię przekrojów poprzecznych zakrętu 3D na podstawie parametrów
    pub fn generate(params: TrackParams) -> Self {
        let n = params.num_sections.max(40);

        // Całkowita długość linii środkowej dobrana do charakterystyki każdego zakrętu
        let total_s = match params.preset {
            TrackPreset::Parabolica => 210.0,
            TrackPreset::ChicaneS => 225.0,
            TrackPreset::Hairpin => 190.0,
        };
        let ds = total_s / (n - 1) as f32;

        // Krok 1: Wyznaczenie profilu krzywizny linii środkowej
        let mut raw_curvatures: Vec<f32> = (0..n)
            .map(|i| {
                let u = i as f32 / (n - 1) as f32;
                Self::preset_curvature(params.preset, u)
            })
            .collect();

        // Dla nawrotu (Hairpin) normalizujemy całkę krzywizny dokładnie do kąta 167° (~2.915 rad)
        // przy promieniu minimalnym R ≈ 20 m, aby prosta wejściowa i wyjściowa miały duży odstęp (~44 m)
        // i lekko rozchodziły się na zewnątrz, nigdy na siebie nie nachodząc.
        if params.preset == TrackPreset::Hairpin {
            let current_total_angle: f32 = raw_curvatures.iter().map(|&k| k * ds).sum();
            let target_angle_rad = 167.0_f32.to_radians();
            if current_total_angle.abs() > 1e-5 {
                let scale = target_angle_rad / current_total_angle;
                for k in &mut raw_curvatures {
                    *k *= scale;
                }
            }
        }

        // Krok 2: Całkowanie pozycji linii środkowej (centerline) w przestrzeni 3D
        let mut raw_centers = Vec::with_capacity(n);
        let mut x = 0.0_f32;
        let mut z = 0.0_f32;
        let mut heading = 0.0_f32;

        for (i, &kappa) in raw_curvatures.iter().enumerate() {
            let u = i as f32 / (n - 1) as f32;
            let y = Self::preset_elevation(params.preset, u, params.elevation_change);
            raw_centers.push(Vec3::new(x, y, z));

            // Całkowanie metodą punktu środkowego dla gładkiego łuku
            let next_heading = heading + kappa * ds;
            let mid_heading = 0.5 * (heading + next_heading);
            x += mid_heading.sin() * ds;
            z -= mid_heading.cos() * ds;
            heading = next_heading;
        }

        // Wyśrodkowanie zakrętu wokół (0, 0, 0) w płaszczyźnie XZ dla wygody obracania kamery
        let centroid = raw_centers.iter().copied().sum::<Vec3>() / (n as f32);
        for p in &mut raw_centers {
            p.x -= centroid.x;
            p.z -= centroid.z;
        }

        // Krok 3: Wyznaczenie gładkiego profilu pochylenia poprzecznego (bankingu beta)
        // Zastosowano przejścia kosinusoidalne na prostych dojazdowych oraz dodatkowe wygładzanie,
        // aby wyeliminować nagłe skręcenia siatki nawet dla beta = 15°.
        let max_bank_rad = params.banking_deg.to_radians();
        let mut bank_angles: Vec<f32> = (0..n)
            .map(|i| {
                let u = i as f32 / (n - 1) as f32;
                Self::preset_banking_factor(params.preset, u) * max_bank_rad
            })
            .collect();
        smooth_profile(&mut bank_angles, 6);

        // Krok 4: Budowa lokalnego układu odniesienia (styczna, poprzeczna, normalna) dla każdego przekroju.
        // Obrót o kąt bankingu odbywa się ściśle wokół lokalnej osi stycznej przechodzącej przez środek toru (`center`).
        let mut sections = Vec::with_capacity(n);
        let mut cumulative_s = 0.0_f32;
        let half_w = 0.5 * params.width;

        for i in 0..n {
            if i > 0 {
                cumulative_s += raw_centers[i].distance(raw_centers[i - 1]);
            }

            let tangent_raw = if i == 0 {
                raw_centers[1] - raw_centers[0]
            } else if i == n - 1 {
                raw_centers[n - 1] - raw_centers[n - 2]
            } else {
                raw_centers[i + 1] - raw_centers[i - 1]
            };
            let tangent = tangent_raw.normalize_or_zero();

            // Poziomy wektor kierunkowy gwarantuje brak sztucznego skręcenia (torsji) na podjazdach
            let horiz_dir = Vec3::new(tangent.x, 0.0, tangent.z).normalize_or_zero();
            let flat_right = horiz_dir.cross(Vec3::Y).normalize_or_zero();
            let base_up = flat_right.cross(tangent).normalize_or_zero();

            // Dodatni kąt bankingu dla skrętu w prawo (kappa > 0) obniża prawą (wewnętrzną) krawędź
            // i unosi lewą (zewnętrzną) krawędź, pochylając wektor normalny do wnętrza zakrętu.
            let signed_bank_rad = bank_angles[i];
            let rot = Mat3::from_axis_angle(tangent, signed_bank_rad);

            let lateral = (rot * flat_right).normalize_or_zero();
            let normal = (rot * base_up).normalize_or_zero();

            let center = raw_centers[i];
            let left_edge = center - lateral * half_w;
            let right_edge = center + lateral * half_w;

            sections.push(CrossSection {
                s: cumulative_s,
                center,
                tangent,
                lateral,
                normal,
                left_edge,
                right_edge,
                banking_rad: signed_bank_rad.abs(),
                turn_sign: raw_curvatures[i].signum(),
            });
        }

        Self {
            params,
            sections,
            total_length: cumulative_s,
        }
    }

    /// Zwraca lokalną krzywiznę [1/m] linii środkowej dla danego presetu i parametru u w [0, 1]
    fn preset_curvature(preset: TrackPreset, u: f32) -> f32 {
        match preset {
            TrackPreset::Parabolica => {
                // Prosta dojazdowa -> ciasne wejście (R ~ 35 m) -> stopniowo otwierający się łuk na wyjściu (~105° łącznie)
                let entry = cosine_ramp(0.10, 0.28, u);
                let exit = 1.0 - cosine_ramp(0.74, 0.92, u);
                let parabolic_decay = 1.0 - 0.56 * cosine_ramp(0.28, 0.86, u);
                0.028 * entry * exit * parabolic_decay
            }
            TrackPreset::ChicaneS => {
                // Szykana 90°–120°: płynny łuk w prawo, gładkie przejście przez zero i łuk w lewo
                let lobe1 = cosine_ramp(0.10, 0.24, u) * (1.0 - cosine_ramp(0.36, 0.50, u));
                let lobe2 = cosine_ramp(0.50, 0.64, u) * (1.0 - cosine_ramp(0.76, 0.90, u));
                0.030 * lobe1 - 0.028 * lobe2
            }
            TrackPreset::Hairpin => {
                // Nawrót o promieniu minimalnym R ≈ 20 m (kappa_max ≈ 0.050 1/m) i kącie całkowitym 167°.
                // Proste wejściowa i wyjściowa mają wyraźny odstęp (~44 m) i nigdy na siebie nie nachodzą.
                let entry = cosine_ramp(0.20, 0.36, u);
                let exit = 1.0 - cosine_ramp(0.64, 0.80, u);
                0.050 * entry * exit
            }
        }
    }

    /// Zwraca znormalizowany współczynnik pochylenia poprzecznego w [-1, 1] wzdłuż parametru u w [0, 1].
    /// Wykorzystuje kosinusoidalne funkcje przejścia, dzięki czemu banking narasta od 0° na prostej
    /// do pełnej wartości beta w łuku i gładko wraca do 0° na wyjściu.
    fn preset_banking_factor(preset: TrackPreset, u: f32) -> f32 {
        match preset {
            TrackPreset::Parabolica => {
                let entry = cosine_ramp(0.06, 0.28, u);
                let exit = 1.0 - cosine_ramp(0.72, 0.94, u);
                let decay = 1.0 - 0.35 * cosine_ramp(0.32, 0.88, u);
                entry * exit * decay
            }
            TrackPreset::ChicaneS => {
                let right_bank = cosine_ramp(0.07, 0.23, u) * (1.0 - cosine_ramp(0.35, 0.50, u));
                let left_bank = cosine_ramp(0.50, 0.65, u) * (1.0 - cosine_ramp(0.77, 0.93, u));
                right_bank - left_bank
            }
            TrackPreset::Hairpin => {
                let entry = cosine_ramp(0.14, 0.34, u);
                let exit = 1.0 - cosine_ramp(0.66, 0.86, u);
                entry * exit
            }
        }
    }

    /// Zwraca wysokość Y [m] na przekroju u w [0, 1]
    fn preset_elevation(preset: TrackPreset, u: f32, amp: f32) -> f32 {
        match preset {
            TrackPreset::Parabolica => {
                // Łagodny spadek na dohamowaniu i płynny podjazd na wyjściu z zakrętu
                let dip = -0.25 * amp * (std::f32::consts::PI * u).sin();
                let climb = amp * cosine_ramp(0.25, 0.95, u);
                dip + climb
            }
            TrackPreset::ChicaneS => {
                // Płynne wzniesienie w sekcji szykany
                amp * cosine_ramp(0.15, 0.85, u)
            }
            TrackPreset::Hairpin => {
                // Łagodny profil wysokości dla nawrotu (lekka niecka w szczycie łuku)
                0.6 * amp * (1.0 - (std::f32::consts::PI * u).sin())
            }
        }
    }

    /// Wyznacza punkt 3D na przekroju `section_idx` dla parametru `alpha` z przedziału [0, 1]
    /// (0.0 = lewa krawędź, 0.5 = środek toru, 1.0 = prawa krawędź).
    /// Parametr `height_offset` unosi punkt lekko nad asfalt, aby rysowane linie były dobrze widoczne.
    pub fn point_at(&self, section_idx: usize, alpha: f32, height_offset: f32) -> Vec3 {
        let sec = &self.sections[section_idx];
        let pos = sec.left_edge.lerp(sec.right_edge, alpha.clamp(0.0, 1.0));
        pos + sec.normal * height_offset
    }

    /// Buduje siatkę 3D (`CpuMesh`) nawierzchni asfaltowej zakrętu
    pub fn build_road_cpu_mesh(&self) -> CpuMesh {
        let n = self.sections.len();
        let mut positions = Vec::with_capacity(n * 2);
        let mut normals = Vec::with_capacity(n * 2);
        let mut colors = Vec::with_capacity(n * 2);
        let mut indices = Vec::with_capacity((n - 1) * 6);

        for (i, sec) in self.sections.iter().enumerate() {
            positions.push(to_td_vec3(sec.left_edge));
            positions.push(to_td_vec3(sec.right_edge));

            let n_td = to_td_vec3(sec.normal);
            normals.push(n_td);
            normals.push(n_td);

            // Delikatne pasy poprzeczne na asfalcie ułatwiające postrzeganie głębi i skali
            let stripe = if (i / 2) % 2 == 0 {
                Srgba::new(52, 56, 63, 255)
            } else {
                Srgba::new(45, 49, 56, 255)
            };
            colors.push(stripe);
            colors.push(stripe);

            if i + 1 < n {
                let base = (i * 2) as u32;
                indices.extend_from_slice(&[
                    base,
                    base + 1,
                    base + 2,
                    base + 1,
                    base + 3,
                    base + 2,
                ]);
            }
        }

        CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        }
    }

    /// Buduje siatkę 3D (`CpuMesh`) biało-czerwonych krawężników (tarek) po obu stronach toru
    pub fn build_curbs_cpu_mesh(&self) -> CpuMesh {
        let n = self.sections.len();
        let curb_width = 0.75_f32;
        let curb_height = 0.03_f32;

        let mut positions = Vec::with_capacity((n - 1) * 8);
        let mut normals = Vec::with_capacity((n - 1) * 8);
        let mut colors = Vec::with_capacity((n - 1) * 8);
        let mut indices = Vec::with_capacity((n - 1) * 12);

        for i in 0..(n - 1) {
            let s0 = &self.sections[i];
            let s1 = &self.sections[i + 1];

            let color = if (i / 2) % 2 == 0 {
                Srgba::new(215, 45, 45, 255) // Czerwony
            } else {
                Srgba::new(235, 235, 235, 255) // Biały
            };

            // Lewy krawężnik (rozszerza się wzdłuż lokalnego wektora poprzecznego `lateral`)
            let l0_in = s0.left_edge + s0.normal * curb_height;
            let l0_out = s0.left_edge - s0.lateral * curb_width + s0.normal * curb_height;
            let l1_in = s1.left_edge + s1.normal * curb_height;
            let l1_out = s1.left_edge - s1.lateral * curb_width + s1.normal * curb_height;

            push_quad(
                &mut positions,
                &mut normals,
                &mut colors,
                &mut indices,
                l0_out,
                l0_in,
                l1_out,
                l1_in,
                s0.normal,
                color,
            );

            // Prawy krawężnik
            let r0_in = s0.right_edge + s0.normal * curb_height;
            let r0_out = s0.right_edge + s0.lateral * curb_width + s0.normal * curb_height;
            let r1_in = s1.right_edge + s1.normal * curb_height;
            let r1_out = s1.right_edge + s1.lateral * curb_width + s1.normal * curb_height;

            push_quad(
                &mut positions,
                &mut normals,
                &mut colors,
                &mut indices,
                r0_in,
                r0_out,
                r1_in,
                r1_out,
                s0.normal,
                color,
            );
        }

        CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        }
    }

    /// Buduje siatkę 3D (`CpuMesh`) poprzecznych żeberek siatki oraz słupków normalnych
    pub fn build_ribs_cpu_mesh(&self, show_normals: bool) -> CpuMesh {
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut colors = Vec::new();
        let mut indices = Vec::new();

        let rib_half_thickness = 0.07_f32;
        let rib_height = 0.03_f32;
        let rib_color = Srgba::new(115, 175, 235, 255);

        for (i, sec) in self.sections.iter().enumerate() {
            if i % 3 == 0 {
                let p_left = sec.left_edge + sec.normal * rib_height;
                let p_right = sec.right_edge + sec.normal * rib_height;
                let offset = sec.tangent * rib_half_thickness;

                push_quad(
                    &mut positions,
                    &mut normals,
                    &mut colors,
                    &mut indices,
                    p_left - offset,
                    p_right - offset,
                    p_left + offset,
                    p_right + offset,
                    sec.normal,
                    rib_color,
                );
            }

            if show_normals && i % 6 == 0 {
                let base = sec.center + sec.normal * 0.05;
                let tip = sec.center + sec.normal * 2.8;
                let w = 0.08_f32;
                let side = sec.lateral * w;
                let normal_color = Srgba::new(255, 195, 50, 255);

                push_quad(
                    &mut positions,
                    &mut normals,
                    &mut colors,
                    &mut indices,
                    base - side,
                    base + side,
                    tip - side,
                    tip + side,
                    sec.tangent,
                    normal_color,
                );
            }
        }

        CpuMesh {
            positions: Positions::F32(positions),
            normals: Some(normals),
            colors: Some(colors),
            indices: Indices::U32(indices),
            ..Default::default()
        }
    }
}

/// Gładka funkcja przejścia kosinusoidalnego (0 -> 1) między edge0 a edge1 (ciągłe pochodne na krańcach)
fn cosine_ramp(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    0.5 * (1.0 - (std::f32::consts::PI * t).cos())
}

/// Dodatkowe wygładzanie tablicy wartości filtrem uśredniającym [1/4, 1/2, 1/4]
fn smooth_profile(values: &mut [f32], passes: usize) {
    let n = values.len();
    if n < 3 {
        return;
    }
    let mut temp = values.to_vec();
    for _ in 0..passes {
        for i in 1..(n - 1) {
            temp[i] = 0.25 * values[i - 1] + 0.50 * values[i] + 0.25 * values[i + 1];
        }
        temp[0] = 0.0;
        temp[n - 1] = 0.0;
        values.copy_from_slice(&temp);
    }
}

/// Pomocnicza konwersja wektora z `glam::Vec3` na `three_d::Vec3`
pub fn to_td_vec3(v: Vec3) -> three_d::Vec3 {
    three_d::vec3(v.x, v.y, v.z)
}

/// Pomocnicza funkcja dodająca czworokąt (2 trójkąty) do buforów siatki
#[allow(clippy::too_many_arguments)]
pub fn push_quad(
    positions: &mut Vec<three_d::Vec3>,
    normals: &mut Vec<three_d::Vec3>,
    colors: &mut Vec<Srgba>,
    indices: &mut Vec<u32>,
    p00: Vec3,
    p01: Vec3,
    p10: Vec3,
    p11: Vec3,
    normal: Vec3,
    color: Srgba,
) {
    let base = positions.len() as u32;
    positions.push(to_td_vec3(p00));
    positions.push(to_td_vec3(p01));
    positions.push(to_td_vec3(p10));
    positions.push(to_td_vec3(p11));

    let n_td = to_td_vec3(normal);
    normals.extend_from_slice(&[n_td, n_td, n_td, n_td]);
    colors.extend_from_slice(&[color, color, color, color]);

    indices.extend_from_slice(&[base, base + 1, base + 2, base + 1, base + 3, base + 2]);
}
