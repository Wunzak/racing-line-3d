use crate::physics::{evaluate_trajectory, VehicleParams};
use crate::track::{push_quad, Track3D};
use glam::Vec3;
use three_d::{CpuMesh, Indices, Positions, Srgba};

/// Faza jazdy w danym punkcie trajektorii (używana do kolorowania linii w oknie 3D)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrivingPhase {
    /// Hamowanie przed wejściem w zakręt (kolor czerwony)
    Braking,
    /// Szczyt zakrętu (apex) / jazda na limicie przyczepności bocznej (kolor żółty)
    ApexLimit,
    /// Przyspieszanie na wyjściu z zakrętu (kolor zielony)
    Acceleration,
}

/// Dane pojedynczego punktu na wyznaczonej trajektorii
#[derive(Debug, Clone)]
pub struct TrajectoryNode {
    /// Pozycja 3D punktu na torze
    pub position: Vec3,
    /// Względna pozycja poprzeczna alpha w [0, 1] (0 = lewa krawędź, 1 = prawa krawędź)
    pub alpha: f32,
    /// Dystans wzdłuż trajektorii od początku odcinka [m]
    pub distance: f32,
    /// Lokalna krzywizna [1/m]
    pub curvature: f32,
    /// Maksymalna prędkość boczna wynikająca z promienia i tarcia [m/s]
    pub v_lat_limit: f32,
    /// Rzeczywista prędkość przejazdu po przebiegu forward-backward [m/s]
    pub speed: f32,
    /// Przyspieszenie wzdłużne [m/s^2] (ujemne = hamowanie, dodatnie = przyspieszanie)
    pub a_long: f32,
    /// Przyspieszenie boczne [m/s^2]
    pub a_lat: f32,
    /// Lokalny limit całkowitego przyspieszenia poziomego z koła tarcia [m/s^2]
    pub a_max: f32,
    /// Dominująca faza jazdy
    pub phase: DrivingPhase,
}

/// Wynik optymalizacji i symulacji pojedynczej trajektorii
#[derive(Debug, Clone, Default)]
pub struct TrajectoryResult {
    pub nodes: Vec<TrajectoryNode>,
    pub total_time_sec: f32,
    pub total_distance_m: f32,
}

/// Zestaw 3 porównywanych trajektorii: środkowej, geometrycznej i wyścigowej (Time-Optimal)
#[derive(Debug, Clone)]
pub struct ComparisonSuite {
    /// Linia środkowa toru (alpha = 0.5)
    pub centerline: TrajectoryResult,
    /// Linia czysto geometryczna (minimalnej krzywizny)
    pub geometric_line: TrajectoryResult,
    /// Linia wyścigowa (minimalizująca całkowity czas przejazdu T)
    pub racing_line: TrajectoryResult,
}

impl ComparisonSuite {
    /// Oblicza wszystkie 3 trajektorie dla zadanego toru 3D i parametrów pojazdu
    pub fn compute(track: &Track3D, vehicle: &VehicleParams) -> Self {
        let n = track.sections.len();
        let (alpha_min, alpha_max) = safe_alpha_bounds(track);

        // 1. Linia środkowa (stałe alpha = 0.5)
        let center_alphas = vec![0.5_f32; n];
        let centerline = evaluate_trajectory(track, &center_alphas, vehicle);

        // 2. Linia geometryczna (relaksacja minimalnej krzywizny)
        let geom_alphas = optimize_geometric_alphas(track, alpha_min, alpha_max, 220);
        let geometric_line = evaluate_trajectory(track, &geom_alphas, vehicle);

        // 3. Linia wyścigowa (optymalizacja czasu przejazdu z efektem Late Apex)
        let racing_alphas =
            optimize_time_optimal_alphas(track, vehicle, &geom_alphas, alpha_min, alpha_max);
        let racing_line = evaluate_trajectory(track, &racing_alphas, vehicle);

        Self {
            centerline,
            geometric_line,
            racing_line,
        }
    }
}

/// Zwraca dopuszczalny przedział [alpha_min, alpha_max] uwzględniający margines bezpieczeństwa od krawędzi
pub fn safe_alpha_bounds(track: &Track3D) -> (f32, f32) {
    let margin_ratio = (track.params.safety_margin / track.params.width).clamp(0.02, 0.40);
    (margin_ratio, 1.0 - margin_ratio)
}

/// Wyznacza linię geometryczną (minimalnej krzywizny) metodą iteracyjnej relaksacji biharmonicznej
pub fn optimize_geometric_alphas(
    track: &Track3D,
    alpha_min: f32,
    alpha_max: f32,
    iterations: usize,
) -> Vec<f32> {
    let n = track.sections.len();
    let mut alphas = vec![0.5_f32; n];
    let mut next_alphas = alphas.clone();

    // Wstępne wektory poprzeczne przekrojów
    let span_vecs: Vec<Vec3> = track
        .sections
        .iter()
        .map(|s| s.right_edge - s.left_edge)
        .collect();
    let span_len_sq: Vec<f32> = span_vecs.iter().map(|v| v.length_squared().max(1e-4)).collect();

    for _iter in 0..iterations {
        let pts: Vec<Vec3> = (0..n)
            .map(|i| track.sections[i].left_edge + span_vecs[i] * alphas[i])
            .collect();

        for i in 1..(n - 1) {
            // Złożenie relaksacji cięciwowej (Laplasjan) z korekcją 4. rzędu (minimalizacja krzywizny)
            let midpoint = 0.5 * (pts[i - 1] + pts[i + 1]);
            let target = if i >= 2 && i + 2 < n {
                let biharmonic = (4.0 * (pts[i - 1] + pts[i + 1]) - (pts[i - 2] + pts[i + 2])) / 6.0;
                midpoint.lerp(biharmonic, 0.35)
            } else {
                midpoint
            };

            let rel = target - track.sections[i].left_edge;
            let proj_alpha = rel.dot(span_vecs[i]) / span_len_sq[i];
            let blended = alphas[i] + 0.65 * (proj_alpha - alphas[i]);
            next_alphas[i] = blended.clamp(alpha_min, alpha_max);
        }

        // Gładkie warunki brzegowe na wjeździe i wyjeździe
        next_alphas[0] = next_alphas[1];
        next_alphas[n - 1] = next_alphas[n - 2];

        alphas.copy_from_slice(&next_alphas);
    }

    alphas
}

/// Optymalizuje linię wyścigową pod kątem minimalnego czasu przejazdu T = suma(ds_i / v_i).
/// Uwzględnia asymetrię między silnym hamowaniem a słabszym przyspieszaniem (wyznacza późny wierzchołek – Late Apex).
pub fn optimize_time_optimal_alphas(
    track: &Track3D,
    vehicle: &VehicleParams,
    initial_alphas: &[f32],
    alpha_min: f32,
    alpha_max: f32,
) -> Vec<f32> {
    let n = track.sections.len();
    let mut best_alphas = initial_alphas.to_vec();
    let mut best_eval = evaluate_trajectory(track, &best_alphas, vehicle);

    let span_vecs: Vec<Vec3> = track
        .sections
        .iter()
        .map(|s| s.right_edge - s.left_edge)
        .collect();
    let span_len_sq: Vec<f32> = span_vecs.iter().map(|v| v.length_squared().max(1e-4)).collect();

    // Etap A: Przeszeszukanie asymetrycznego przesunięcia wierzchołka zakrętu (Late Apex)
    // Im mniejszy stosunek a_acc / a_brake, tym bardziej opłaca się opóźnić wierzchołek i wyprostować wyjście.
    let acc_brake_ratio = (vehicle.a_acc_max / vehicle.a_brake_max.max(1.0)).clamp(0.2, 1.5);
    for &bias_strength in &[0.06_f32, 0.12, 0.18, 0.24, 0.30] {
        let mut candidate = initial_alphas.to_vec();
        let mut next_cand = candidate.clone();
        let asym = bias_strength * (1.15 - 0.5 * acc_brake_ratio).clamp(0.2, 1.0);

        for _ in 0..90 {
            let pts: Vec<Vec3> = (0..n)
                .map(|i| track.sections[i].left_edge + span_vecs[i] * candidate[i])
                .collect();

            for i in 2..(n - 2) {
                // Większa waga punktu następnego (w stronę wyjścia z zakrętu) wymusza wcześniejsze
                // wyprostowanie toru jazdy w strefie przyspieszania kosztem głębszego hamowania.
                let w_prev = 0.5 - asym;
                let w_next = 0.5 + asym;
                let target = w_prev * pts[i - 1] + w_next * pts[i + 1];

                let rel = target - track.sections[i].left_edge;
                let proj_alpha = rel.dot(span_vecs[i]) / span_len_sq[i];
                next_cand[i] = (candidate[i] + 0.45 * (proj_alpha - candidate[i]))
                    .clamp(alpha_min, alpha_max);
            }
            next_cand[0] = next_cand[2];
            next_cand[1] = next_cand[2];
            next_cand[n - 1] = next_cand[n - 3];
            next_cand[n - 2] = next_cand[n - 3];
            candidate.copy_from_slice(&next_cand);
        }

        let eval = evaluate_trajectory(track, &candidate, vehicle);
        if eval.total_time_sec < best_eval.total_time_sec {
            best_eval = eval;
            best_alphas = candidate;
        }
    }

    // Etap B: Bezpośrednia optymalizacja gradientowa / lokalna z gładkimi funkcjami bazowymi Gaussa
    // Modyfikujemy profil alpha gładkimi garbikami (bump functions) i zachowujemy zmiany skracające czas T.
    let num_centers = 18;
    let sigma = (n as f32) / 14.0;

    for &step_size in &[0.045_f32, 0.022, 0.010, 0.004] {
        for c_idx in 1..num_centers {
            let center = (c_idx as f32 / num_centers as f32) * (n as f32);

            for &dir in &[-1.0_f32, 1.0_f32] {
                let mut trial_alphas = best_alphas.clone();
                for (i, val) in trial_alphas.iter_mut().enumerate() {
                    let dist = (i as f32 - center) / sigma;
                    let bump = (-0.5 * dist * dist).exp();
                    *val = (*val + dir * step_size * bump).clamp(alpha_min, alpha_max);
                }
                smooth_alphas_in_place(&mut trial_alphas, alpha_min, alpha_max, 2);

                let trial_eval = evaluate_trajectory(track, &trial_alphas, vehicle);
                if trial_eval.total_time_sec < best_eval.total_time_sec {
                    best_eval = trial_eval;
                    best_alphas = trial_alphas;
                }
            }
        }
    }

    best_alphas
}

/// Wygładza wektor współczynników alpha filtrem uśredniającym, zachowując ograniczenia brzegowe
fn smooth_alphas_in_place(alphas: &mut [f32], alpha_min: f32, alpha_max: f32, passes: usize) {
    let n = alphas.len();
    if n < 3 {
        return;
    }
    let mut temp = alphas.to_vec();
    for _ in 0..passes {
        for i in 1..(n - 1) {
            temp[i] = (0.25 * alphas[i - 1] + 0.5 * alphas[i] + 0.25 * alphas[i + 1])
                .clamp(alpha_min, alpha_max);
        }
        temp[0] = temp[1];
        temp[n - 1] = temp[n - 2];
        alphas.copy_from_slice(&temp);
    }
}

/// Buduje siatkę 3D (`CpuMesh`) głównej linii wyścigowej pokolorowanej według fazy jazdy:
/// - Czerwony = hamowanie,
/// - Żółty = szczyt zakrętu (apex) / limit przyczepności,
/// - Zielony = przyspieszanie.
pub fn build_phase_colored_ribbon_mesh(
    track: &Track3D,
    trajectory: &TrajectoryResult,
    half_width: f32,
    height_offset: f32,
) -> CpuMesh {
    let n = trajectory.nodes.len();
    let mut positions = Vec::with_capacity(n * 2);
    let mut normals = Vec::with_capacity(n * 2);
    let mut colors = Vec::with_capacity(n * 2);
    let mut indices = Vec::with_capacity((n.saturating_sub(1)) * 6);

    for i in 0..(n.saturating_sub(1)) {
        let node0 = &trajectory.nodes[i];
        let node1 = &trajectory.nodes[i + 1];
        let sec0 = &track.sections[i];
        let sec1 = &track.sections[i + 1];

        let p0 = track.point_at(i, node0.alpha, height_offset);
        let p1 = track.point_at(i + 1, node1.alpha, height_offset);

        let dir0 = (p1 - p0).normalize_or_zero();
        let side0 = dir0.cross(sec0.normal).normalize_or_zero() * half_width;
        let side1 = dir0.cross(sec1.normal).normalize_or_zero() * half_width;

        let color = phase_color(node0);

        push_quad(
            &mut positions,
            &mut normals,
            &mut colors,
            &mut indices,
            p0 - side0,
            p0 + side0,
            p1 - side1,
            p1 + side1,
            sec0.normal,
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

/// Buduje jednolitą kolorystycznie wstęgę 3D (`CpuMesh`) dla trajektorii pomocniczych (np. środkowej lub geometrycznej)
pub fn build_solid_ribbon_mesh(
    track: &Track3D,
    trajectory: &TrajectoryResult,
    half_width: f32,
    height_offset: f32,
    color: Srgba,
    dashed: bool,
) -> CpuMesh {
    let n = trajectory.nodes.len();
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut colors = Vec::new();
    let mut indices = Vec::new();

    for i in 0..(n.saturating_sub(1)) {
        if dashed && (i % 2 == 1) {
            continue;
        }
        let node0 = &trajectory.nodes[i];
        let node1 = &trajectory.nodes[i + 1];
        let sec0 = &track.sections[i];
        let sec1 = &track.sections[i + 1];

        let p0 = track.point_at(i, node0.alpha, height_offset);
        let p1 = track.point_at(i + 1, node1.alpha, height_offset);

        let dir = (p1 - p0).normalize_or_zero();
        let side0 = dir.cross(sec0.normal).normalize_or_zero() * half_width;
        let side1 = dir.cross(sec1.normal).normalize_or_zero() * half_width;

        push_quad(
            &mut positions,
            &mut normals,
            &mut colors,
            &mut indices,
            p0 - side0,
            p0 + side0,
            p1 - side1,
            p1 + side1,
            sec0.normal,
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

/// Dobiera wyrazisty kolor RGB odpowiadający fazie jazdy w danym punkcie
fn phase_color(node: &TrajectoryNode) -> Srgba {
    match node.phase {
        DrivingPhase::Braking => Srgba::new(235, 55, 50, 255),      // Czerwony (hamowanie)
        DrivingPhase::ApexLimit => Srgba::new(250, 210, 45, 255),   // Żółty (szczyt zakrętu / limit)
        DrivingPhase::Acceleration => Srgba::new(45, 215, 95, 255), // Zielony (przyspieszanie)
    }
}
