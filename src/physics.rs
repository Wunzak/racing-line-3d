use crate::optimizer::{DrivingPhase, TrajectoryNode, TrajectoryResult};
use crate::track::Track3D;
use glam::Vec3;

/// Parametry modelu fizycznego pojazdu (punktowy model z kołem tarcia)
#[derive(Debug, Clone)]
pub struct VehicleParams {
    /// Współczynnik tarcia opon mi (np. 1.4 dla auta wyścigowego)
    pub mu: f32,
    /// Maksymalne przyspieszenie wzdłużne silnika [m/s^2] (np. 8.0 m/s^2)
    pub a_acc_max: f32,
    /// Maksymalne opóźnienie hamowania [m/s^2] (np. 14.0 m/s^2)
    pub a_brake_max: f32,
    /// Przyspieszenie ziemskie g [m/s^2]
    pub g: f32,
    /// Maksymalna prędkość na prostej dojazdowej [m/s]
    pub v_top_speed: f32,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            mu: 1.4,
            a_acc_max: 8.0,
            a_brake_max: 14.0,
            g: 9.81,
            v_top_speed: 78.0, // ok. 280 km/h
        }
    }
}

impl VehicleParams {
    /// Oblicza maksymalne wypadkowe przyspieszenie poziome a_max [m/s^2]
    /// z uwzględnieniem współczynnika tarcia `mu` oraz kąta pochylenia zakrętu `beta` [rad]:
    /// a_max = g * (mu * cos(beta) + sin(beta)) / (cos(beta) - mu * sin(beta))
    pub fn max_lateral_accel(&self, beta_rad: f32) -> f32 {
        let cos_b = beta_rad.cos();
        let sin_b = beta_rad.sin();
        let denom = (cos_b - self.mu * sin_b).max(0.15);
        self.g * (self.mu * cos_b + sin_b) / denom
    }
}

/// Oblicza dyskretną krzywiznę 3D [1/m] w każdym punkcie ścieżki (krzywizna Mengera z 3 kolejnych punktów)
pub fn compute_curvatures(points: &[Vec3]) -> Vec<f32> {
    let n = points.len();
    if n < 3 {
        return vec![0.0; n];
    }

    let mut curvatures = vec![0.0_f32; n];
    for i in 1..(n - 1) {
        let p_prev = points[i - 1];
        let p_curr = points[i];
        let p_next = points[i + 1];

        let v1 = p_curr - p_prev;
        let v2 = p_next - p_curr;
        let v3 = p_next - p_prev;

        let a = v1.length();
        let b = v2.length();
        let c = v3.length();

        let denom = a * b * c;
        if denom > 1e-6 {
            let area2 = v1.cross(v2).length();
            curvatures[i] = (2.0 * area2 / denom).min(0.25);
        }
    }

    // Przepisanie wartości brzegowych z sąsiednich węzłów
    curvatures[0] = curvatures[1];
    curvatures[n - 1] = curvatures[n - 2];

    curvatures
}

/// Wyznacza pełny profil prędkości (przebieg Forward-Backward z kołem tarcia)
/// oraz czas przejazdu dla zadanej tablicy współczynników poprzecznych `alphas`.
pub fn evaluate_trajectory(
    track: &Track3D,
    alphas: &[f32],
    vehicle: &VehicleParams,
) -> TrajectoryResult {
    let n = track.sections.len();
    assert_eq!(
        n,
        alphas.len(),
        "Liczba współczynników alpha musi odpowiadać liczbie przekrojów toru"
    );

    // Krok 1: Wyznaczenie punktów 3D na powierzchni toru dla każdego przekroju
    let mut points = Vec::with_capacity(n);
    for (i, &alpha) in alphas.iter().enumerate() {
        points.push(track.point_at(i, alpha, 0.08));
    }

    // Odległości między kolejnymi punktami trajektorii
    let mut ds = vec![0.0_f32; n.saturating_sub(1)];
    let mut distances = vec![0.0_f32; n];
    for i in 0..(n - 1) {
        let seg_len = points[i + 1].distance(points[i]).max(1e-3);
        ds[i] = seg_len;
        distances[i + 1] = distances[i] + seg_len;
    }

    // Krok 2: Wyliczenie krzywizny i maksymalnej prędkości bocznej v_lat = sqrt(a_max * R)
    let curvatures = compute_curvatures(&points);
    let mut a_max_per_node = Vec::with_capacity(n);
    let mut v_lat_limit = Vec::with_capacity(n);

    for i in 0..n {
        let beta = track.sections[i].banking_rad;
        let a_max = vehicle.max_lateral_accel(beta);
        a_max_per_node.push(a_max);

        let kappa = curvatures[i];
        let v_lat = if kappa > 1e-5 {
            (a_max / kappa).sqrt().min(vehicle.v_top_speed)
        } else {
            vehicle.v_top_speed
        };
        v_lat_limit.push(v_lat);
    }

    // Początkowy profil prędkości jest ograniczony od góry przez prędkość graniczną w zakręcie
    let mut speeds = v_lat_limit.clone();

    // Krok 3: Przebieg wsteczny (Backward Pass) – ograniczenie hamowaniem przed zakrętem
    for i in (0..(n - 1)).rev() {
        let v_next = speeds[i + 1];
        let a_max = a_max_per_node[i];
        // Aktualne przyspieszenie boczne wynikające z krzywizny
        let a_lat_est = (curvatures[i] * v_next * v_next).min(a_max);
        // Zapas przyczepności wzdłużnej wynikający z koła tarcia opon
        let ratio = (a_lat_est / a_max).clamp(0.0, 1.0);
        let tire_long_avail = a_max * (1.0 - ratio * ratio).max(0.0).sqrt();

        // Uwzględnienie maksymalnego opóźnienia hamulców
        let a_brake = vehicle.a_brake_max.min(tire_long_avail);
        let v_max_from_braking = (v_next * v_next + 2.0 * a_brake * ds[i]).sqrt();
        if v_max_from_braking < speeds[i] {
            speeds[i] = v_max_from_braking;
        }
    }

    // Krok 4: Przebieg w przód (Forward Pass) – ograniczenie przyspieszaniem silnika na wyjściu
    for i in 0..(n - 1) {
        let v_curr = speeds[i];
        let a_max = a_max_per_node[i];
        let a_lat_est = (curvatures[i] * v_curr * v_curr).min(a_max);
        let ratio = (a_lat_est / a_max).clamp(0.0, 1.0);
        let tire_long_avail = a_max * (1.0 - ratio * ratio).max(0.0).sqrt();

        // Uwzględnienie maksymalnego przyspieszenia silnika
        let a_acc = vehicle.a_acc_max.min(tire_long_avail);
        let v_max_from_acc = (v_curr * v_curr + 2.0 * a_acc * ds[i]).sqrt();
        if v_max_from_acc < speeds[i + 1] {
            speeds[i + 1] = v_max_from_acc;
        }
    }

    // Krok 5: Wyliczenie przyspieszeń wzdłużnych, bocznych, fazy jazdy oraz całkowitego czasu T
    let mut nodes = Vec::with_capacity(n);
    let mut total_time_sec = 0.0_f32;

    for i in 0..n {
        let v = speeds[i];
        let a_lat = curvatures[i] * v * v;

        let a_long = if i + 1 < n {
            let v_next = speeds[i + 1];
            (v_next * v_next - v * v) / (2.0 * ds[i])
        } else if i > 0 {
            let v_prev = speeds[i - 1];
            (v * v - v_prev * v_prev) / (2.0 * ds[i - 1])
        } else {
            0.0
        };

        if i + 1 < n {
            let v_avg = 0.5 * (speeds[i] + speeds[i + 1]).max(0.5);
            total_time_sec += ds[i] / v_avg;
        }

        // Klasyfikacja fazy jazdy do kolorowania trajektorii:
        // - Czerwony: wyraźne hamowanie przed zakrętem
        // - Żółty: szczyt zakrętu (apex) / jazda blisko limitu bocznego
        // - Zielony: przyspieszanie lub jazda z pełną prędkością
        let a_max = a_max_per_node[i];
        let phase = if a_long < -1.2 {
            DrivingPhase::Braking
        } else if a_lat > 0.65 * a_max || (a_lat > 0.35 * a_max && a_long.abs() <= 1.2) {
            DrivingPhase::ApexLimit
        } else {
            DrivingPhase::Acceleration
        };

        nodes.push(TrajectoryNode {
            position: points[i],
            alpha: alphas[i],
            distance: distances[i],
            curvature: curvatures[i],
            v_lat_limit: v_lat_limit[i],
            speed: v,
            a_long,
            a_lat,
            a_max,
            phase,
        });
    }

    TrajectoryResult {
        nodes,
        total_time_sec,
        total_distance_m: *distances.last().unwrap_or(&0.0),
    }
}
