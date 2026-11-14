use racing_line_3d::optimizer::{safe_alpha_bounds, ComparisonSuite};
use racing_line_3d::physics::VehicleParams;
use racing_line_3d::track::{Track3D, TrackParams, TrackPreset};

#[test]
fn trajectory_stays_within_track_boundaries_for_all_presets() {
    for preset in [
        TrackPreset::Parabolica,
        TrackPreset::ChicaneS,
        TrackPreset::Hairpin,
    ] {
        let params = TrackParams {
            preset,
            width: 12.0,
            safety_margin: 0.8,
            ..Default::default()
        };
        let track = Track3D::generate(params);
        let vehicle = VehicleParams::default();
        let suite = ComparisonSuite::compute(&track, &vehicle);

        let (alpha_min, alpha_max) = safe_alpha_bounds(&track);
        let eps = 1e-4;

        for (i, node) in suite.racing_line.nodes.iter().enumerate() {
            // Sprawdzenie współczynnika alpha wewnątrz marginesu bezpieczeństwa
            assert!(
                node.alpha >= alpha_min - eps && node.alpha <= alpha_max + eps,
                "Punkt {} dla presetu {:?} wyszedł poza margines bezpieczeństwa: alpha = {}, dozwolone [{}, {}]",
                i,
                preset,
                node.alpha,
                alpha_min,
                alpha_max
            );

            // Sprawdzenie odległości 3D od środka przekroju
            let sec = &track.sections[i];
            let lateral_dist = (node.position - sec.center).dot(sec.lateral).abs();
            let max_allowed_dist = 0.5 * track.params.width - track.params.safety_margin + 0.05;
            assert!(
                lateral_dist <= max_allowed_dist,
                "Odległość poprzeczna {} m przekracza dozwoloną połowę szerokości toru {} m",
                lateral_dist,
                max_allowed_dist
            );
        }
    }
}

#[test]
fn velocity_profile_respects_grip_and_friction_circle_limits() {
    let track = Track3D::generate(TrackParams::default());
    let vehicle = VehicleParams::default();
    let suite = ComparisonSuite::compute(&track, &vehicle);

    let eps = 1e-2;
    for (i, node) in suite.racing_line.nodes.iter().enumerate() {
        // 1. Prędkość nie może przekraczać lokalnej prędkości granicznej w zakręcie v_lat_limit
        assert!(
            node.speed <= node.v_lat_limit + eps,
            "Węzeł {}: prędkość {:.3} m/s przekracza limit boczny {:.3} m/s",
            i,
            node.speed,
            node.v_lat_limit
        );

        // 2. Przyspieszenie boczne nie może przekraczać a_max wynikającego z tarcia i bankingu
        assert!(
            node.a_lat <= node.a_max + eps,
            "Węzeł {}: przyspieszenie boczne {:.3} m/s² przekracza a_max {:.3} m/s²",
            i,
            node.a_lat,
            node.a_max
        );

        // 3. Przyspieszenie wzdłużne musi mieścić się w limitach hamowania i przyspieszania silnika
        assert!(
            node.a_long <= vehicle.a_acc_max + eps && node.a_long >= -vehicle.a_brake_max - eps,
            "Węzeł {}: przyspieszenie wzdłużne {:.3} m/s² poza zakresem [-{}, +{}]",
            i,
            node.a_long,
            vehicle.a_brake_max,
            vehicle.a_acc_max
        );
    }
}

#[test]
fn optimized_racing_line_is_faster_than_centerline() {
    for preset in [
        TrackPreset::Parabolica,
        TrackPreset::ChicaneS,
        TrackPreset::Hairpin,
    ] {
        let track = Track3D::generate(TrackParams {
            preset,
            ..Default::default()
        });
        let vehicle = VehicleParams::default();
        let suite = ComparisonSuite::compute(&track, &vehicle);

        assert!(
            suite.racing_line.total_time_sec < suite.centerline.total_time_sec,
            "Linia wyścigowa ({:.3} s) powinna być szybsza niż linia środkowa ({:.3} s) dla {:?}",
            suite.racing_line.total_time_sec,
            suite.centerline.total_time_sec,
            preset
        );

        assert!(
            suite.racing_line.total_time_sec <= suite.geometric_line.total_time_sec + 1e-4,
            "Linia wyścigowa ({:.3} s) nie powinna być wolniejsza niż linia geometryczna ({:.3} s) dla {:?}",
            suite.racing_line.total_time_sec,
            suite.geometric_line.total_time_sec,
            preset
        );
    }
}

#[test]
fn banking_angle_increases_grip_and_reduces_lap_time() {
    let vehicle = VehicleParams::default();

    let flat_track = Track3D::generate(TrackParams {
        banking_deg: 0.0,
        ..Default::default()
    });
    let banked_track = Track3D::generate(TrackParams {
        banking_deg: 12.0,
        ..Default::default()
    });

    let flat_suite = ComparisonSuite::compute(&flat_track, &vehicle);
    let banked_suite = ComparisonSuite::compute(&banked_track, &vehicle);

    assert!(
        banked_suite.racing_line.total_time_sec < flat_suite.racing_line.total_time_sec,
        "Tor z pochyleniem poprzecznym 12° ({:.3} s) powinien dawać krótszy czas przejazdu niż płaski tor ({:.3} s)",
        banked_suite.racing_line.total_time_sec,
        flat_suite.racing_line.total_time_sec
    );
}

#[test]
fn hairpin_legs_do_not_overlap_and_banking_is_orthogonal() {
    let track = Track3D::generate(TrackParams {
        preset: TrackPreset::Hairpin,
        width: 20.0,
        banking_deg: 15.0,
        ..Default::default()
    });

    // Odległość w płaszczyźnie XZ między początkiem prostej wejściowej a końcem prostej wyjściowej
    // musi być znacznie większa niż maksymalna szerokość toru W = 20 m (brak nakładania się nóg nawrotu)
    let first = &track.sections[0];
    let last = track.sections.last().unwrap();
    let leg_separation = (first.center - last.center).length();
    assert!(
        leg_separation > 35.0,
        "Prosta wejściowa i wyjściowa nawrotu są zbyt blisko siebie: {:.2} m",
        leg_separation
    );

    // Sprawdzenie ortogonalności i symetrii przekrojów przy maksymalnym bankingu 15°
    for (i, sec) in track.sections.iter().enumerate() {
        let midpoint = 0.5 * (sec.left_edge + sec.right_edge);
        assert!(
            midpoint.distance(sec.center) < 1e-4,
            "Przekrój {}: środek obrotu bankingu przesunął się względem centerline",
            i
        );
        assert!(
            sec.lateral.dot(sec.tangent).abs() < 1e-4,
            "Przekrój {}: wektor poprzeczny nie jest prostopadły do osi stycznej",
            i
        );
    }
}
