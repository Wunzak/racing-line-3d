use crate::optimizer::ComparisonSuite;
use crate::physics::VehicleParams;
use crate::track::{TrackParams, TrackPreset};
use egui_plot::{Legend, Line, Plot, PlotPoints, Points};
use three_d::egui;

/// Stan interfejsu użytkownika sterujący geometrią toru, fizyką pojazdu i warstwami wizualizacji
pub struct UiState {
    pub track_params: TrackParams,
    pub vehicle_params: VehicleParams,
    pub show_curbs: bool,
    pub show_ribs: bool,
    pub show_normals: bool,
    pub show_racing_line: bool,
    pub show_geometric_line: bool,
    pub show_centerline: bool,
    pub show_gg_diagram: bool,
    pub needs_recompute: bool,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            track_params: TrackParams::default(),
            vehicle_params: VehicleParams::default(),
            show_curbs: true,
            show_ribs: true,
            show_normals: false,
            show_racing_line: true,
            show_geometric_line: true,
            show_centerline: true,
            show_gg_diagram: true,
            needs_recompute: true,
        }
    }
}

impl UiState {
    /// Rysuje boczny panel sterowania oraz dolne/boczne wykresy telemetryczne w `egui`.
    /// Zwraca szerokość lewego panelu w pikselach logicznych.
    pub fn draw_ui(&mut self, ctx: &egui::Context, suite: &ComparisonSuite) -> f32 {
        let mut changed = false;

        let panel_res = egui::SidePanel::left("control_panel")
            .default_width(395.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("Optymalizacja Linii Wyścigowej 3D");
                    ui.separator();

                    // --- Sekcja wyników czasowych (Telemetria i porównanie 3 linii) ---
                    let t_opt = suite.racing_line.total_time_sec;
                    let t_geom = suite.geometric_line.total_time_sec;
                    let t_cent = suite.centerline.total_time_sec;

                    let gain_vs_center = (t_cent - t_opt).max(0.0);
                    let gain_vs_geom = (t_geom - t_opt).max(0.0);

                    let min_speed_kmh = suite
                        .racing_line
                        .nodes
                        .iter()
                        .map(|n| n.speed * 3.6)
                        .fold(f32::INFINITY, f32::min);
                    let max_g_lat = suite
                        .racing_line
                        .nodes
                        .iter()
                        .map(|n| n.a_lat / self.vehicle_params.g)
                        .fold(0.0_f32, f32::max);

                    egui::Frame::group(ui.style()).show(ui, |ui| {
                        ui.label(
                            egui::RichText::new(format!(
                                "Czas przejazdu (Linia wyścigowa): {:.3} s",
                                t_opt
                            ))
                            .strong()
                            .size(16.0)
                            .color(egui::Color32::from_rgb(80, 235, 120)),
                        );
                        ui.label(format!(
                            "Linia geometryczna: {:.3} s  (ΔT = -{:.3} s)",
                            t_geom, gain_vs_geom
                        ));
                        ui.label(format!(
                            "Linia środkowa:     {:.3} s  (ΔT = -{:.3} s)",
                            t_cent, gain_vs_center
                        ));
                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.label(format!("Min. prędkość (Apex): {:.1} km/h", min_speed_kmh));
                            ui.separator();
                            ui.label(format!("Max. przeciążenie: {:.2} G", max_g_lat));
                        });
                    });

                    ui.add_space(6.0);

                    // --- Zwijana zakładka opisująca model fizyczny i parametry pojazdu ---
                    egui::CollapsingHeader::new("Model fizyczny i parametry")
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.small(egui::RichText::new("Domyślne parametry pojazdu (Auto klasy GT / Formula):").strong());
                            ui.small(format!(
                                "• Współczynnik przyczepności opon: μ = {:.2} (domyślnie 1.40)\n\
                                 • Maks. przyspieszenie silnika: a_acc = {:.1} m/s² (~0.82 G)\n\
                                 • Maks. opóźnienie hamowania: a_brake = {:.1} m/s² (~1.43 G)\n\
                                 • Prędkość maksymalna na prostej: v_max = {:.0} m/s ({:.0} km/h)\n\
                                 • Przyspieszenie ziemskie: g = {:.2} m/s²",
                                self.vehicle_params.mu,
                                self.vehicle_params.a_acc_max,
                                self.vehicle_params.a_brake_max,
                                self.vehicle_params.v_top_speed,
                                self.vehicle_params.v_top_speed * 3.6,
                                self.vehicle_params.g
                            ));
                            ui.separator();
                            ui.small(egui::RichText::new("1. Koło tarcia (Friction Circle):").strong());
                            ui.small(
                                "Opona ma skończony budżet przyczepności: wypadkowe przyspieszenie poziome \
                                 sqrt(a_lat² + a_long²) nie może przekroczyć a_max. Jeśli auto skręca na pełnym \
                                 limicie bocznym, nie ma już zapasu na hamowanie ani przyspieszanie.",
                            );
                            ui.add_space(4.0);
                            ui.small(egui::RichText::new("2. Wpływ bankingu (β) i przyczepności (μ):").strong());
                            ui.small(
                                "Pochylenie zakrętu do wewnątrz (banking β) sprawia, że składowa grawitacji \
                                 pomaga utrzymać auto w łuku i dociska opony do asfaltu:\n\
                                 a_max = g * (μ cos β + sin β) / (cos β - μ sin β)\n\
                                 Prędkość graniczna w łuku rośnie z promieniem R: v_lat = sqrt(a_max * R).",
                            );
                            ui.add_space(4.0);
                            ui.small(egui::RichText::new("3. Forward-Backward Pass i Late Apex:").strong());
                            ui.small(
                                "• Przebieg wsteczny (Backward) liczy od szczytu zakrętu wstecz, gdzie trzeba zacząć hamować.\n\
                                 • Przebieg w przód (Forward) symuluje otwarcie gazu na wyjściu.\n\
                                 • Ponieważ silnik przyspiesza wolniej (8 m/s²) niż hamulce zwalniają (14 m/s²), \
                                 opłaca się opóźnić wierzchołek zakrętu (Late Apex) – wcześniej wyprostować auto \
                                 i szybciej wyjść na prostą.",
                            );
                        });

                    ui.add_space(6.0);
                    ui.separator();

                    // --- Wybór presetu zakrętu ---
                    ui.heading("1. Kształt i geometria zakrętu");
                    for preset in [
                        TrackPreset::Parabolica,
                        TrackPreset::ChicaneS,
                        TrackPreset::Hairpin,
                    ] {
                        if ui
                            .radio_value(&mut self.track_params.preset, preset, preset.label())
                            .changed()
                        {
                            changed = true;
                        }
                    }

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.track_params.width, 8.0..=20.0)
                                .text("Szerokość toru W [m]"),
                        )
                        .changed();

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.track_params.banking_deg, 0.0..=15.0)
                                .text("Pochylenie (banking β) [°]"),
                        )
                        .changed();

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.track_params.elevation_change, 0.0..=8.0)
                                .text("Zmiana wysokości [m]"),
                        )
                        .changed();

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.track_params.safety_margin, 0.3..=2.5)
                                .text("Margines od krawędzi [m]"),
                        )
                        .changed();

                    ui.add_space(6.0);
                    ui.separator();

                    // --- Parametry fizyczne pojazdu ---
                    ui.heading("2. Fizyka pojazdu (Koło tarcia)");

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.vehicle_params.mu, 0.7..=2.2)
                                .text("Współczynnik przyczepności μ"),
                        )
                        .changed();

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.vehicle_params.a_brake_max, 6.0..=22.0)
                                .text("Limit hamowania [m/s²]"),
                        )
                        .changed();

                    changed |= ui
                        .add(
                            egui::Slider::new(&mut self.vehicle_params.a_acc_max, 3.0..=15.0)
                                .text("Limit przyspieszenia [m/s²]"),
                        )
                        .changed();

                    ui.add_space(6.0);
                    ui.separator();

                    // --- Widoczność warstw 3D ---
                    ui.heading("3. Widoczność trajektorii i siatki");
                    ui.checkbox(
                        &mut self.show_racing_line,
                        "Linia wyścigowa (Czerw./Żółty/Ziel. wg fazy)",
                    );
                    ui.checkbox(
                        &mut self.show_geometric_line,
                        "Linia geometryczna (Cyjanowa ciągła)",
                    );
                    ui.checkbox(
                        &mut self.show_centerline,
                        "Linia środkowa (Jasnoszara przerywana)",
                    );
                    ui.checkbox(&mut self.show_ribs, "Poprzeczne żeberka siatki (banking)");
                    ui.checkbox(&mut self.show_curbs, "Biało-czerwone krawężniki (tarki)");
                    ui.checkbox(&mut self.show_normals, "Słupki wektorów normalnych");

                    ui.add_space(8.0);
                    ui.separator();

                    // --- Wykres 1: Profil prędkości v(s) [km/h] ---
                    ui.heading("4. Wykres prędkości v(s) [km/h]");
                    draw_speed_plot(ui, suite);

                    ui.add_space(8.0);
                    ui.checkbox(&mut self.show_gg_diagram, "Pokaż wykres koła tarcia (G-G)");
                    if self.show_gg_diagram {
                        ui.heading("5. Diagram przeciążeń G-G [g]");
                        draw_gg_plot(ui, suite, &self.vehicle_params);
                    }

                    ui.add_space(6.0);
                    ui.small("Kamera 3D: LPM = obrót, PPM = przesunięcie, Rolka = przybliżenie.");
                });
            });

        if changed {
            self.needs_recompute = true;
        }

        panel_res.response.rect.width()
    }
}

/// Rysuje wykres prędkości v(s) [km/h] w funkcji dystansu dla wszystkich 3 trajektorii oraz limitu bocznego
fn draw_speed_plot(ui: &mut egui::Ui, suite: &ComparisonSuite) {
    let opt_pts: PlotPoints = suite
        .racing_line
        .nodes
        .iter()
        .map(|n| [n.distance as f64, (n.speed * 3.6) as f64])
        .collect();

    let limit_pts: PlotPoints = suite
        .racing_line
        .nodes
        .iter()
        .map(|n| [n.distance as f64, (n.v_lat_limit * 3.6) as f64])
        .collect();

    let geom_pts: PlotPoints = suite
        .geometric_line
        .nodes
        .iter()
        .map(|n| [n.distance as f64, (n.speed * 3.6) as f64])
        .collect();

    let cent_pts: PlotPoints = suite
        .centerline
        .nodes
        .iter()
        .map(|n| [n.distance as f64, (n.speed * 3.6) as f64])
        .collect();

    Plot::new("speed_profile_plot")
        .height(175.0)
        .legend(Legend::default())
        .x_axis_label("Dystans s [m]")
        .y_axis_label("v [km/h]")
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new(limit_pts)
                    .name("Limit boczny v_lat")
                    .color(egui::Color32::from_rgb(245, 165, 65)),
            );
            plot_ui.line(
                Line::new(cent_pts)
                    .name("Środkowa")
                    .color(egui::Color32::from_rgb(160, 165, 175)),
            );
            plot_ui.line(
                Line::new(geom_pts)
                    .name("Geometryczna")
                    .color(egui::Color32::from_rgb(70, 210, 235)),
            );
            plot_ui.line(
                Line::new(opt_pts)
                    .name("Wyścigowa (Opt)")
                    .width(2.5_f32)
                    .color(egui::Color32::from_rgb(75, 235, 110)),
            );
        });
}

/// Rysuje wykres koła tarcia (G-G Diagram): przyspieszenie boczne [g] vs przyspieszenie wzdłużne [g]
fn draw_gg_plot(ui: &mut egui::Ui, suite: &ComparisonSuite, vehicle: &VehicleParams) {
    let g = vehicle.g.max(0.1) as f64;

    // Obwiednia teoretycznego koła tarcia na płaskim asfalcie (promień = mu)
    let mu = vehicle.mu as f64;
    let circle_pts: PlotPoints = (0..=64)
        .map(|k| {
            let angle = (k as f64 / 64.0) * std::f64::consts::TAU;
            [mu * angle.cos(), mu * angle.sin()]
        })
        .collect();

    // Punkty robocze pojazdu wzdłuż linii wyścigowej
    let operating_pts: PlotPoints = suite
        .racing_line
        .nodes
        .iter()
        .map(|n| [(n.a_lat as f64) / g, (n.a_long as f64) / g])
        .collect();

    Plot::new("gg_diagram_plot")
        .height(175.0)
        .data_aspect(1.0)
        .legend(Legend::default())
        .x_axis_label("a_lat [g]")
        .y_axis_label("a_long [g]")
        .show(ui, |plot_ui| {
            plot_ui.line(
                Line::new(circle_pts)
                    .name("Koło tarcia (płaskie μ)")
                    .color(egui::Color32::from_rgb(220, 90, 90)),
            );
            plot_ui.points(
                Points::new(operating_pts)
                    .name("Punkty pracy auta")
                    .radius(3.0_f32)
                    .color(egui::Color32::from_rgb(250, 215, 60)),
            );
        });
}
