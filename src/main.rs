use racing_line_3d::optimizer::{
    build_phase_colored_ribbon_mesh, build_solid_ribbon_mesh, ComparisonSuite,
};
use racing_line_3d::track::Track3D;
use racing_line_3d::ui::UiState;
use std::io::{self, Write};
use std::time::Instant;
use three_d::*;

/// Pomocnicza funkcja wypisująca komunikat diagnostyczny z natychmiastowym opróżnieniem bufora konsoli
fn log_step(msg: &str) {
    println!("{}", msg);
    let _ = io::stdout().flush();
}

fn main() {
    log_step("[DIAG 1/6] Start programu racing_line_3d...");

    // Tworzenie pętli zdarzeń Winit i wykrywanie głównego monitora
    let event_loop = winit::event_loop::EventLoop::new();
    let primary_monitor = event_loop
        .primary_monitor()
        .or_else(|| event_loop.available_monitors().next());

    let (win_w, win_h) = (1280_u32, 720_u32);
    let mut start_pos = winit::dpi::PhysicalPosition::new(80_i32, 60_i32);

    if let Some(ref mon) = primary_monitor {
        let mon_size = mon.size();
        let mon_pos = mon.position();
        let scale = mon.scale_factor();
        log_step(&format!(
            "[DIAG 1/6] Wykryto monitor: {:?}, rozdzielczość={}x{} px, pozycja=({}, {}), skalowanie DPI={:.2}",
            mon.name().unwrap_or_else(|| "Domyślny".to_string()),
            mon_size.width,
            mon_size.height,
            mon_pos.x,
            mon_pos.y,
            scale
        ));
        let phys_w = (win_w as f64 * scale) as i32;
        let phys_h = (win_h as f64 * scale) as i32;
        start_pos.x = mon_pos.x + ((mon_size.width as i32 - phys_w) / 2).max(20);
        start_pos.y = mon_pos.y + ((mon_size.height as i32 - phys_h) / 2).max(20);
    } else {
        log_step("[DIAG 1/6] Ostrzeżenie: Winit nie zwrócił informacji o głównym monitorze (używam pozycji (80, 60)).");
    }

    log_step(&format!(
        "[DIAG 2/6] Budowanie okna systemowego Windows ({}x{}, pozycja=({}, {}))...",
        win_w, win_h, start_pos.x, start_pos.y
    ));

    let winit_window = match winit::window::WindowBuilder::new()
        .with_title("Optymalizacja Trajektorii Wyścigowej 3D (Racing Line)")
        .with_inner_size(winit::dpi::LogicalSize::new(win_w as f64, win_h as f64))
        .with_min_inner_size(winit::dpi::LogicalSize::new(640.0, 480.0))
        .with_position(start_pos)
        .with_visible(true)
        .with_active(true)
        .with_decorations(true)
        .build(&event_loop)
    {
        Ok(w) => w,
        Err(err) => {
            eprintln!("[BŁĄD KRYTYCZNY] Nie udało się utworzyć okna Winit: {:?}", err);
            return;
        }
    };

    // Wymuszenie widoczności, przywrócenia i fokusu okna w systemie Windows
    winit_window.set_minimized(false);
    winit_window.set_visible(true);
    winit_window.focus_window();
    winit_window.request_user_attention(Some(winit::window::UserAttentionType::Informational));

    log_step(&format!(
        "[DIAG 2/6] Okno utworzone: outer_position={:?}, inner_size={:?}, tworzenie kontekstu OpenGL (MSAA=0, vsync=false)...",
        winit_window.outer_position(),
        winit_window.inner_size()
    ));

    // Konfiguracja powierzchni OpenGL (MSAA=0 oraz vsync=false zapobiegają blokowaniu bufora WGL na kartach hybrydowych)
    let surface_settings = SurfaceSettings {
        vsync: false,
        depth_buffer: 24,
        stencil_buffer: 0,
        multisamples: 0,
        hardware_acceleration: HardwareAcceleration::Preferred,
    };

    let window = match Window::from_winit_window(winit_window, event_loop, surface_settings, false) {
        Ok(win) => win,
        Err(err) => {
            eprintln!(
                "[BŁĄD KRYTYCZNY] Nie udało się zainicjować kontekstu OpenGL (three-d): {:?}",
                err
            );
            return;
        }
    };

    let init_vp = window.viewport();
    log_step(&format!(
        "[DIAG 3/6] Kontekst OpenGL gotowy! Viewport fizyczny: {}x{} px, współczynnik DPI: {:.2}",
        init_vp.width,
        init_vp.height,
        window.device_pixel_ratio()
    ));

    let context = window.gl();

    // Kamera perspektywiczna ze sterowaniem orbitalnym (myszką)
    let mut camera = Camera::new_perspective(
        init_vp,
        vec3(0.0, 95.0, 145.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.5,
        1500.0,
    );
    let mut control = OrbitControl::new(camera.target(), 10.0, 600.0);

    // Oświetlenie sceny
    let ambient_light = AmbientLight::new(&context, 0.60, Srgba::WHITE);
    let directional_light =
        DirectionalLight::new(&context, 1.35, Srgba::WHITE, vec3(-0.6, -1.0, -0.5));

    log_step("[DIAG 4/6] Generowanie geometrii zakrętu 3D oraz optymalizacja 3 trajektorii...");
    let t_start = Instant::now();
    let mut ui_state = UiState::default();
    let track = Track3D::generate(ui_state.track_params.clone());
    let mut suite = ComparisonSuite::compute(&track, &ui_state.vehicle_params);
    log_step(&format!(
        "[DIAG 4/6] Obliczenia fizyki i optymalizacji zakończone w {:.1} ms (T_opt = {:.3} s).",
        t_start.elapsed().as_secs_f64() * 1000.0,
        suite.racing_line.total_time_sec
    ));

    log_step("[DIAG 5/6] Inicjalizacja interfejsu egui i przesyłanie siatek 3D (Mesh) do GPU...");
    let mut gui = three_d::GUI::new(&context);

    let mut road_gm = Gm::new(
        Mesh::new(&context, &track.build_road_cpu_mesh()),
        ColorMaterial::default(),
    );
    let mut curbs_gm = Gm::new(
        Mesh::new(&context, &track.build_curbs_cpu_mesh()),
        ColorMaterial::default(),
    );
    let mut ribs_gm = Gm::new(
        Mesh::new(&context, &track.build_ribs_cpu_mesh(ui_state.show_normals)),
        ColorMaterial::default(),
    );
    let mut centerline_gm = Gm::new(
        Mesh::new(
            &context,
            &build_solid_ribbon_mesh(
                &track,
                &suite.centerline,
                0.14,
                0.06,
                Srgba::new(165, 170, 180, 255),
                true,
            ),
        ),
        ColorMaterial::default(),
    );
    let mut geometric_gm = Gm::new(
        Mesh::new(
            &context,
            &build_solid_ribbon_mesh(
                &track,
                &suite.geometric_line,
                0.18,
                0.09,
                Srgba::new(65, 210, 240, 255),
                false,
            ),
        ),
        ColorMaterial::default(),
    );
    let mut racing_gm = Gm::new(
        Mesh::new(
            &context,
            &build_phase_colored_ribbon_mesh(&track, &suite.racing_line, 0.34, 0.13),
        ),
        ColorMaterial::default(),
    );

    let mut last_show_normals = ui_state.show_normals;
    ui_state.needs_recompute = false;
    let mut first_frame_logged = false;

    log_step("[DIAG 6/6] Uruchamianie głównej pętli zdarzeń (render_loop)...");

    // Główna pętla renderowania
    window.render_loop(move |mut frame_input| {
        let mut panel_width = 0.0;

        // Aktualizacja panelu egui wraz z wykresami
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |gui_context| {
                panel_width = ui_state.draw_ui(gui_context, &suite);
            },
        );

        // Przeliczenie geometrii i optymalizacji po zmianie dowolnego suwaka
        if ui_state.needs_recompute || ui_state.show_normals != last_show_normals {
            let track = Track3D::generate(ui_state.track_params.clone());
            suite = ComparisonSuite::compute(&track, &ui_state.vehicle_params);

            road_gm.geometry = Mesh::new(&context, &track.build_road_cpu_mesh());
            curbs_gm.geometry = Mesh::new(&context, &track.build_curbs_cpu_mesh());
            ribs_gm.geometry =
                Mesh::new(&context, &track.build_ribs_cpu_mesh(ui_state.show_normals));

            centerline_gm.geometry = Mesh::new(
                &context,
                &build_solid_ribbon_mesh(
                    &track,
                    &suite.centerline,
                    0.14,
                    0.06,
                    Srgba::new(165, 170, 180, 255),
                    true,
                ),
            );
            geometric_gm.geometry = Mesh::new(
                &context,
                &build_solid_ribbon_mesh(
                    &track,
                    &suite.geometric_line,
                    0.18,
                    0.09,
                    Srgba::new(65, 210, 240, 255),
                    false,
                ),
            );
            racing_gm.geometry = Mesh::new(
                &context,
                &build_phase_colored_ribbon_mesh(&track, &suite.racing_line, 0.34, 0.13),
            );

            last_show_normals = ui_state.show_normals;
            ui_state.needs_recompute = false;
        }

        // Dopasowanie obszaru widoku 3D (z zabezpieczeniem przed zerowym rozmiarem przy minimalizacji okna)
        let panel_px = (panel_width * frame_input.device_pixel_ratio) as u32;
        let viewport_3d = Viewport {
            x: panel_px as i32,
            y: 0,
            width: frame_input.viewport.width.saturating_sub(panel_px).max(1),
            height: frame_input.viewport.height.max(1),
        };
        camera.set_viewport(viewport_3d);
        control.handle_events(&mut camera, &mut frame_input.events);

        // Zbieranie aktywnych obiektów 3D do wyrenderowania
        let mut objects: Vec<&dyn Object> = vec![&road_gm];
        if ui_state.show_curbs {
            objects.push(&curbs_gm);
        }
        if ui_state.show_ribs || ui_state.show_normals {
            objects.push(&ribs_gm);
        }
        if ui_state.show_centerline {
            objects.push(&centerline_gm);
        }
        if ui_state.show_geometric_line {
            objects.push(&geometric_gm);
        }
        if ui_state.show_racing_line {
            objects.push(&racing_gm);
        }

        // Renderowanie sceny 3D i nałożenie interfejsu egui
        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.10, 0.12, 0.16, 1.0, 1.0))
            .render(&camera, objects, &[&ambient_light, &directional_light])
            .write(|| gui.render())
            .expect("Błąd renderowania klatki");

        if !first_frame_logged {
            log_step(&format!(
                "[DIAG OK] Pierwsza klatka 3D (viewport={}x{}) wyrenderowana i przesłana do bufora okna!",
                frame_input.viewport.width, frame_input.viewport.height
            ));
            first_frame_logged = true;
        }

        FrameOutput::default()
    });
}
