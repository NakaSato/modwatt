mod app;
mod discovery;
mod growatt;
mod meter;
mod modbus;
mod scanner;
mod sim;
mod theme;

use eframe::egui;
use sim::{SimOptions, Simulator};

fn main() -> eframe::Result {
    // Headless mode: `modwatt --sim [port]` runs only the simulated gateway, so
    // other Modbus tools (mbpoll, QModMaster, scripts) can poll it too.
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("--sim") {
        let port = args.get(2).and_then(|p| p.parse().ok()).unwrap_or(5020);
        let opts = SimOptions { port, ..Default::default() };
        let ids: Vec<String> = opts.inverters.iter().map(|i| format!("{} {}", i.unit, i.model.label())).collect();
        match Simulator::start(opts) {
            Ok(sim) => {
                println!("Simulated Growatt gateway on 127.0.0.1:{} (device IDs: {})", sim.port, ids.join(", "));
                println!("Ctrl+C to stop.");
                loop {
                    std::thread::park();
                }
            }
            Err(e) => {
                eprintln!("Could not start simulator on port {port}: {e}");
                std::process::exit(1);
            }
        }
    }

    #[allow(unused_mut)]
    let mut options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("modwatt")
            .with_inner_size([1280.0, 760.0])
            .with_min_inner_size([960.0, 560.0]),
        ..Default::default()
    };
    #[cfg(feature = "screenshot")]
    {
        options.renderer = eframe::Renderer::Glow;
    }
    eframe::run_native(
        "modwatt",
        options,
        Box::new(|cc| {
            theme::install(&cc.egui_ctx);
            #[allow(unused_mut)]
            let mut app = app::App::new();
            #[cfg(feature = "screenshot")]
            {
                // eframe captures exactly pass #2; keep one pass per frame so
                // that pass is a fully painted frame, not a measuring pass.
                cc.egui_ctx.options_mut(|o| o.max_passes = std::num::NonZeroUsize::MIN);
                if std::env::var("MODWATT_THEME").as_deref() == Ok("light") {
                    cc.egui_ctx.set_theme(egui::Theme::Light);
                }
                if std::env::var_os("MODWATT_DEMO").is_some() {
                    app.run_demo_scan(cc.egui_ctx.clone());
                }
            }
            Ok(Box::new(app))
        }),
    )
}
