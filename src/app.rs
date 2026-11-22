use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use eframe::egui::{self, Align, Color32, Layout, Margin, RichText, Stroke};
use egui_extras::{Column, TableBuilder};

use crate::discovery::{self, DiscoveryEvent, Found, Verdict};
use crate::growatt::{DeviceInfo, DeviceKind, Health};
use crate::scanner::{self, Level, ScanConfig, ScanEvent};
use crate::sim::{EmptyIdReply, Model, SimInverter, SimOptions, Simulator};
use crate::theme::{self, Palette};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Source {
    Gateway,
    Simulator,
}

struct LogEntry {
    t: f64,
    level: Level,
    text: String,
}

pub struct App {
    source: Source,
    host: String,
    port: u16,
    first: u8,
    last: u8,
    timeout_ms: u64,
    gap_ms: u64,

    results: Vec<DeviceInfo>,
    selected: Option<u8>,
    target: String,
    current: Option<u8>,
    scan_range: (u8, u8),
    rx: Option<Receiver<ScanEvent>>,
    cancel: Arc<AtomicBool>,

    log: Vec<LogEntry>,
    log_open: bool,
    started: Instant,

    sim: Option<Simulator>,
    sim_opts: SimOptions,

    disc_prefix: String,
    disc_edit_prefix: bool,
    disc_rx: Option<Receiver<DiscoveryEvent>>,
    disc_checked: u32,
    disc_found: Vec<Found>,
    disc_ran: bool,
}

impl App {
    pub fn new() -> Self {
        Self {
            source: Source::Gateway,
            host: String::new(),
            port: 502,
            first: 1,
            last: 32,
            timeout_ms: 500,
            gap_ms: 200,
            results: Vec::new(),
            selected: None,
            target: String::new(),
            current: None,
            scan_range: (1, 32),
            rx: None,
            cancel: Arc::new(AtomicBool::new(false)),
            log: Vec::new(),
            log_open: false,
            started: Instant::now(),
            sim: None,
            sim_opts: SimOptions::default(),
            disc_prefix: discovery::local_prefix().unwrap_or_else(|| "192.168.1.".into()),
            disc_edit_prefix: false,
            disc_rx: None,
            disc_checked: 0,
            disc_found: Vec::new(),
            disc_ran: false,
        }
    }

    /// Screenshot/demo helper: scan the simulator synchronously before the first frame.
    #[cfg(feature = "screenshot")]
    pub fn run_demo_scan(&mut self, ctx: egui::Context) {
        self.source = Source::Simulator;
        self.first = 1;
        self.last = 16;
        self.sim_opts.port = 0; // any free port, so a running copy of the app doesn't clash
        self.start_scan(ctx);
        while self.rx.is_some() {
            std::thread::sleep(Duration::from_millis(20));
            self.poll();
        }
        // MODWATT_SELECT=1 selects ID 7; any other ID selects that row instead.
        if let Some(v) = std::env::var_os("MODWATT_SELECT") {
            let id = v.to_str().and_then(|s| s.parse::<u8>().ok()).filter(|&id| id > 1).unwrap_or(7);
            self.selected = Some(id);
        }
    }

    fn log(&mut self, level: Level, text: impl Into<String>) {
        let t = self.started.elapsed().as_secs_f64();
        self.log.push(LogEntry { t, level, text: text.into() });
    }

    fn scanning(&self) -> bool {
        self.rx.is_some()
    }

    fn discovering(&self) -> bool {
        self.disc_rx.is_some()
    }

    // --- actions -----------------------------------------------------------

    fn start_sim(&mut self) -> bool {
        if self.sim.is_some() {
            return true;
        }
        let mut ids: Vec<u8> = self.sim_opts.inverters.iter().map(|i| i.unit).collect();
        ids.sort();
        if let Some(w) = ids.windows(2).find(|w| w[0] == w[1]) {
            self.log(Level::Error, format!("Simulator: duplicate slave ID {}", w[0]));
            return false;
        }
        match Simulator::start(self.sim_opts.clone()) {
            Ok(sim) => {
                self.log(Level::Info, format!("Simulator listening on 127.0.0.1:{}", sim.port));
                self.sim = Some(sim);
                true
            }
            Err(e) => {
                self.log(Level::Error, format!("Simulator: port {}: {e}", self.sim_opts.port));
                false
            }
        }
    }

    fn stop_sim(&mut self) {
        if self.sim.take().is_some() {
            self.log(Level::Info, "Simulator stopped");
        }
    }

    fn start_scan(&mut self, ctx: egui::Context) {
        let (host, port) = match self.source {
            Source::Gateway => (self.host.trim().to_owned(), self.port),
            Source::Simulator => {
                if !self.start_sim() {
                    return;
                }
                if let Some(max) = self.sim_opts.inverters.iter().map(|i| i.unit).max() {
                    self.last = self.last.max(max);
                }
                ("127.0.0.1".to_owned(), self.sim.as_ref().unwrap().port)
            }
        };
        self.results.clear();
        self.selected = None;
        self.current = None;
        self.cancel = Arc::new(AtomicBool::new(false));
        self.scan_range = (self.first.min(self.last), self.first.max(self.last));
        self.target = format!("{host}:{port}");
        let cfg = ScanConfig {
            host,
            port,
            first: self.scan_range.0,
            last: self.scan_range.1,
            timeout: Duration::from_millis(self.timeout_ms),
            gap: Duration::from_millis(self.gap_ms),
        };
        self.rx = Some(scanner::spawn(cfg, ctx, self.cancel.clone()));
    }

    fn start_discovery(&mut self, ctx: egui::Context) {
        let Some(prefix) = discovery::parse_prefix(&self.disc_prefix) else {
            self.log(Level::Error, format!("“{}” is not a valid network prefix", self.disc_prefix));
            self.disc_edit_prefix = true;
            return;
        };
        let ports = discovery::ports_for(self.port);
        self.disc_found.clear();
        self.disc_checked = 0;
        self.disc_ran = true;
        self.disc_edit_prefix = false;
        self.log(
            Level::Info,
            format!(
                "Searching {}.{}.{}.1–254 for Modbus TCP on ports {}",
                prefix[0],
                prefix[1],
                prefix[2],
                ports.iter().map(u16::to_string).collect::<Vec<_>>().join(", ")
            ),
        );
        self.disc_rx = Some(discovery::spawn(prefix, ports, ctx));
    }

    fn poll(&mut self) {
        if let Some(rx) = self.rx.take() {
            let mut done = false;
            while let Ok(ev) = rx.try_recv() {
                match ev {
                    ScanEvent::Probing(u) => self.current = Some(u),
                    ScanEvent::Found(info) => self.results.push(info),
                    ScanEvent::Log(l, s) => self.log(l, s),
                    ScanEvent::Done => done = true,
                }
            }
            if done {
                self.current = None;
            } else {
                self.rx = Some(rx);
            }
        }
        if let Some(rx) = self.disc_rx.take() {
            let mut done = false;
            while let Ok(ev) = rx.try_recv() {
                match ev {
                    DiscoveryEvent::Progress(n) => self.disc_checked = n,
                    DiscoveryEvent::Found(f) => {
                        let lvl = if f.verdict == Verdict::PortOpen { Level::Info } else { Level::Success };
                        self.log(lvl, format!("{}:{} — {}", f.ip, f.port, describe_found(&f)));
                        self.disc_found.push(f);
                    }
                    DiscoveryEvent::Done => done = true,
                }
            }
            if done {
                let n = self.disc_found.len();
                self.log(
                    if n > 0 { Level::Success } else { Level::Warn },
                    format!("Network search done: {n} open Modbus port(s) found"),
                );
                if n == 0 && !self.disc_prefix.trim_end_matches('.').ends_with("192.168.0") {
                    self.log(
                        Level::Info,
                        "A new ShineMaster-X is at 192.168.0.254 with DHCP off. Connect it straight to this \
                         computer, set this computer to 192.168.0.10, and search 192.168.0. instead.",
                    );
                }
                if let [only] = self.disc_found.as_slice()
                    && only.verdict != Verdict::PortOpen
                    && self.host.trim().is_empty()
                {
                    self.host = only.ip.to_string();
                    self.port = only.port;
                }
            } else {
                self.disc_rx = Some(rx);
            }
        }
    }

    fn results_csv(&self) -> String {
        let mut s = String::from(
            "id,type,dtc,serial,firmware,com_address,baud,status,pv_w,ac_w,today_kwh,total_kwh,\
             battery_soc_pct,battery_v,battery_charge_w,battery_discharge_w,\
             meter_v,meter_w,meter_hz,import_kwh,export_kwh,notes\n",
        );
        let o = |v: Option<f64>| v.map_or(String::new(), |x| format!("{x:.1}"));
        let csv = |t: &str| if t.contains([',', '"']) { format!("\"{}\"", t.replace('"', "\"\"")) } else { t.to_owned() };
        for r in &self.results {
            let b = r.battery.clone().unwrap_or_default();
            let m = r.meter.clone().unwrap_or_default();
            let fields = [
                r.unit.to_string(),
                csv(r.kind.label()),
                r.device_type_code.map_or(String::new(), |c| c.to_string()),
                csv(r.serial.as_deref().unwrap_or("")),
                csv(r.firmware.as_deref().unwrap_or("")),
                r.com_address.map_or(String::new(), |a| a.to_string()),
                r.baud.map_or(String::new(), |b| b.to_string()),
                csv(&r.status_text()),
                o(r.pv_power_w),
                o(r.ac_power_w),
                o(r.energy_today_kwh),
                o(r.energy_total_kwh),
                o(b.soc_pct),
                o(b.voltage_v),
                o(b.charge_w),
                o(b.discharge_w),
                o(m.voltage_v),
                o(m.power_w),
                m.frequency_hz.map_or(String::new(), |x| format!("{x:.2}")),
                o(m.import_kwh),
                o(m.export_kwh),
                csv(&r.notes.join("; ")),
            ];
            s += &fields.join(",");
            s.push('\n');
        }
        s
    }

    // --- header --------------------------------------------------------------

    fn header(&mut self, ui: &mut egui::Ui, p: &Palette) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("modwatt").size(17.0).strong().color(p.text));
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let dark = ui.visuals().dark_mode;
                if theme::text_button(ui, if dark { "☀" } else { "🌙" }, p.muted)
                    .on_hover_text(if dark { "Light mode" } else { "Dark mode" })
                    .clicked()
                {
                    ui.ctx().set_theme(if dark { egui::Theme::Light } else { egui::Theme::Dark });
                }
                ui.add_space(12.0);
                if let Some(u) = self.current {
                    theme::dot_label(ui, &format!("Scanning ID {u}"), p.accent, p.muted);
                } else if self.discovering() {
                    theme::dot_label(ui, "Searching network", p.accent, p.muted);
                } else if let Some(sim) = &self.sim {
                    theme::dot_label(ui, &format!("Simulator on :{}", sim.port), p.success, p.muted);
                } else {
                    theme::dot_label(ui, "Ready", p.faint, p.muted);
                }
            });
        });
    }

    // --- sidebar -------------------------------------------------------------

    fn sidebar(&mut self, ui: &mut egui::Ui, p: &Palette) {
        let ctx = ui.ctx().clone();
        let width = ui.available_width();

        // Pinned action at the bottom; settings scroll above it.
        ui.with_layout(Layout::bottom_up(Align::Min), |ui| {
            if self.scanning() {
                if theme::secondary_button(ui, "Stop", width).clicked() {
                    self.cancel.store(true, Ordering::Relaxed);
                }
                if let Some(u) = self.current {
                    let (lo, hi) = self.scan_range;
                    let frac = (u - lo) as f32 / (hi - lo + 1) as f32;
                    ui.horizontal(|ui| {
                        theme::hint(ui, format!("Checking ID {u}"));
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            theme::hint(ui, format!("{:.0}%", frac * 100.0));
                        });
                    });
                    ui.add(egui::ProgressBar::new(frac).desired_width(width).desired_height(4.0).fill(p.text));
                }
            } else {
                let ready = match self.source {
                    Source::Gateway => !self.host.trim().is_empty(),
                    Source::Simulator => !self.sim_opts.inverters.is_empty(),
                };
                if theme::primary_button(ui, "Scan", width, ready && !self.discovering()).clicked() {
                    self.start_scan(ctx.clone());
                }
            }
            ui.add_space(8.0);

            ui.with_layout(Layout::top_down(Align::Min), |ui| {
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                    ui.add_enabled_ui(!self.scanning(), |ui| {
                        theme::segmented(
                            ui,
                            &mut self.source,
                            &[(Source::Gateway, "Gateway"), (Source::Simulator, "Simulator")],
                        );
                        ui.add_space(14.0);
                        match self.source {
                            Source::Gateway => self.gateway_section(ui, p),
                            Source::Simulator => self.simulator_section(ui, p),
                        }
                        ui.add_space(18.0);
                        self.range_section(ui);
                        ui.add_space(18.0);
                        self.advanced_section(ui, p);
                    });
                });
            });
        });
    }

    fn gateway_section(&mut self, ui: &mut egui::Ui, p: &Palette) {
        theme::field(ui, "Gateway IP address", |ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.host)
                    .hint_text("192.168.1.50")
                    .margin(Margin::symmetric(10, 7))
                    .desired_width(f32::INFINITY),
            );
        });

        if self.discovering() {
            ui.horizontal(|ui| {
                ui.spinner();
                theme::hint(ui, format!("Searching your network… {}/254", self.disc_checked));
            });
        } else if self.disc_edit_prefix {
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.disc_prefix)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(110.0),
                );
                theme::hint(ui, "1–254");
                if ui.button("Search").clicked() {
                    self.start_discovery(ui.ctx().clone());
                }
            });
        } else {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                if theme::text_button(ui, "Find on network", p.text).clicked() {
                    self.start_discovery(ui.ctx().clone());
                }
                theme::hint(ui, "·");
                if theme::text_button(ui, &format!("{}x", self.disc_prefix), p.muted)
                    .on_hover_text("Change which network to search")
                    .clicked()
                {
                    self.disc_edit_prefix = true;
                }
            });
        }

        let mut pick = None;
        for f in &self.disc_found {
            let is_modbus = f.verdict != Verdict::PortOpen;
            let selected = self.host.trim() == f.ip.to_string() && self.port == f.port;
            let resp = egui::Frame::new()
                .fill(if selected { p.surface } else { Color32::TRANSPARENT })
                .stroke(Stroke::new(1.0, if selected { p.faint } else { p.border }))
                .corner_radius(8)
                .inner_margin(Margin::symmetric(10, 8))
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.vertical(|ui| {
                        ui.spacing_mut().item_spacing.y = 2.0;
                        let addr = if f.port == 502 { f.ip.to_string() } else { format!("{}:{}", f.ip, f.port) };
                        theme::dot_label(ui, &addr, if is_modbus { p.success } else { p.faint }, p.text);
                        theme::hint(ui, describe_found(f));
                    });
                })
                .response
                .interact(egui::Sense::click())
                .on_hover_cursor(egui::CursorIcon::PointingHand);
            if resp.clicked() {
                pick = Some((f.ip.to_string(), f.port));
            }
        }
        if let Some((ip, port)) = pick {
            self.host = ip;
            self.port = port;
        }
        if self.disc_ran && !self.discovering() && self.disc_found.is_empty() {
            ui.label(
                RichText::new("No gateway found. Check it's powered, on the same router, and has Modbus TCP enabled.")
                    .small()
                    .color(p.warn),
            );
        }
    }

    fn simulator_section(&mut self, ui: &mut egui::Ui, p: &Palette) {
        if let Some(sim) = &self.sim {
            theme::dot_label(ui, &format!("Running on 127.0.0.1:{}", sim.port), p.success, p.text);
            theme::hint(ui, format!("{} requests served", sim.requests()));
            ui.ctx().request_repaint_after(Duration::from_millis(500));
            if theme::text_button(ui, "Stop simulator", p.text).clicked() {
                self.stop_sim();
            }
            return;
        }

        theme::field(ui, "Simulated devices", |ui| {
            let o = &mut self.sim_opts;
            let mut remove = None;
            egui::Grid::new("sim_devices").num_columns(5).spacing([6.0, 6.0]).show(ui, |ui| {
                for h in ["ID", "Model", "kW", "Fault", ""] {
                    ui.label(RichText::new(h).small().color(p.faint));
                }
                ui.end_row();
                for (i, inv) in o.inverters.iter_mut().enumerate() {
                    ui.add(egui::DragValue::new(&mut inv.unit).range(1..=247));
                    egui::ComboBox::from_id_salt(("model", i))
                        .width(84.0)
                        .selected_text(inv.model.label())
                        .show_ui(ui, |ui| {
                            for m in Model::ALL {
                                ui.selectable_value(&mut inv.model, m, m.label()).on_hover_text(m.hint());
                            }
                        })
                        .response
                        .on_hover_text(inv.model.hint());
                    if inv.model.is_meter() {
                        ui.label(RichText::new("—").color(p.faint)).on_hover_text("Meters have no rated power");
                    } else {
                        ui.add(egui::DragValue::new(&mut inv.rated_kw).range(1.0..=250.0).speed(0.5));
                    }
                    ui.checkbox(&mut inv.fault, "");
                    if theme::text_button(ui, "×", p.faint).on_hover_text("Remove").clicked() {
                        remove = Some(i);
                    }
                    ui.end_row();
                }
            });
            if let Some(i) = remove {
                o.inverters.remove(i);
            }
            if theme::text_button(ui, "+ Add device", p.text).clicked() {
                let next = (1..=247u8).find(|id| !o.inverters.iter().any(|i| i.unit == *id)).unwrap_or(1);
                o.inverters.push(SimInverter { unit: next, model: Model::Mid, rated_kw: 50.0, fault: false });
            }
        });
    }

    fn range_section(&mut self, ui: &mut egui::Ui) {
        theme::field(ui, "Slave IDs", |ui| {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut self.first).range(1..=247));
                theme::hint(ui, "to");
                ui.add(egui::DragValue::new(&mut self.last).range(1..=247));
            });
            let mut preset = if self.first == 1 { self.last } else { 0 };
            if theme::segmented(ui, &mut preset, &[(32u8, "32"), (64, "64"), (124, "124"), (247, "All")]) {
                self.first = 1;
                self.last = preset;
            }
            theme::hint(ui, "ShineMaster 32 · 4G 64 · ShineMaster-X 124");
        });
    }

    fn advanced_section(&mut self, ui: &mut egui::Ui, p: &Palette) {
        egui::CollapsingHeader::new(RichText::new("Advanced").color(p.muted))
            .id_salt("advanced")
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("advanced_grid").num_columns(2).spacing([12.0, 8.0]).show(ui, |ui| {
                    if self.source == Source::Gateway {
                        ui.label(RichText::new("Port").color(p.muted));
                        ui.add(egui::DragValue::new(&mut self.port).range(1..=65535));
                        ui.end_row();
                    }
                    ui.label(RichText::new("Timeout").color(p.muted));
                    ui.add(egui::DragValue::new(&mut self.timeout_ms).range(50..=10_000).suffix(" ms"));
                    ui.end_row();
                    ui.label(RichText::new("Request gap").color(p.muted))
                        .on_hover_text("Quiet time on the RS485 bus between requests");
                    ui.add(egui::DragValue::new(&mut self.gap_ms).range(0..=5_000).suffix(" ms"));
                    ui.end_row();
                    if self.source == Source::Simulator && self.sim.is_none() {
                        ui.label(RichText::new("Listen port").color(p.muted));
                        ui.add(egui::DragValue::new(&mut self.sim_opts.port).range(1..=65535));
                        ui.end_row();
                    }
                });

                if self.source == Source::Simulator && self.sim.is_none() {
                    let o = &mut self.sim_opts;
                    let mut silent = o.empty_id_reply == EmptyIdReply::Silent;
                    ui.checkbox(&mut silent, "Empty IDs stay silent")
                        .on_hover_text("Off: gateway answers exception 0x0B. On: no reply, so the scanner waits out its timeout.");
                    o.empty_id_reply = if silent { EmptyIdReply::Silent } else { EmptyIdReply::GatewayException };
                    let mut slow = o.serial_baud.is_some();
                    ui.checkbox(&mut slow, "9600-baud bus timing");
                    o.serial_baud = slow.then_some(9600);
                    if theme::text_button(ui, "Start without scanning", p.text)
                        .on_hover_text("Run the simulator for other Modbus tools")
                        .clicked()
                    {
                        self.start_sim();
                    }
                }

                let (lo, hi) = (self.first.min(self.last), self.first.max(self.last));
                let worst = (hi - lo + 1) as u64 * (self.timeout_ms + self.gap_ms);
                theme::hint(
                    ui,
                    format!(
                        "Growatt recommends an 850–1000 ms gap. Worst case for this range: ~{} s.",
                        worst.div_ceil(1000)
                    ),
                );
            });
    }

    // --- main area -----------------------------------------------------------

    fn stats(&self, ui: &mut egui::Ui, p: &Palette) {
        let total = self.results.len();
        let online = self.results.iter().filter(|r| r.health() == Health::Ok).count();
        let faults = self.results.iter().filter(|r| r.health() == Health::Fault).count();
        // Meters measure the grid connection, not production: keep them out of the inverter sums.
        let inverters = || self.results.iter().filter(|r| !r.kind.is_meter());
        let ac_kw: f64 = inverters().filter_map(|r| r.ac_power_w).sum::<f64>() / 1000.0;
        let today: f64 = inverters().filter_map(|r| r.energy_today_kwh).sum();
        // Several meters usually measure different points (grid, PV, load), so summing them is
        // meaningless: show the lowest-ID meter, which is normally the grid meter.
        let grid: Vec<(u8, f64)> = self.results.iter().filter_map(|r| Some((r.unit, r.meter.as_ref()?.power_w?))).collect();
        let socs: Vec<f64> = self.results.iter().filter_map(|r| r.battery.as_ref()?.soc_pct).collect();

        struct Tile {
            label: &'static str,
            value: String,
            unit: &'static str,
            color: Color32,
            /// Small text on the right of the label row.
            note: Option<(String, Color32)>,
        }
        let tile = |label, value, unit| Tile { label, value, unit, color: p.text, note: None };
        let mut tiles = vec![
            Tile {
                note: (faults > 0).then(|| (format!("{faults} fault{}", if faults == 1 { "" } else { "s" }), p.danger)),
                ..tile("Online", format!("{online}/{total}"), "")
            },
            tile("Power now", group(ac_kw, 1), "kW"),
            tile("Energy today", group(today, 1), "kWh"),
        ];
        if let Some(&(unit, w)) = grid.iter().min_by_key(|(u, _)| *u) {
            let label = match w {
                w if w <= -50.0 => "Grid export",
                w if w >= 50.0 => "Grid import",
                _ => "Grid",
            };
            let note = (grid.len() > 1).then(|| (format!("ID {unit}"), p.faint));
            tiles.push(Tile { note, ..tile(label, group(w.abs() / 1000.0, 1), "kW") });
        }
        if !socs.is_empty() {
            let avg = socs.iter().sum::<f64>() / socs.len() as f64;
            let label = if socs.len() > 1 { "Battery (avg)" } else { "Battery" };
            tiles.push(Tile { color: soc_color(p, avg), ..tile(label, group(avg, 0), "%") });
        }
        let value_size = if tiles.len() > 4 { 22.0 } else { 26.0 };

        ui.columns(tiles.len(), |cols| {
            for (ui, t) in cols.iter_mut().zip(tiles) {
                theme::card(p).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.vertical(|ui| {
                        ui.spacing_mut().item_spacing.y = 4.0;
                        ui.horizontal(|ui| {
                            ui.add(egui::Label::new(RichText::new(t.label).small().color(p.muted)).truncate());
                            if let Some((note, c)) = &t.note {
                                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                    ui.label(RichText::new(note).small().color(*c));
                                });
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 5.0;
                            ui.label(RichText::new(t.value).size(value_size).color(t.color));
                            if !t.unit.is_empty() {
                                ui.label(RichText::new(t.unit).color(p.muted));
                            }
                        });
                    });
                });
            }
        });
    }

    /// Returns the unit ID of a clicked row.
    fn results_table(&self, ui: &mut egui::Ui, p: &Palette) -> Option<u8> {
        if self.results.is_empty() {
            ui.vertical_centered(|ui| {
                ui.add_space((ui.available_height() / 2.0 - 60.0).max(32.0));
                if self.scanning() {
                    ui.spinner();
                    ui.label(RichText::new("Scanning the RS485 bus…").color(p.muted));
                } else {
                    ui.label(RichText::new("No devices yet").size(17.0).color(p.text));
                    ui.label(
                        RichText::new(match self.source {
                            Source::Gateway => "Enter your gateway's IP address, then press Scan.",
                            Source::Simulator => "Press Scan to try the app with simulated devices.",
                        })
                        .color(p.muted),
                    );
                }
            });
            return None;
        }

        let right = Layout::right_to_left(Align::Center);
        let head = |ui: &mut egui::Ui, t: &str| {
            ui.label(RichText::new(t).small().color(p.muted));
        };
        let dash = |ui: &mut egui::Ui| {
            ui.with_layout(right, |ui| ui.label(RichText::new("—").color(p.faint)));
        };
        let num = |ui: &mut egui::Ui, v: Option<f64>, unit: &str| match v {
            Some(x) => {
                ui.with_layout(right, |ui| {
                    ui.label(RichText::new(unit).small().color(p.faint));
                    ui.label(RichText::new(group(x, 1)).color(p.text));
                });
            }
            None => dash(ui),
        };

        let mut clicked = None;
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
            TableBuilder::new(ui)
                .striped(false)
                .resizable(false)
                .sense(egui::Sense::click())
                .auto_shrink([false, false])
                .cell_layout(Layout::left_to_right(Align::Center))
                .column(Column::exact(44.0)) // ID
                .column(Column::exact(68.0)) // Type
                .column(Column::remainder().at_least(96.0).clip(true)) // Serial
                .column(Column::exact(84.0)) // Status
                .column(Column::exact(86.0)) // Power
                .column(Column::exact(52.0)) // Battery
                .column(Column::exact(76.0)) // Today
                .column(Column::exact(92.0)) // Lifetime
                .header(32.0, |mut h| {
                    for t in ["ID", "Type", "Serial number", "Status"] {
                        h.col(|ui| head(ui, t));
                    }
                    for (t, tip) in [
                        ("Power", "Inverter output; for meters, grid import or export"),
                        ("Battery", "State of charge"),
                        ("Today", "Energy produced today"),
                        ("Lifetime", "Lifetime energy produced"),
                    ] {
                        h.col(|ui| {
                            ui.with_layout(right, |ui| head(ui, t)).response.on_hover_text(tip);
                        });
                    }
                })
                .body(|mut body| {
                    for r in &self.results {
                        let meter = r.kind.is_meter();
                        body.row(40.0, |mut row| {
                            row.set_selected(self.selected == Some(r.unit));
                            row.col(|ui| {
                                ui.label(RichText::new(r.unit.to_string()).color(p.muted));
                                if r.address_mismatch() {
                                    ui.label(RichText::new("!").strong().color(p.warn))
                                        .on_hover_text("Address mismatch — see details");
                                }
                            });
                            row.col(|ui| {
                                ui.add(egui::Label::new(RichText::new(kind_short(r.kind)).color(p.muted)).truncate())
                                    .on_hover_text(r.kind.label());
                            });
                            row.col(|ui| {
                                let sn = r.serial.as_deref().unwrap_or("—");
                                ui.add(egui::Label::new(RichText::new(sn).monospace().color(p.text)).truncate());
                            });
                            row.col(|ui| {
                                theme::dot_label(ui, &r.status_text(), health_color(p, r.health()), p.text);
                            });
                            row.col(|ui| {
                                if meter {
                                    match r.meter.as_ref().and_then(|m| m.power_w) {
                                        Some(w) => {
                                            let (dir, tip) = grid_direction(w);
                                            ui.with_layout(right, |ui| {
                                                ui.label(RichText::new(format!("kW {dir}")).small().color(p.faint));
                                                ui.label(RichText::new(group(w.abs() / 1000.0, 1)).color(p.text));
                                            })
                                            .response
                                            .on_hover_text(tip);
                                        }
                                        None => dash(ui),
                                    }
                                } else {
                                    num(ui, r.ac_power_w.map(|w| w / 1000.0), "kW");
                                }
                            });
                            row.col(|ui| match r.battery.as_ref().and_then(|b| b.soc_pct) {
                                Some(soc) => {
                                    ui.with_layout(right, |ui| {
                                        ui.label(RichText::new("%").small().color(p.faint));
                                        ui.label(RichText::new(group(soc, 0)).color(soc_color(p, soc)));
                                    });
                                }
                                None => dash(ui),
                            });
                            if meter {
                                row.col(dash);
                                row.col(dash);
                            } else {
                                row.col(|ui| num(ui, r.energy_today_kwh, "kWh"));
                                row.col(|ui| num(ui, r.energy_total_kwh, "kWh"));
                            }
                            if row.response().clicked() {
                                clicked = Some(r.unit);
                            }
                        });
                    }
                });
        });
        clicked
    }

    fn details(&mut self, ui: &mut egui::Ui, p: &Palette) {
        let Some(r) = self.selected.and_then(|u| self.results.iter().find(|r| r.unit == u)).cloned() else {
            return;
        };
        ui.horizontal(|ui| {
            ui.label(RichText::new(format!("{} {}", kind_short(r.kind), r.unit)).size(17.0).strong());
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                if theme::text_button(ui, "×", p.muted).on_hover_text("Close").clicked() {
                    self.selected = None;
                }
            });
        });
        theme::dot_label(ui, &r.status_text(), health_color(p, r.health()), p.text);
        ui.add_space(8.0);

        let opt = |v: Option<f64>, unit: &str| v.map_or("—".to_owned(), |x| format!("{} {unit}", group(x, 1)));
        let kw = |w: Option<f64>| opt(w.map(|w| w / 1000.0), "kW");
        let show = |ui: &mut egui::Ui, rows: Vec<(&str, String)>| {
            for (k, v) in rows {
                ui.vertical(|ui| {
                    ui.spacing_mut().item_spacing.y = 2.0;
                    theme::hint(ui, k);
                    ui.add(egui::Label::new(RichText::new(v).color(p.text)).wrap());
                });
            }
        };

        let mut device = vec![("Type", r.kind.label().to_owned())];
        if let Some(c) = r.device_type_code {
            device.push(("Device type code (DTC)", c.to_string()));
        }
        device.push(("Serial number", r.serial.clone().unwrap_or("—".into())));
        show(ui, device);

        if !r.kind.is_meter() {
            theme::section(ui, "Output");
            let ac = if r.kind == DeviceKind::OffGrid { "Load power" } else { "AC power" };
            show(
                ui,
                vec![
                    (ac, kw(r.ac_power_w)),
                    ("PV power", kw(r.pv_power_w)),
                    ("Energy today", opt(r.energy_today_kwh, "kWh")),
                    ("Lifetime energy", opt(r.energy_total_kwh, "kWh")),
                ],
            );
        }

        if let Some(b) = &r.battery {
            theme::section(ui, "Battery");
            let flow = match (b.charge_w, b.discharge_w) {
                (Some(c), _) if c > 0.0 => format!("Charging {}", kw(Some(c))),
                (_, Some(d)) if d > 0.0 => format!("Discharging {}", kw(Some(d))),
                (None, None) => "—".into(),
                _ => "Idle".into(),
            };
            show(
                ui,
                vec![
                    ("State of charge", b.soc_pct.map_or("—".into(), |s| format!("{} %", group(s, 0)))),
                    ("Voltage", opt(b.voltage_v, "V")),
                    ("Power", flow),
                ],
            );
        }

        if let Some(m) = &r.meter {
            theme::section(ui, "Grid meter");
            let power = m.power_w.map_or("—".into(), |w| {
                let dir = if w < 0.0 { "exporting" } else { "importing" };
                format!("{} kW {dir}", group(w.abs() / 1000.0, 1))
            });
            show(
                ui,
                vec![
                    ("Grid power", power),
                    ("Voltage", opt(m.voltage_v, "V")),
                    ("Frequency", m.frequency_hz.map_or("—".into(), |f| format!("{f:.2} Hz"))),
                    ("Imported", opt(m.import_kwh, "kWh")),
                    ("Exported", opt(m.export_kwh, "kWh")),
                ],
            );
        }

        theme::section(ui, "Connection");
        show(
            ui,
            vec![
                ("RS485 address", r.com_address.map_or("—".into(), |a| a.to_string())),
                ("Baud rate", r.baud.map_or("—".into(), |b| b.to_string())),
                ("Firmware", r.firmware.clone().unwrap_or("—".into())),
                ("Polled as ID", r.unit.to_string()),
            ],
        );
        if !r.notes.is_empty() {
            theme::section(ui, "Probe notes");
            ui.vertical(|ui| {
                ui.spacing_mut().item_spacing.y = 3.0;
                for n in &r.notes {
                    ui.add(egui::Label::new(RichText::new(n).small().monospace().color(p.muted)).wrap());
                }
            });
        }
        if r.address_mismatch() {
            ui.add_space(4.0);
            ui.label(
                RichText::new(
                    "This device reports a different RS485 address than the ID it was polled as. The gateway may be ignoring the unit ID.",
                )
                .small()
                .color(p.warn),
            );
        }
    }

    fn status_bar(&mut self, ui: &mut egui::Ui, p: &Palette) {
        ui.horizontal(|ui| {
            match self.log.last() {
                Some(e) => {
                    theme::dot_label(ui, &e.text, level_color(p, e.level), p.muted);
                }
                None => theme::hint(ui, "No activity yet"),
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                let label = format!("{} Activity", if self.log_open { "Hide" } else { "Show" });
                if theme::text_button(ui, &label, p.muted).clicked() {
                    self.log_open = !self.log_open;
                }
                if self.log_open && !self.log.is_empty() && theme::text_button(ui, "Clear", p.muted).clicked() {
                    self.log.clear();
                }
            });
        });
        if !self.log_open {
            return;
        }
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .stick_to_bottom(true)
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                for e in &self.log {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format_elapsed(e.t)).monospace().small().color(p.faint));
                        let c = if e.level == Level::Info { p.text } else { level_color(p, e.level) };
                        theme::dot_label(ui, &e.text, level_color(p, e.level), c);
                    });
                }
            });
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.poll();
        let p = theme::palette(ui);

        egui::Panel::top("header")
            .frame(egui::Frame::new().fill(p.bg).inner_margin(Margin::symmetric(24, 14)))
            .show(ui, |ui| self.header(ui, &p));

        let bar = egui::Panel::bottom("status").frame(
            egui::Frame::new().fill(p.chrome).inner_margin(Margin::symmetric(24, 8)),
        );
        let bar = if self.log_open {
            bar.resizable(true).default_size(180.0).min_size(90.0)
        } else {
            bar.resizable(false).exact_size(40.0)
        };
        bar.show(ui, |ui| self.status_bar(ui, &p));

        egui::Panel::left("sidebar")
            .resizable(false)
            .exact_size(300.0)
            .frame(egui::Frame::new().fill(p.chrome).inner_margin(Margin::same(20)))
            .show(ui, |ui| self.sidebar(ui, &p));

        if self.selected.is_some_and(|u| self.results.iter().any(|r| r.unit == u)) {
            egui::Panel::right("details")
                .resizable(false)
                .exact_size(260.0)
                .frame(egui::Frame::new().fill(p.chrome).inner_margin(Margin::same(20)))
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| self.details(ui, &p));
                });
        }

        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(p.bg).inner_margin(Margin::symmetric(28, 20)))
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("Devices").size(22.0).strong());
                    if !self.target.is_empty() {
                        ui.label(RichText::new(&self.target).color(p.faint));
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if !self.results.is_empty()
                            && theme::text_button(ui, "Copy CSV", p.muted)
                                .on_hover_text("Copy the results to the clipboard as CSV")
                                .clicked()
                        {
                            ui.ctx().copy_text(self.results_csv());
                            self.log(Level::Info, format!("Copied {} rows as CSV", self.results.len()));
                        }
                    });
                });
                ui.add_space(8.0);
                self.stats(ui, &p);
                ui.add_space(16.0);
                if let Some(u) = self.results_table(ui, &p) {
                    self.selected = if self.selected == Some(u) { None } else { Some(u) };
                }
            });
    }
}

fn health_color(p: &Palette, h: Health) -> Color32 {
    match h {
        Health::Ok => p.success,
        Health::Idle => p.warn,
        Health::Fault => p.danger,
        Health::Unknown => p.faint,
    }
}

fn soc_color(p: &Palette, soc: f64) -> Color32 {
    if soc < 20.0 { p.warn } else { p.text }
}

/// Compact kind label for the table and the details title.
fn kind_short(k: DeviceKind) -> &'static str {
    match k {
        DeviceKind::Meter(_) => "Meter",
        DeviceKind::Unknown => "Device",
        k => k.label(),
    }
}

/// Meter power sign convention: positive = import from grid.
fn grid_direction(w: f64) -> (&'static str, &'static str) {
    if w < 0.0 { ("exp", "Exporting to the grid") } else { ("imp", "Importing from the grid") }
}

fn level_color(p: &Palette, l: Level) -> Color32 {
    match l {
        Level::Info => p.faint,
        Level::Success => p.success,
        Level::Warn => p.warn,
        Level::Error => p.danger,
    }
}

fn format_elapsed(t: f64) -> String {
    let s = t as u64;
    format!("{:02}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
}

/// Fixed decimals with thousands separators: 12345.6 -> "12,345.60".
fn group(x: f64, decimals: usize) -> String {
    let s = format!("{:.*}", decimals, x.abs());
    let (int, frac) = s.split_once('.').map_or((s.as_str(), ""), |(a, b)| (a, b));
    let mut out = String::new();
    for (i, ch) in int.chars().enumerate() {
        if i > 0 && (int.len() - i) % 3 == 0 {
            out.push(',');
        }
        out.push(ch);
    }
    if !frac.is_empty() {
        out.push('.');
        out.push_str(frac);
    }
    if x < 0.0 { format!("-{out}") } else { out }
}

/// "Growatt ShineMaster · Modbus gateway, device at ID 1"
fn describe_found(f: &Found) -> String {
    match f.vendor {
        Some(v) => format!("{v} · {}", f.verdict.describe()),
        None => f.verdict.describe(),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn groups_thousands() {
        assert_eq!(super::group(1234567.891, 2), "1,234,567.89");
        assert_eq!(super::group(999.0, 1), "999.0");
        assert_eq!(super::group(0.0, 0), "0");
    }
}
