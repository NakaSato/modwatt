//! Background scan of RS485 slave addresses through a Modbus TCP gateway.
//!
//! RS485 is a single-master half-duplex bus, so addresses are polled strictly
//! one at a time over a single TCP connection.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::Duration;

use eframe::egui;

use crate::growatt::{self, DeviceInfo, DeviceKind};
use crate::modbus::ModbusTcp;

#[derive(Clone, Debug)]
pub struct ScanConfig {
    pub host: String,
    pub port: u16,
    pub first: u8,
    pub last: u8,
    pub timeout: Duration,
    /// Minimum gap between consecutive RS485 requests.
    pub gap: Duration,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Info,
    Success,
    Warn,
    Error,
}

// `Found` is large, but events are rare (one per device) and app.rs matches on
// the unboxed `DeviceInfo`.
#[allow(clippy::large_enum_variant)]
pub enum ScanEvent {
    Probing(u8),
    Found(DeviceInfo),
    Log(Level, String),
    Done,
}

pub fn spawn(cfg: ScanConfig, ctx: egui::Context, cancel: Arc<AtomicBool>) -> Receiver<ScanEvent> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        run(&cfg, &tx, &ctx, &cancel);
        let _ = tx.send(ScanEvent::Done);
        ctx.request_repaint();
    });
    rx
}

fn run(cfg: &ScanConfig, tx: &Sender<ScanEvent>, ctx: &egui::Context, cancel: &AtomicBool) {
    let send = |ev: ScanEvent| {
        let _ = tx.send(ev);
        ctx.request_repaint();
    };

    send(ScanEvent::Log(Level::Info, format!(
        "Connecting to {}:{} …",
        cfg.host, cfg.port
    )));
    let mut client = match ModbusTcp::new(&cfg.host, cfg.port, cfg.timeout, cfg.gap) {
        Ok(c) => c,
        Err(e) => {
            send(ScanEvent::Log(Level::Error, format!("Gateway unreachable: {e}")));
            return;
        }
    };
    send(ScanEvent::Log(Level::Info, format!(
        "Connected. Scanning IDs {}–{} (timeout {} ms, gap {} ms)",
        cfg.first,
        cfg.last,
        cfg.timeout.as_millis(),
        cfg.gap.as_millis()
    )));

    let mut found: Vec<DeviceKind> = Vec::new();
    for unit in cfg.first..=cfg.last {
        if cancel.load(Ordering::Relaxed) {
            send(ScanEvent::Log(Level::Warn, "Scan stopped.".into()));
            return;
        }
        send(ScanEvent::Probing(unit));
        match growatt::probe(&mut client, unit) {
            Ok(Some(info)) => {
                found.push(info.kind);
                send(ScanEvent::Log(Level::Success, format!(
                    "ID {unit}: found {} {}",
                    info.kind.label(),
                    info.serial.as_deref().unwrap_or("(no serial)")
                )));
                if info.kind == DeviceKind::Unknown {
                    send(ScanEvent::Log(Level::Warn, format!(
                        "ID {unit}: unrecognised device — {}",
                        info.notes.join("; ")
                    )));
                }
                if info.address_mismatch() {
                    send(ScanEvent::Log(Level::Warn, format!(
                        "ID {unit}: device reports COM address {} — the gateway may be ignoring the unit ID",
                        info.com_address.unwrap()
                    )));
                }
                send(ScanEvent::Found(info));
            }
            Ok(None) => {}
            Err(e) => {
                send(ScanEvent::Log(Level::Error, format!("ID {unit}: lost gateway: {e}")));
                return;
            }
        }
    }
    send(ScanEvent::Log(Level::Success, format!("Scan complete: {}", summary(&found))));
}

/// "3 device(s) found (2 Inverter, 1 Meter (Eastron))."
fn summary(kinds: &[DeviceKind]) -> String {
    let mut counts: Vec<(&'static str, usize)> = Vec::new();
    for k in kinds {
        match counts.iter_mut().find(|(l, _)| *l == k.label()) {
            Some((_, n)) => *n += 1,
            None => counts.push((k.label(), 1)),
        }
    }
    let parts: Vec<String> = counts.iter().map(|(l, n)| format!("{n} {l}")).collect();
    if parts.is_empty() {
        format!("{} device(s) found.", kinds.len())
    } else {
        format!("{} device(s) found ({}).", kinds.len(), parts.join(", "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::modbus::Table;
    use crate::sim::{EmptyIdReply, SimOptions, Simulator};

    fn scan(port: u16, first: u8, last: u8) -> Vec<DeviceInfo> {
        let cfg = ScanConfig {
            host: "127.0.0.1".into(),
            port,
            first,
            last,
            timeout: Duration::from_millis(100),
            gap: Duration::ZERO,
        };
        let rx = spawn(cfg, egui::Context::default(), Arc::new(AtomicBool::new(false)));
        rx.iter()
            .take_while(|e| !matches!(e, ScanEvent::Done))
            .filter_map(|e| match e {
                ScanEvent::Found(i) => Some(i),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn finds_all_simulated_inverters() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let found = scan(sim.port, 1, 16);
        assert_eq!(found.iter().map(|i| i.unit).collect::<Vec<_>>(), vec![1, 2, 3, 4, 5, 7, 12]);

        let mid = &found[0];
        assert_eq!(mid.serial.as_deref(), Some("SIM0001111"));
        assert_eq!(mid.firmware.as_deref(), Some("GH1.0A"));
        assert_eq!((mid.com_address, mid.baud, mid.status), (Some(1), Some(9600), Some(1)));
        assert!(mid.ac_power_w.unwrap() > 0.0);
        assert!(!mid.address_mismatch());

        // MOD TL3-XH: blank short serial, falls back to the 30-char one at 3001.
        let xh = &found[5];
        assert_eq!(xh.serial.as_deref().map(str::len), Some(30));
        assert!(xh.serial.as_deref().unwrap().starts_with("SIMXH"));

        // Faulted unit.
        assert_eq!((found[6].status, found[6].ac_power_w), (Some(3), Some(0.0)));
        assert_eq!(found[6].kind, DeviceKind::GridTie);
    }

    #[test]
    fn silent_empty_ids_time_out_and_reconnect() {
        let sim = Simulator::start(SimOptions {
            port: 0,
            empty_id_reply: EmptyIdReply::Silent,
            ..Default::default()
        })
        .unwrap();
        let found = scan(sim.port, 1, 8);
        assert_eq!(found.iter().map(|i| i.unit).collect::<Vec<_>>(), vec![1, 2, 3, 4, 5, 7]);
    }

    #[test]
    fn on_off_write_stops_production() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let mut c = ModbusTcp::new("127.0.0.1", sim.port, Duration::from_millis(200), Duration::ZERO).unwrap();
        assert_eq!(c.read(2, Table::Input, 0, 1).unwrap(), vec![1]);
        c.write_single(2, 0, 0).unwrap();
        let r = c.read(2, Table::Input, 0, 57).unwrap();
        assert_eq!((r[0], r[35], r[36]), (0, 0, 0));
        assert!(c.read(2, Table::Holding, 500, 1).is_err()); // illegal address
    }

    use crate::growatt::MeterKind;

    fn client(port: u16) -> ModbusTcp {
        ModbusTcp::new("127.0.0.1", port, Duration::from_millis(150), Duration::ZERO).unwrap()
    }

    /// Probe one ID and return the result plus how many requests it cost.
    fn probe_counted(sim: &Simulator, c: &mut ModbusTcp, unit: u8) -> (Option<DeviceInfo>, u64) {
        let before = sim.requests();
        let info = growatt::probe(c, unit).unwrap();
        (info, sim.requests() - before)
    }

    #[test]
    fn classifies_every_kind() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let found = scan(sim.port, 1, 24);
        let kinds: Vec<(u8, DeviceKind)> = found.iter().map(|i| (i.unit, i.kind)).collect();
        assert_eq!(kinds, vec![
            (1, DeviceKind::GridTie),
            (2, DeviceKind::GridTie),
            (3, DeviceKind::GridTie),
            (4, DeviceKind::Hybrid),
            (5, DeviceKind::OffGrid),
            (7, DeviceKind::Hybrid),
            (12, DeviceKind::GridTie),
            (20, DeviceKind::Meter(MeterKind::EastronSdm)),
            (21, DeviceKind::Meter(MeterKind::ChintDtsu)),
            (22, DeviceKind::Meter(MeterKind::EastronSdm)),
            (23, DeviceKind::Meter(MeterKind::ChintDdsu)),
        ]);
        for i in &found {
            assert!(!i.notes.is_empty(), "ID {} has no notes", i.unit);
            assert!(!i.status_text().is_empty());
        }
        // Grid-tie inverters carry no battery or meter data; DTC is recorded raw.
        assert!(found[0].battery.is_none() && found[0].meter.is_none());
        assert_eq!(found[0].device_type_code, Some(0x0800));
        assert_eq!(found[5].device_type_code, Some(5400));
    }

    #[test]
    fn reads_battery_soc_of_hybrid_and_off_grid() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let mut c = client(sim.port);
        for (unit, v_range) in [(4u8, 40.0..=60.0), (5, 40.0..=60.0), (7, 300.0..=500.0)] {
            let info = growatt::probe(&mut c, unit).unwrap().unwrap();
            let b = info.battery.as_ref().unwrap_or_else(|| panic!("ID {unit}: no battery"));
            let soc = b.soc_pct.unwrap();
            assert!((15.0..=85.0).contains(&soc), "ID {unit}: SOC {soc}");
            let v = b.voltage_v.unwrap();
            assert!(v_range.contains(&v), "ID {unit}: Vbat {v}");
            let (ch, dis) = (b.charge_w.unwrap(), b.discharge_w.unwrap());
            assert!(ch >= 0.0 && dis >= 0.0 && (ch == 0.0 || dis == 0.0), "ID {unit}: {ch} / {dis}");
        }
        // SPF status codes use the off-grid meanings.
        let spf = growatt::probe(&mut c, 5).unwrap().unwrap();
        assert!(matches!(spf.status, Some(2 | 5)));
        assert!(["PV charging", "Battery discharging"].contains(&spf.status_text().as_str()));
        assert_eq!(spf.health(), growatt::Health::Ok);
        assert!(spf.pv_power_w.unwrap() > 0.0 && spf.ac_power_w.unwrap() > 0.0);
    }

    #[test]
    fn meter_readings_are_plausible() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let mut c = client(sim.port);
        for unit in 20..=23u8 {
            let info = growatt::probe(&mut c, unit).unwrap().unwrap();
            assert!(info.kind.is_meter(), "ID {unit}: {:?}", info.kind);
            let m = info.meter.as_ref().unwrap();
            let v = m.voltage_v.unwrap();
            let hz = m.frequency_hz.unwrap();
            let p = m.power_w.unwrap();
            assert!((220.0..=240.0).contains(&v), "ID {unit}: V {v}");
            assert!((49.9..=50.1).contains(&hz), "ID {unit}: f {hz}");
            assert!((-2200.0..=1600.0).contains(&p) && p != 0.0, "ID {unit}: P {p}");
            assert!(m.import_kwh.unwrap() > 1000.0 && m.export_kwh.unwrap() > 100.0);
            assert_eq!(info.health(), growatt::Health::Ok);
            assert_eq!(info.status_text(), "Metering");
            assert!(info.notes.iter().any(|n| n.starts_with("holding 0–124: exception 0x02")));
        }
        let sdm = growatt::probe(&mut c, 20).unwrap().unwrap();
        assert_eq!(sdm.serial.as_deref(), Some("20000020"));
    }

    #[test]
    fn empty_id_costs_exactly_one_request() {
        for reply in [EmptyIdReply::GatewayException, EmptyIdReply::Silent] {
            let sim = Simulator::start(SimOptions { port: 0, empty_id_reply: reply, ..Default::default() }).unwrap();
            let mut c = client(sim.port);
            let (info, n) = probe_counted(&sim, &mut c, 9);
            assert!(info.is_none());
            assert_eq!(n, 1, "{reply:?}");
        }
    }

    #[test]
    fn request_budget_per_kind() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let mut c = client(sim.port);
        for (unit, expected) in [(1u8, 3u64), (4, 3), (5, 2), (7, 4), (12, 3), (20, 3), (21, 4), (22, 3), (23, 5)] {
            let (info, n) = probe_counted(&sim, &mut c, unit);
            let info = info.unwrap();
            assert_eq!(n, expected, "ID {unit} ({}): {:?}", info.kind.label(), info.notes);
        }
    }

    #[test]
    fn scan_log_names_devices_and_kinds() {
        let sim = Simulator::start(SimOptions { port: 0, ..Default::default() }).unwrap();
        let cfg = ScanConfig {
            host: "127.0.0.1".into(),
            port: sim.port,
            first: 4,
            last: 5,
            timeout: Duration::from_millis(100),
            gap: Duration::ZERO,
        };
        let rx = spawn(cfg, egui::Context::default(), Arc::new(AtomicBool::new(false)));
        let logs: Vec<String> = rx
            .iter()
            .take_while(|e| !matches!(e, ScanEvent::Done))
            .filter_map(|e| match e {
                ScanEvent::Log(_, s) => Some(s),
                _ => None,
            })
            .collect();
        assert!(logs.iter().any(|l| l == "ID 4: found Hybrid SIM0004444"), "{logs:?}");
        assert!(logs.iter().any(|l| l == "Scan complete: 2 device(s) found (1 Hybrid, 1 Off-grid)."), "{logs:?}");
    }
}
