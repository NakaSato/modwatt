//! Energy meters that Growatt sells rebadged and that sit on the same RS485 bus.
//!
//! | Growatt | Meter              | Phases | Default ID | Map below      |
//! |---------|--------------------|--------|------------|----------------|
//! | SPM-E   | Eastron SDM120     | 1      | 1          | Eastron        |
//! | TPM-E   | Eastron SDM630     | 3      | 2          | Eastron        |
//! | SPM-C   | Chint DDSU666      | 1      | 3          | Chint DDSU     |
//! | TPM-C   | Chint DTSU666      | 3      | 4          | Chint DTSU     |
//!
//! All values are IEEE754 float32, high word first ("ABCD").
//!
//! **Eastron SDM** (FC 0x04 input registers).
//! Source: volkszaehler/mbmd `meters/rs485/sdm.go`
//! (<https://github.com/volkszaehler/mbmd/blob/master/meters/rs485/sdm.go>), which is verified for:
//!   0x0000/2/4 V L1/L2/L3, 0x000C P L1 (W), 0x0034 total system power (W),
//!   0x0046 frequency (Hz), 0x0048 import kWh, 0x004A export kWh.
//! Unverified, from memory: the SDM120 lacks 0x0034 (we use 0x000C instead),
//! and the serial number is a u32 at holding 0xFC00.
//!
//! **Chint DTSU666** (FC 0x03 holding registers).
//! Source: mbmd `meters/rs485/dtsu666.go`
//! (<https://github.com/volkszaehler/mbmd/blob/master/meters/rs485/dtsu666.go>), which is verified for:
//!   0x2006 Ua (×0.1 V), 0x2012 Pt (×0.1 W), 0x2044 Freq (×0.01 Hz),
//!   0x101E ImpEp (kWh), 0x1028 ExpEp (kWh).
//!   The sign convention (positive = import) is unverified.
//!
//! **Chint DDSU666** (FC 0x03 holding registers).
//! Source: helixzz/ha-modbus-powermeter `custom_components/ddsu666/const.py`
//! (<https://github.com/helixzz/ha-modbus-powermeter>), which is verified for:
//!   0x2000 U (V), 0x2002 I (A), 0x2004 P (**kW**, ×1000 → W), 0x200E Freq (Hz),
//!   0x4000 import kWh.
//! Unverified: 0x400A export kWh, and the sign of P.

use crate::growatt::{MeterInfo, MeterKind};
use crate::modbus::{ModbusError, ModbusTcp, Table};

pub struct MeterReading {
    pub kind: MeterKind,
    pub info: MeterInfo,
    pub serial: Option<String>,
}

/// Try the known meter signatures, in order Eastron, then Chint (DTSU, then DDSU).
///
/// Every step appends a fingerprint line to `notes`. Only a `Connect` error
/// is returned as `Err`; any other failure just means "not this meter".
pub fn probe(client: &mut ModbusTcp, unit: u8, notes: &mut Vec<String>) -> Result<Option<MeterReading>, ModbusError> {
    if let Some(m) = probe_eastron(client, unit, notes)? {
        return Ok(Some(m));
    }
    probe_chint(client, unit, notes)
}

fn probe_eastron(client: &mut ModbusTcp, unit: u8, notes: &mut Vec<String>) -> Result<Option<MeterReading>, ModbusError> {
    // One read covers V L1..L3 (0x00..0x05), P L1 (0x0C), total P (0x34), Hz, import, export (0x46..0x4B).
    let mut info = MeterInfo::default();
    let mut phases = "";
    match client.read(unit, Table::Input, 0x0000, 0x4C) {
        Err(e @ ModbusError::Connect(_)) => return Err(e),
        // Illegal function / address at 0x0000: FC 0x04 or that range is not
        // implemented at all, so this is not an Eastron.
        Err(e @ ModbusError::Exception(0x01 | 0x02)) => {
            notes.push(format!("input 0x0000×76 (SDM): {}", short_err(&e)));
            return Ok(None);
        }
        Ok(r) => {
            notes.push("input 0x0000×76 (SDM): ok".into());
            let (v1, v2, v3) = (f32_at(&r, 0x00), f32_at(&r, 0x02), f32_at(&r, 0x04));
            let hz = f32_at(&r, 0x46);
            if !(plausible_voltage(v1) && plausible_hz(hz)) {
                notes.push(format!("SDM: implausible V {v1:.1} / f {hz:.2}"));
                return Ok(None);
            }
            let single_phase = v2.abs() < 1.0 && v3.abs() < 1.0;
            let total = f32_at(&r, 0x34);
            let p = if single_phase || !total.is_finite() { f32_at(&r, 0x0C) } else { total };
            phases = if single_phase { "1-phase (SDM120-like)" } else { "3-phase (SDM630-like)" };
            info = MeterInfo {
                voltage_v: Some(v1),
                power_w: Some(p),
                frequency_hz: Some(hz),
                import_kwh: finite(f32_at(&r, 0x48)),
                export_kwh: finite(f32_at(&r, 0x4A)),
            };
        }
        Err(e) => {
            // Some SDM variants may reject a long read (e.g. 0x03 "too many
            // registers") or time out on it; retry with the individual blocks.
            // 0x0034 is skipped (not on SDM120).
            notes.push(format!("input 0x0000×76 (SDM): {}", short_err(&e)));
            let Some(v) = read(client, unit, Table::Input, 0x0000, 2, "input 0x0000×2 (SDM V)", notes)? else {
                return Ok(None);
            };
            let v1 = f32_at(&v, 0);
            if !plausible_voltage(v1) {
                notes.push(format!("SDM: implausible V {v1:.1}"));
                return Ok(None);
            }
            info.voltage_v = Some(v1);
            if let Some(p) = read(client, unit, Table::Input, 0x000C, 2, "input 0x000C×2 (SDM P)", notes)? {
                info.power_w = finite(f32_at(&p, 0));
            }
            if let Some(f) = read(client, unit, Table::Input, 0x0046, 6, "input 0x0046×6 (SDM f/E)", notes)? {
                info.frequency_hz = finite(f32_at(&f, 0)).filter(|hz| plausible_hz(*hz));
                info.import_kwh = finite(f32_at(&f, 2));
                info.export_kwh = finite(f32_at(&f, 4));
            }
        }
    }
    let serial = read(client, unit, Table::Holding, 0xFC00, 2, "holding 0xFC00×2 (SDM serial)", notes)?
        .map(|r| (((r[0] as u32) << 16) | r[1] as u32).to_string())
        .filter(|s| s != "0");
    notes.push(format!("Eastron SDM {phases}").trim_end().to_string());
    Ok(Some(MeterReading { kind: MeterKind::EastronSdm, info, serial }))
}

fn probe_chint(client: &mut ModbusTcp, unit: u8, notes: &mut Vec<String>) -> Result<Option<MeterReading>, ModbusError> {
    // DTSU666 has 0x2000..=0x2045 contiguous. A DDSU666's map ends near 0x200F,
    // so the long read usually fails there and the short one follows.
    let long = read(client, unit, Table::Holding, 0x2000, 0x46, "holding 0x2000×70 (Chint)", notes)?;
    let block = match long {
        Some(r) => r,
        None => match read(client, unit, Table::Holding, 0x2000, 0x10, "holding 0x2000×16 (DDSU)", notes)? {
            Some(r) => r,
            None => return Ok(None),
        },
    };

    // DDSU666: U at 0x2000 in V and frequency at 0x200E in Hz. On a DTSU,
    // 0x2000 is Uab ×0.1 V (thousands) and 0x200E is the L2 current.
    let (u, hz) = (f32_at(&block, 0x00), f32_at(&block, 0x0E));
    if plausible_voltage(u) && plausible_hz(hz) {
        let mut info = MeterInfo {
            voltage_v: Some(u),
            power_w: finite(f32_at(&block, 0x04) * 1000.0),
            frequency_hz: Some(hz),
            ..Default::default()
        };
        if let Some(e) = read(client, unit, Table::Holding, 0x4000, 12, "holding 0x4000×12 (DDSU energy)", notes)? {
            info.import_kwh = finite(f32_at(&e, 0x00));
            info.export_kwh = finite(f32_at(&e, 0x0A));
        }
        notes.push("Chint DDSU666 (1-phase)".into());
        return Ok(Some(MeterReading { kind: MeterKind::ChintDdsu, info, serial: None }));
    }

    if block.len() >= 0x46 {
        let ua = f32_at(&block, 0x06) / 10.0;
        let hz = f32_at(&block, 0x44) / 100.0;
        if plausible_voltage(ua) && plausible_hz(hz) {
            let mut info = MeterInfo {
                voltage_v: Some(ua),
                power_w: finite(f32_at(&block, 0x12) / 10.0),
                frequency_hz: Some(hz),
                ..Default::default()
            };
            if let Some(e) = read(client, unit, Table::Holding, 0x101E, 12, "holding 0x101E×12 (DTSU energy)", notes)? {
                info.import_kwh = finite(f32_at(&e, 0x00));
                info.export_kwh = finite(f32_at(&e, 0x0A));
            }
            notes.push("Chint DTSU666 (3-phase)".into());
            return Ok(Some(MeterReading { kind: MeterKind::ChintDtsu, info, serial: None }));
        }
        notes.push(format!("Chint: implausible Ua {ua:.1} / f {hz:.2}"));
    } else {
        notes.push(format!("Chint: implausible U {u:.1} / f {hz:.2}"));
    }
    Ok(None)
}

/// Read and record the outcome in `notes`. Returns `Ok(None)` on any
/// non-connect failure.
fn read(
    client: &mut ModbusTcp,
    unit: u8,
    table: Table,
    start: u16,
    count: u16,
    what: &str,
    notes: &mut Vec<String>,
) -> Result<Option<Vec<u16>>, ModbusError> {
    match client.read(unit, table, start, count) {
        Ok(r) => {
            notes.push(format!("{what}: ok"));
            Ok(Some(r))
        }
        Err(e @ ModbusError::Connect(_)) => Err(e),
        Err(e) => {
            notes.push(format!("{what}: {}", short_err(&e)));
            Ok(None)
        }
    }
}

pub fn short_err(e: &ModbusError) -> String {
    match e {
        ModbusError::Exception(c) => format!("exception 0x{c:02X}"),
        ModbusError::Io(_) => "no reply".into(),
        other => other.to_string(),
    }
}

/// Float32 from two registers, high word first.
pub fn f32_at(regs: &[u16], i: usize) -> f64 {
    f32::from_bits(((regs[i] as u32) << 16) | regs[i + 1] as u32) as f64
}

fn finite(v: f64) -> Option<f64> {
    Some(v).filter(|v| v.is_finite())
}

pub fn plausible_voltage(v: f64) -> bool {
    (80.0..=300.0).contains(&v)
}

pub fn plausible_hz(f: f64) -> bool {
    (45.0..=65.0).contains(&f)
}
