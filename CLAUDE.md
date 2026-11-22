# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`modwatt` is a native Rust GUI (eframe/egui 0.37) that scans the Growatt ecosystem (grid-tie, hybrid and off-grid inverters with their batteries, plus Eastron/Chint meters) sitting on an RS485 bus behind a Modbus TCP gateway (ShineMaster or a generic RS485-to-Ethernet converter). The computer sends Modbus TCP to the gateway on port 502. The gateway forwards each request onto RS485, with the MBAP unit ID used as the RTU slave address.

## Commands

```sh
cargo run                         # GUI
cargo run -- --sim 5020           # headless simulated gateway only (for mbpoll, scripts, etc.)
cargo test                        # all tests (they spin up the simulator on port 0; no hardware needed)
cargo test finds_all_simulated    # single test by name substring
cargo clippy
```

To render the GUI to a PNG without screen-recording permission, use the dev-only `screenshot` feature. It switches to the glow renderer, because eframe's screenshot hook is unimplemented for wgpu.

```sh
EFRAME_SCREENSHOT_TO=out.png MODWATT_DEMO=1 [MODWATT_THEME=light] [MODWATT_SELECT=1] cargo run --features screenshot
```

The screenshot captures exactly egui pass #2. `main.rs` therefore forces `max_passes = 1` in this mode; otherwise the capture can land on an invisible measuring pass and come out blank. `MODWATT_DEMO` scans a simulator bound to a free port before the first frame.

## egui 0.37 API notes

This version differs from most online examples:
- `eframe::App` implements `fn ui(&mut self, ui: &mut egui::Ui, frame)`, not `update(ctx, …)`.
- Panels are `egui::Panel::{top,bottom,left,right}(id)…show(ui, …)` and take `&mut Ui`.
- Sizes use `default_size`/`exact_size`.
- `Margin` fields are `i8`, and `CornerRadius` replaces `Rounding`.
- Theme styles are set per theme with `ctx.style_mut_of(Theme::Dark|Light, …)`.

Some glyphs (e.g. `✕`, `▶`) render as tofu or colour emoji in the default fonts; use `×` or plain text instead.

## Architecture

Data flows: `app` (UI thread) → `scanner` / `discovery` (background threads) → `modbus` client → TCP → gateway or `sim`.

- **`modbus.rs`**: blocking Modbus TCP client over `std::net`, with no async runtime.
  - **Request path:** every request goes through `transact()`. It enforces a minimum inter-request `gap`, because RS485 is half-duplex and Growatt wants roughly 850–1000 ms between requests.
  - **Reconnect:** after any I/O or protocol error the socket is dropped and lazily reopened. This means a late reply from a slow slave can never be read as the answer to a later request.
  - **Stale replies:** a mismatched transaction ID is skipped.
  - **Errors:** `ModbusError::Connect` means the gateway itself is unreachable, so scans abort. Exceptions `0x0A` and `0x0B` mean "nobody answered on the serial side".
- **`growatt.rs`**: register maps and the `probe()` presence and identification logic. Returns `DeviceInfo` with a `DeviceKind` (GridTie / Hybrid / OffGrid / Meter / Unknown), optional `battery` / `meter` data and `notes` (probe fingerprint lines, the main clue for Unknown devices).
  - **Classification:** by register signatures, not by trusting the DTC (holding 43) table. SPF layout at input 17/18/21; storage blocks at input 1000+ (SPH/SPA) or 3169+ (TL-XH/WIT). Each map's doc comment states its source and which parts are unverified.
  - **Request budget:** an empty ID must cost exactly 1 request; `request_budget_per_kind` in `scanner.rs` pins the per-kind counts.
  - **Presence:** holding registers 0–124 are read in one request. Any non-gateway Modbus exception still counts as "device present".
  - **Serial number:** falls back from holding 23–27 (10 chars) to 3001–3015, then 133–147 (30 chars), for newer models.
  - **Address check:** holding register 30 is the inverter's own RS485 address. A mismatch with the polled ID means the gateway is likely ignoring the unit ID.
  - **Telemetry:** read from input registers 0–56. U32 values are high word first, with 0.1 scaling.
- **`meter.rs`**: Eastron SDM120/SDM630 (FC04 floats) and Chint DTSU666/DDSU666 (FC03 floats) — Growatt's SPM/TPM meters. Tried when a device rejects or doesn't look like the Growatt base map.
- **`scanner.rs`**: one background thread sweeps slave IDs strictly sequentially over a single connection. It reports to the UI via `mpsc` `ScanEvent`s and calls `ctx.request_repaint()`. Cancellation uses an `AtomicBool`. The integration tests that run the scanner against the simulator live here.
- **`discovery.rs`**: finds gateways on the LAN for users whose gateway gets a DHCP address from the router.
  - **Local subnet:** derived from a UDP `connect` that sends no packets.
  - **Sweep:** parallel TCP connects to `.1–.254:port`, then one Modbus read per open port to classify it as an inverter, a gateway, or just an open port.
- **`sim.rs`**: an in-process fake gateway that implements the same register map, used by tests, the GUI "Simulator" source and `--sim`.
  - **Serialized bus:** one `Mutex` serializes the simulated RS485 bus.
  - **Empty IDs:** reply with `0x0B` or stay silent.
  - **Timing:** optional 9600-baud reply timing.
  - **Models:** `Model::ALL` = Mid, ModXh, Sph, Spf, Sdm630, Dtsu666, Sdm120, Ddsu666. `ModXh` leaves 23–27 blank to exercise the long-serial fallback. Meters reject holding 0–124 with 0x02, like real ones.
  - **Writes:** FC 0x06 to holding 0 (on/off) and 3 (power limit) is supported.

  **Keep `sim.rs` in sync with `growatt.rs`** when adding registers.
- **`app.rs`**: all UI state and views.
  - **Background work:** results come back on `Receiver`s that `poll()` drains each frame.
  - **Simulator source:** scanning in Simulator mode auto-starts the simulator and targets `127.0.0.1:<sim port>` without overwriting the user's gateway IP.
  - **Layout:** header, bottom status/activity bar, left settings sidebar with a pinned Scan button, optional right details panel for the selected row, and a central stats-plus-table view.
- **`theme.rs`**: light and dark `Palette`s, applied to egui styles, plus small widget helpers (`segmented`, `primary_button`, `dot_label`, `field`, …). Views take colours from `theme::palette(ui)` rather than hard-coding them.
