//! Visual theme: a neutral, low-contrast palette per light/dark mode, applied
//! to egui's styles, plus a few small reusable widgets.

use eframe::egui::{
    self, Color32, CornerRadius, FontFamily, FontId, Frame, Margin, RichText, Stroke, TextStyle, Theme, Ui,
};

#[derive(Clone, Copy)]
pub struct Palette {
    /// Main content background.
    pub bg: Color32,
    /// Sidebar / chrome background.
    pub chrome: Color32,
    /// Inputs, buttons, segmented controls.
    pub surface: Color32,
    pub hover: Color32,
    pub border: Color32,
    pub text: Color32,
    pub muted: Color32,
    pub faint: Color32,
    pub accent: Color32,
    pub success: Color32,
    pub warn: Color32,
    pub danger: Color32,
}

const fn hex(rgb: u32) -> Color32 {
    Color32::from_rgb((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
}

pub const DARK: Palette = Palette {
    bg: hex(0x0A0A0B),
    chrome: hex(0x101012),
    surface: hex(0x1A1A1D),
    hover: hex(0x242428),
    border: hex(0x232327),
    text: hex(0xF4F4F5),
    muted: hex(0xA1A1AA),
    faint: hex(0x5F5F67),
    accent: hex(0xF59E0B),
    success: hex(0x34D399),
    warn: hex(0xFBBF24),
    danger: hex(0xF87171),
};

pub const LIGHT: Palette = Palette {
    bg: hex(0xFFFFFF),
    chrome: hex(0xFAFAFA),
    surface: hex(0xF4F4F5),
    hover: hex(0xE9E9EC),
    border: hex(0xE7E7EA),
    text: hex(0x18181B),
    muted: hex(0x71717A),
    faint: hex(0xA1A1AA),
    accent: hex(0xD97706),
    success: hex(0x059669),
    warn: hex(0xB45309),
    danger: hex(0xDC2626),
};

pub fn palette(ui: &Ui) -> Palette {
    if ui.visuals().dark_mode { DARK } else { LIGHT }
}

pub fn install(ctx: &egui::Context) {
    ctx.style_mut_of(Theme::Dark, |s| style(s, &DARK, true));
    ctx.style_mut_of(Theme::Light, |s| style(s, &LIGHT, false));
}

fn style(s: &mut egui::Style, p: &Palette, dark: bool) {
    s.visuals = if dark { egui::Visuals::dark() } else { egui::Visuals::light() };
    let v = &mut s.visuals;
    let r = CornerRadius::same(8);

    v.panel_fill = p.bg;
    v.window_fill = p.bg;
    v.window_stroke = Stroke::new(1.0, p.border);
    v.window_corner_radius = CornerRadius::same(12);
    v.menu_corner_radius = CornerRadius::same(10);
    v.extreme_bg_color = p.surface;
    v.faint_bg_color = p.chrome;
    v.code_bg_color = p.surface;
    v.warn_fg_color = p.warn;
    v.error_fg_color = p.danger;
    v.hyperlink_color = p.text;
    v.selection.bg_fill = p.hover;
    v.selection.stroke = Stroke::new(1.0, p.text);

    let w = &mut v.widgets;
    for (wv, fill, stroke) in [
        (&mut w.noninteractive, p.bg, p.border),
        (&mut w.inactive, p.surface, p.surface),
        (&mut w.hovered, p.hover, p.hover),
        (&mut w.active, p.hover, p.faint),
        (&mut w.open, p.hover, p.border),
    ] {
        wv.bg_fill = fill;
        wv.weak_bg_fill = fill;
        wv.bg_stroke = Stroke::new(1.0, stroke);
        wv.fg_stroke = Stroke::new(1.0, p.text);
        wv.corner_radius = r;
        wv.expansion = 0.0;
    }

    let sp = &mut s.spacing;
    sp.item_spacing = egui::vec2(8.0, 10.0);
    sp.button_padding = egui::vec2(12.0, 6.0);
    sp.interact_size.y = 30.0;

    use FontFamily::{Monospace, Proportional};
    s.text_styles = [
        (TextStyle::Small, FontId::new(12.0, Proportional)),
        (TextStyle::Body, FontId::new(14.0, Proportional)),
        (TextStyle::Button, FontId::new(14.0, Proportional)),
        (TextStyle::Heading, FontId::new(22.0, Proportional)),
        (TextStyle::Monospace, FontId::new(13.0, Monospace)),
    ]
    .into();
}

// --- Small building blocks --------------------------------------------------

/// Soft card with a hairline border.
pub fn card(p: &Palette) -> Frame {
    Frame::new()
        .fill(p.chrome)
        .stroke(Stroke::new(1.0, p.border))
        .corner_radius(12)
        .inner_margin(Margin::same(16))
}

/// Muted helper text.
pub fn hint(ui: &mut Ui, text: impl Into<String>) {
    ui.label(RichText::new(text.into()).small().color(palette(ui).muted));
}

/// Coloured dot followed by text, sized to content in any layout direction.
pub fn dot_label(ui: &mut Ui, text: &str, dot: Color32, text_color: Color32) -> egui::Response {
    use egui::text::{LayoutJob, TextFormat};
    let font = egui::TextStyle::Body.resolve(ui.style());
    let mut job = LayoutJob::default();
    job.append("● ", 0.0, TextFormat::simple(FontId::new(font.size * 0.75, font.family.clone()), dot));
    job.append(text, 0.0, TextFormat::simple(font, text_color));
    ui.label(job)
}

/// Solid high-contrast primary button (text colour on background colour).
pub fn primary_button(ui: &mut Ui, text: &str, width: f32, enabled: bool) -> egui::Response {
    let p = palette(ui);
    let (fill, fg) = if enabled { (p.text, p.bg) } else { (p.surface, p.faint) };
    ui.add_enabled_ui(enabled, |ui| {
        ui.add_sized(
            [width, 40.0],
            egui::Button::new(RichText::new(text).strong().color(fg)).fill(fill).stroke(Stroke::NONE).corner_radius(10),
        )
    })
    .inner
}

/// Quiet secondary button.
pub fn secondary_button(ui: &mut Ui, text: &str, width: f32) -> egui::Response {
    let p = palette(ui);
    ui.add_sized(
        [width, 40.0],
        egui::Button::new(RichText::new(text).strong().color(p.text)).fill(p.surface).stroke(Stroke::NONE).corner_radius(10),
    )
}

/// Text-only button.
pub fn text_button(ui: &mut Ui, text: &str, color: Color32) -> egui::Response {
    ui.add(egui::Button::new(RichText::new(text).color(color)).frame(false))
        .on_hover_cursor(egui::CursorIcon::PointingHand)
}

/// Segmented control over a small set of values. Returns true when changed.
pub fn segmented<T: PartialEq + Copy>(ui: &mut Ui, value: &mut T, options: &[(T, &str)]) -> bool {
    let p = palette(ui);
    let mut changed = false;
    Frame::new()
        .fill(p.surface)
        .corner_radius(10)
        .inner_margin(Margin::same(3))
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            let n = options.len() as f32;
            let w = (ui.available_width() - 2.0 * (n - 1.0)) / n;
            ui.horizontal(|ui| {
                for &(opt, label) in options {
                    let sel = *value == opt;
                    let btn = egui::Button::new(RichText::new(label).color(if sel { p.text } else { p.muted }))
                        .fill(if sel { p.bg } else { Color32::TRANSPARENT })
                        .stroke(if sel { Stroke::new(1.0, p.border) } else { Stroke::NONE })
                        .corner_radius(8)
                        .min_size(egui::vec2(w, 28.0));
                    if ui.add(btn).clicked() && !sel {
                        *value = opt;
                        changed = true;
                    }
                }
            });
        });
    changed
}

/// Labelled form field: muted label above the widget.
pub fn field<R>(ui: &mut Ui, label: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    ui.vertical(|ui| {
        ui.spacing_mut().item_spacing.y = 6.0;
        ui.label(RichText::new(label).small().color(palette(ui).muted));
        add(ui)
    })
    .inner
}

/// Small heading that separates groups of fields in a side panel.
pub fn section(ui: &mut Ui, title: &str) {
    let p = palette(ui);
    ui.add_space(6.0);
    ui.separator();
    ui.label(RichText::new(title).small().strong().color(p.muted));
}
