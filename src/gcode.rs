use anyhow::Result;
use nalgebra::Vector3;
use nom_gcode::{GCodeLine::*, Mnemonic};

pub enum GCode1 {
    Layer(usize),
    TypedComment(String, String),
    Coord(GCode1Coord),
}

#[derive(Default, Debug, Clone, Copy)]
pub struct GCode1Coord {
    pub major: u32,
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
    pub e: Option<f32>,
    pub f: Option<f32>,
}

impl GCode1Coord {
    fn from_argument<'a>(code: nom_gcode::GCode<'a>) -> Self {
        let mut out = Self::default();
        out.major = code.major;
        for (letter, value) in code.arguments() {
            let letter = *letter;
            let v = match value {
                Some(v) => *v,
                None => todo!(),
            };

            if letter == 'X' {
                out.x = Some(v);
            }
            if letter == 'Y' {
                out.y = Some(v);
            }
            if letter == 'Z' {
                out.z = Some(v);
            }
            if letter == 'E' {
                out.e = Some(v);
            }
            if letter == 'F' {
                out.f = Some(v);
            }
        }
        out
    }

    pub fn apply(&self, other: &Self) -> Self {
        let mut out = self.clone();
        if other.x.is_some() {
            out.x = other.x;
        }
        if other.y.is_some() {
            out.y = other.y;
        }
        if other.z.is_some() {
            out.z = other.z;
        }
        if other.e.is_some() {
            out.e = other.e;
        }
        if other.f.is_some() {
            out.f = other.f;
        }
        out
    }
}

const PREFIX_LAYER: &'static str = "LAYER:";

pub fn parse_gcode(filename: &str) -> Result<Vec<(usize, GCode1)>> {
    let gcode = std::fs::read_to_string(filename)?;
    let mut out = Vec::new();
    for (number, line) in gcode.lines().enumerate() {
        let number = number + 1;
        let parsed = nom_gcode::parse_gcode(&line)?;
        match parsed {
            (_, Some(Comment(comment))) => {
                if comment.0.starts_with(PREFIX_LAYER) {
                    let layer_idx = comment.0[PREFIX_LAYER.len()..].parse::<usize>()?;
                    out.push((number, GCode1::Layer(layer_idx)));
                } else {
                    let mut parts = comment.0.splitn(2, ':');
                    let prefix = parts.next().unwrap_or("");
                    let value = parts.next().unwrap_or("");
                    out.push((
                        number,
                        GCode1::TypedComment(prefix.to_string(), value.to_string()),
                    ));
                }
            }
            (_, Some(GCode(code))) => {
                if code.mnemonic == Mnemonic::General && [0, 1, 2, 3, 92].contains(&code.major) {
                    out.push((number, GCode1::Coord(GCode1Coord::from_argument(code))));
                }
            }
            (_, _) => (),
        }
    }

    Ok(out)
}

#[derive(Debug)]
pub struct GCodeMeta {
    pub flavor: Option<String>,
    pub time: Option<f32>,
    pub filament_used: Option<f32>,
    pub layer_height: Option<f32>,
    pub bounding_box: Option<(Vector3<f32>, Vector3<f32>)>,
    pub target_machine: Option<String>,
    pub layer_count: Option<usize>,
}

impl GCodeMeta {
    pub fn from_comments(comments: &[(&str, &str)]) -> Self {
        let mut time = None;
        let mut flavor = None;
        let mut filament_used = None;
        let mut layer_height = None;
        let mut target_machine = None;
        let mut layer_count = None;

        let mut minx = None;
        let mut miny = None;
        let mut minz = None;
        let mut maxx = None;
        let mut maxy = None;
        let mut maxz = None;

        for (prefix, value) in comments {
            match *prefix {
                "FLAVOR" => {
                    if !value.is_empty() {
                        flavor = Some(value.to_string());
                    }
                }
                "TIME" => {
                    if let Ok(v) = value.parse::<f32>() {
                        time = Some(v);
                    }
                }
                "Filament used" => {
                    if value.trim().ends_with("m") {
                        let value = value.trim().trim_end_matches('m');
                        if let Ok(v) = value.parse::<f32>() {
                            // convert to mm
                            filament_used = Some(v * 1000.0);
                        }
                    }
                }
                "Layer height" => {
                    if let Ok(v) = value.trim().parse::<f32>() {
                        layer_height = Some(v);
                    }
                }
                "MINX" => {
                    if let Ok(v) = value.parse::<f32>() {
                        minx = Some(v);
                    }
                }
                "MINY" => {
                    if let Ok(v) = value.parse::<f32>() {
                        miny = Some(v);
                    }
                }
                "MINZ" => {
                    if let Ok(v) = value.parse::<f32>() {
                        minz = Some(v);
                    }
                }
                "MAXX" => {
                    if let Ok(v) = value.parse::<f32>() {
                        maxx = Some(v);
                    }
                }
                "MAXY" => {
                    if let Ok(v) = value.parse::<f32>() {
                        maxy = Some(v);
                    }
                }
                "MAXZ" => {
                    if let Ok(v) = value.parse::<f32>() {
                        maxz = Some(v);
                    }
                }
                "TARGET_MACHINE.NAME" => {
                    target_machine = Some(value.to_string());
                }
                "LAYER_COUNT" => {
                    if let Ok(v) = value.parse::<usize>() {
                        layer_count = Some(v);
                    }
                }
                _ => {}
            }
        }

        let bounding_box =
            if let (Some(minx), Some(miny), Some(minz), Some(maxx), Some(maxy), Some(maxz)) =
                (minx, miny, minz, maxx, maxy, maxz)
            {
                Some((
                    Vector3::new(minx, miny, minz),
                    Vector3::new(maxx, maxy, maxz),
                ))
            } else {
                None
            };

        Self {
            flavor,
            time,
            filament_used,
            layer_height,
            bounding_box,
            target_machine,
            layer_count,
        }
    }
}
