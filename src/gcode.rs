use anyhow::Result;
use nom_gcode::{GCodeLine::*, Mnemonic};

pub enum GCode1 {
    Layer(usize),
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

pub fn parse_gcode(filename: &str) -> Result<Vec<(usize, GCode1)>> {
    let gcode = std::fs::read_to_string(filename)?;
    let mut out = Vec::new();
    for (number, line) in gcode.lines().enumerate() {
        let number = number + 1;
        let parsed = nom_gcode::parse_gcode(&line)?;
        match parsed {
            (_, Some(Comment(comment))) => {
                let prefix = "LAYER:";
                if !comment.0.starts_with(prefix) {
                    continue;
                }
                let layer_idx = comment.0[prefix.len()..].parse::<usize>()?;
                out.push((number, GCode1::Layer(layer_idx)));
            }
            (_, Some(GCode(code))) => {
                if code.mnemonic != Mnemonic::General {
                    continue;
                }

                if [0, 1, 92].contains(&code.major) {
                    out.push((number, GCode1::Coord(GCode1Coord::from_argument(code))));
                }
            }
            (_, _) => (),
        }
    }

    Ok(out)
}
