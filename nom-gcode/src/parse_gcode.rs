use nom::branch::*;
use nom::combinator::*;
use nom::sequence::*;
use nom::{character::complete::*, error::VerboseError};

use super::{
    comment,
    parse_args,
    parse_command,
    // GCode,
    GCodeLine,
    GCodeParseError,
    GCodeParseError::*,
    M,
};

const STRING_ARG_MCODES: [u32; 7] = [
    23, 28, 30, // 32, Unsupported: Marlin M32 is a whole different kind of weird
    36, // M37 Unsupported: RepRap M37 is a whole different kind of weird
    38, 117, 118,
];

// #[inline(always)]
pub fn parse_gcode(input: &str) -> Result<(&str, Option<GCodeLine>), GCodeParseError> {
    let original_input = input;
    let demarcator = map(pair(char('%'), not_line_ending), |_: (char, &str)| {
        GCodeLine::FileDemarcator
    });

    // Strip leading whitespace
    let (input, _) = space0::<_, VerboseError<&str>>(input)
        .map_err(|_| InvalidGCode(original_input.to_string()))?;

    // empty lines without a newline character
    if input.is_empty() {
        return Ok((input, None));
    };

    // Parse and return a non-gcode (empty line, demarcator or comment)
    let mut non_gcode_line = alt((
        // Empty line
        map(newline, |_| None),
        map(
            alt((
                // Demarcator (eg. "%")
                demarcator,
                // Comment Line (eg. "; Comment")
                map(comment, |comment| GCodeLine::Comment(comment)),
            )),
            |line| Some(line),
        ),
    ));

    if let Ok((input, gcode_line)) = non_gcode_line(input) {
        return Ok((input, gcode_line));
    };

    /*
     * Parse the GCode command (eg. this would parse "G1" out of "G1 X10")
     */

    if let Ok((input, mut gcode)) = parse_command(input) {
        /*
         * Parse the GCode args (eg. this would parse "X10" out of "G1 X10")
         */

        let string_arg_mcode =
            gcode.mnemonic == M && gcode.minor == 0 && STRING_ARG_MCODES.contains(&gcode.major);

        let (input, args_or_comments) = parse_args(string_arg_mcode, input)
            .map_err(|_| InvalidArguments(original_input.to_string()))?;

        gcode.args_or_comments = args_or_comments;
        Ok((input, Some(GCodeLine::GCode(gcode))))
    } else {
        Ok((input, Some(GCodeLine::GCodeMacro(input))))
    }
}
