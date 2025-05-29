use nom::{
    IResult,
    character::complete::*,
    bytes::complete::*,
};
use nom::branch::*;
use nom::combinator::*;
use nom::sequence::*;
use nom::multi::*;
use super::{
    Comment,
};

// #[inline(always)]
pub fn parentheses_comment<'r>(input: &'r str) -> IResult<&'r str, &'r str> {
    let parser = preceded(
        char('('),
        is_not("\n\r)"),
    );

    terminated(
        parser,
        char(')'),
    )(input)
}

pub struct WithComments<'r, O> {
    pub comments: Option<Vec<&'r str>>,
    pub value: O,
}

// #[inline(always)]
pub fn with_parentheses_comments<
    'r,
    T: FnMut(&'r str,) -> IResult<&'r str, O>,
    O,
>(
    parser: T,
) -> impl FnMut(&'r str,) -> IResult<&'r str, WithComments<'r, O>> {
    let parser = pair(
        parser,
        opt(many1(parentheses_comment)),
    );

    let parser = pair(
        opt(many1(parentheses_comment)),
        parser,
    );

    map(
        parser,
        |(mut comments, (value, more_comments))| {
            // Merge all comments into one optional vec
            if let Some(more_comments) = more_comments {
                comments = Some([
                    comments.unwrap_or_else(|| vec![]),
                    more_comments,
                ].concat())
            }

            WithComments {
                comments,
                value,
            }
        }
    )
}

// #[inline(always)]
pub fn seimcolon_comment<'r>(input: &'r str,) -> IResult<&'r str, &'r str> {
    preceded(
        char(';'),
        not_line_ending,
    )(input)
}

// #[inline(always)]
pub fn comment<'r>(input: &'r str) -> IResult<&'r str, Comment<'r>> {
    map(
        alt((seimcolon_comment, parentheses_comment)),
        |comment| Comment(comment),
    )(input)
}

