use lib::can::ping;
use std::io::{self};

fn main() -> io::Result<()> {
    ping(None)
}
