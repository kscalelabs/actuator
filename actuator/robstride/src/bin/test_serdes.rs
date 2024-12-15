use robstride::{CommandData, ObtainIDCommand};

fn main() {
    let cmd = ObtainIDCommand { host_id: 0xFE };
    let serialized = cmd.to_can_packet(0x01);

    println!("{:?}, {:x} {:02x?}", cmd, serialized.0, serialized.1);
}
