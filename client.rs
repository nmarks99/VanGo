use std::io::{self, Write};
use std::net::TcpStream;
use std::process::Command;

fn clear_screen() {
    if cfg!(target_os = "windows") {
        let _ = Command::new("cls").status();
    } else {
        let _ = Command::new("clear").status();
    }
}

fn main() {

    let mut stream = match TcpStream::connect("192.168.4.1:80") {
        Ok(stream) => {
            println!("Connected to ESP32!");
            stream
        }
        Err(err) => {
            eprintln!("Failed to connect: {}", err);
            return;
        }
    };

    clear_screen();

    loop {
        let mut val = String::new();
        print!("> ");
        io::stdout().flush().unwrap();
        io::stdin().read_line(&mut val).unwrap();

        let cmd = match val.trim() {
            "h" => "GET /H",
            "l" => "GET /L",
            _ => {
                println!("unknown command {}", val.trim());
                continue;
            }
        };

        if let Err(err) = stream.write_all(cmd.as_bytes()) {
            eprintln!("Failed to send command: {}", err);
            break;
        }

        clear_screen();
    }

    println!("Socket closed");
}

