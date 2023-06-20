use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;
use std::io::Write;
use std::net::TcpStream;

fn main() {
    
    
    // connect to ESP32 over WiFi
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

    let stdin = std::io::stdin();
    let mut stdout = std::io::stdout().into_raw_mode().unwrap();
    
    // clear the screen, move and hide cursor
    write!(stdout,"{} {} {}",
        termion::clear::All,
        termion::cursor::Goto(1, 1),
        termion::cursor::Hide
    ).unwrap();

    stdout.flush().unwrap();


    // blocking, exit with q
    for c in stdin.keys() {
        
        write!(stdout,"{} {}",
            termion::clear::All,
            termion::cursor::Goto(1, 1)
        ).unwrap();
        
        let cmd: Option<&str> = match c.unwrap() {
            Key::Up => {
                println!("ON");
                Some("UP")
            },

            Key::Down => {
                println!("OFF");
                Some("DOWN")
            },

            Key::Char('q') => break,

            _ => None

        };
        
        if cmd.is_some() {
            if let Err(err) = stream.write_all(cmd.unwrap().as_bytes()) {
                eprintln!("Write to TcpStream failed, {}", err);
            }
        }

        stdout.flush().unwrap();
    }
    
    // clear screen, move and show cursor at the end
    write!(stdout,"{} {} {}",
        termion::clear::All,
        termion::cursor::Goto(1, 1),
        termion::cursor::Show
    ).unwrap();
}


