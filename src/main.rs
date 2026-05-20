use serialport::{SerialPort, SerialPortType};
use std::env;
use std::fs;
use std::io;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const BAUD_RATE: u32 = 9_600;
const DEFAULT_INTERVAL: Duration = Duration::from_millis(250);
const DEFAULT_RETRY_INTERVAL: Duration = Duration::from_secs(5);
const MAX_NETWORK_MBPS: f32 = 100.0;
const DEFAULT_BY_ID_DIR: &str = "/dev/serial/by-id";

#[derive(Debug)]
struct Options {
    port: Option<String>,
    interval: Duration,
    retry_interval: Duration,
    verbose: bool,
    once: bool,
    help: bool,
}

#[derive(Clone, Copy, Debug)]
struct CpuTimes {
    idle: u64,
    total: u64,
}

#[derive(Clone, Copy, Debug, Default)]
struct NetworkStats {
    received: u64,
    sent: u64,
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    cpu_percent: f32,
    ram_percent: f32,
    network_mbps: f32,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let options = parse_args(env::args().collect())?;
    if options.help {
        print_usage();
        return Ok(());
    }

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || {
        signal_running.store(false, Ordering::SeqCst);
    })
    .map_err(|error| format!("Failed to install signal handler: {error}"))?;

    run_daemon(&options, &running)
}

fn run_daemon(options: &Options, running: &AtomicBool) -> Result<(), String> {
    let mut waiting_logged = false;

    while running.load(Ordering::SeqCst) {
        let Some(port_name) = resolve_port(options) else {
            if !waiting_logged {
                eprintln!("Waiting for sysguage serial device");
                waiting_logged = true;
            }
            sleep_interruptible(options.retry_interval, running);
            continue;
        };

        match open_serial_port(&port_name) {
            Ok(mut port) => {
                waiting_logged = false;
                eprintln!("Connected to {port_name} at {BAUD_RATE} baud");
                match drive_connected_port(&mut *port, options, running) {
                    Ok(DriveResult::Done) => return Ok(()),
                    Ok(DriveResult::Reconnect) => {
                        eprintln!("Serial device disconnected; waiting to reconnect");
                        sleep_interruptible(options.retry_interval, running);
                    }
                    Err(error) => return Err(error),
                }
            }
            Err(error) => {
                if !waiting_logged {
                    eprintln!("Waiting for sysguage serial device: {error}");
                    waiting_logged = true;
                }
                sleep_interruptible(options.retry_interval, running);
            }
        }
    }

    Ok(())
}

enum DriveResult {
    Done,
    Reconnect,
}

fn resolve_port(options: &Options) -> Option<String> {
    options.port.clone().or_else(auto_detect_port)
}

fn open_serial_port(port_name: &str) -> Result<Box<dyn SerialPort>, String> {
    serialport::new(port_name, BAUD_RATE)
        .timeout(Duration::from_millis(1_000))
        .open()
        .map_err(|error| format!("Error opening {port_name}: {error}"))
}

fn drive_connected_port(
    port: &mut dyn SerialPort,
    options: &Options,
    running: &AtomicBool,
) -> Result<DriveResult, String> {
    let mut previous_cpu = read_cpu_times()?;
    let mut previous_network = read_network_stats()?;
    sleep_interruptible(options.interval, running);
    let mut next_tick = Instant::now();

    while running.load(Ordering::SeqCst) {
        let current_cpu = read_cpu_times()?;
        let current_network = read_network_stats()?;
        let sample = Sample {
            cpu_percent: calculate_cpu_usage(previous_cpu, current_cpu),
            ram_percent: read_ram_usage()?,
            network_mbps: calculate_network_mbps(
                previous_network,
                current_network,
                options.interval.as_secs_f32(),
            ),
        };

        previous_cpu = current_cpu;
        previous_network = current_network;

        let cpu_pwm = percent_to_pwm(sample.cpu_percent);
        let ram_pwm = percent_to_pwm(sample.ram_percent);
        let network_raw = sample.network_mbps.clamp(0.0, MAX_NETWORK_MBPS) as u32;

        if options.verbose {
            println!(
                "CPU {:.1}% ({cpu_pwm}), RAM {:.1}% ({ram_pwm}), NET {:.1} Mbps ({network_raw})",
                sample.cpu_percent, sample.ram_percent, sample.network_mbps
            );
        }

        if let Err(error) = send_frame(port, cpu_pwm, ram_pwm, network_raw) {
            eprintln!("Error writing to serial port: {error}");
            return Ok(DriveResult::Reconnect);
        }

        if options.once {
            return Ok(DriveResult::Done);
        }

        next_tick += options.interval;
        let now = Instant::now();
        if next_tick > now {
            sleep_interruptible(next_tick - now, running);
        } else {
            next_tick = now;
        }
    }

    Ok(DriveResult::Done)
}

fn parse_args(args: Vec<String>) -> Result<Options, String> {
    let mut options = Options {
        port: None,
        interval: DEFAULT_INTERVAL,
        retry_interval: DEFAULT_RETRY_INTERVAL,
        verbose: false,
        once: false,
        help: false,
    };

    let mut index = 1;
    while index < args.len() {
        match args[index].as_str() {
            "-h" | "--help" => {
                options.help = true;
                return Ok(options);
            }
            "-v" | "--verbose" => options.verbose = true,
            "--once" => options.once = true,
            "-p" | "--port" => {
                index += 1;
                let port = args
                    .get(index)
                    .ok_or_else(|| format!("{} requires a path", args[index - 1]))?;
                options.port = Some(port.clone());
            }
            "-i" | "--interval-ms" => {
                index += 1;
                let interval = args
                    .get(index)
                    .ok_or_else(|| format!("{} requires a value", args[index - 1]))?
                    .parse::<u64>()
                    .map_err(|_| {
                        "Interval must be an integer number of milliseconds".to_string()
                    })?;
                if interval < 50 {
                    return Err("Interval must be at least 50 ms".to_string());
                }
                options.interval = Duration::from_millis(interval);
            }
            "--retry-ms" => {
                index += 1;
                let retry_interval = args
                    .get(index)
                    .ok_or_else(|| format!("{} requires a value", args[index - 1]))?
                    .parse::<u64>()
                    .map_err(|_| {
                        "Retry interval must be an integer number of milliseconds".to_string()
                    })?;
                if retry_interval < 250 {
                    return Err("Retry interval must be at least 250 ms".to_string());
                }
                options.retry_interval = Duration::from_millis(retry_interval);
            }
            arg => return Err(format!("Unknown option: {arg}")),
        }
        index += 1;
    }

    Ok(options)
}

fn print_usage() {
    println!("Usage: sysguage [options]");
    println!();
    println!("Options:");
    println!("  -p, --port PATH          Serial port to use. Defaults to auto-detect.");
    println!("  -i, --interval-ms N      Update interval in milliseconds. Default: 250.");
    println!("      --retry-ms N         Missing-device retry interval. Default: 5000.");
    println!("  -v, --verbose            Print each sample before sending it.");
    println!("      --once               Send one frame and exit.");
    println!("  -h, --help               Show this help text.");
}

fn sleep_interruptible(duration: Duration, running: &AtomicBool) {
    let deadline = Instant::now() + duration;
    while running.load(Ordering::SeqCst) {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        thread::sleep((deadline - now).min(Duration::from_millis(100)));
    }
}

fn auto_detect_port() -> Option<String> {
    let by_id = list_serial_by_id(DEFAULT_BY_ID_DIR);
    for path in &by_id {
        let lower = path.to_string_lossy().to_lowercase();
        if lower.contains("arduino") || lower.contains("uno") || lower.contains("2341") {
            return Some(path.to_string_lossy().into_owned());
        }
    }

    if let Some(path) = by_id.first() {
        return Some(path.to_string_lossy().into_owned());
    }

    let mut candidates = serialport::available_ports().ok()?;
    candidates.sort_by(|left, right| left.port_name.cmp(&right.port_name));

    for info in &candidates {
        if let SerialPortType::UsbPort(usb) = &info.port_type {
            if usb.vid == 0x2341 {
                return Some(info.port_name.clone());
            }
        }
    }

    candidates
        .into_iter()
        .find(|info| info.port_name.starts_with("/dev/ttyACM"))
        .or_else(|| {
            serialport::available_ports().ok().and_then(|ports| {
                ports
                    .into_iter()
                    .find(|info| info.port_name.starts_with("/dev/ttyUSB"))
            })
        })
        .map(|info| info.port_name)
}

fn list_serial_by_id(directory: &str) -> Vec<PathBuf> {
    let mut paths = fs::read_dir(directory)
        .into_iter()
        .flat_map(|entries| entries.flatten())
        .map(|entry| entry.path())
        .filter(|path| path.file_name().is_some())
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn send_frame(
    port: &mut dyn SerialPort,
    cpu_pwm: u8,
    ram_pwm: u8,
    network_mbps: u32,
) -> io::Result<()> {
    let buffer = [
        cpu_pwm,
        ram_pwm,
        ((network_mbps >> 24) & 0xff) as u8,
        ((network_mbps >> 16) & 0xff) as u8,
        ((network_mbps >> 8) & 0xff) as u8,
        (network_mbps & 0xff) as u8,
    ];
    port.write_all(&buffer)?;
    port.flush()
}

fn read_cpu_times() -> Result<CpuTimes, String> {
    let stat = fs::read_to_string("/proc/stat")
        .map_err(|error| format!("Failed to read /proc/stat: {error}"))?;
    let line = stat
        .lines()
        .next()
        .ok_or_else(|| "Failed to read CPU line from /proc/stat".to_string())?;
    let mut fields = line.split_whitespace();

    if fields.next() != Some("cpu") {
        return Err("Unexpected /proc/stat format".to_string());
    }

    let values = fields
        .take(8)
        .map(|value| value.parse::<u64>())
        .collect::<Result<Vec<_>, _>>()
        .map_err(|_| "Failed to parse CPU counters".to_string())?;

    if values.len() < 8 {
        return Err("Incomplete CPU counters in /proc/stat".to_string());
    }

    let idle = values[3] + values[4];
    let total = values.iter().sum();
    Ok(CpuTimes { idle, total })
}

fn calculate_cpu_usage(previous: CpuTimes, current: CpuTimes) -> f32 {
    let idle_delta = current.idle.saturating_sub(previous.idle);
    let total_delta = current.total.saturating_sub(previous.total);
    if total_delta == 0 {
        return 0.0;
    }
    ((total_delta - idle_delta) as f64 * 100.0 / total_delta as f64) as f32
}

fn read_ram_usage() -> Result<f32, String> {
    let meminfo = fs::read_to_string("/proc/meminfo")
        .map_err(|error| format!("Failed to read /proc/meminfo: {error}"))?;
    let mut total_kb = None;
    let mut available_kb = None;

    for line in meminfo.lines() {
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("MemTotal:") => {
                total_kb = fields.next().and_then(|value| value.parse::<u64>().ok())
            }
            Some("MemAvailable:") => {
                available_kb = fields.next().and_then(|value| value.parse::<u64>().ok())
            }
            _ => {}
        }
    }

    let total = total_kb.ok_or_else(|| "MemTotal missing from /proc/meminfo".to_string())?;
    let available =
        available_kb.ok_or_else(|| "MemAvailable missing from /proc/meminfo".to_string())?;

    if total == 0 {
        return Ok(0.0);
    }

    Ok(((total - available) as f64 * 100.0 / total as f64) as f32)
}

fn read_network_stats() -> Result<NetworkStats, String> {
    let contents = fs::read_to_string("/proc/net/dev")
        .map_err(|error| format!("Failed to read /proc/net/dev: {error}"))?;
    let mut stats = NetworkStats::default();

    for line in contents.lines().skip(2) {
        let Some((iface, counters)) = line.split_once(':') else {
            continue;
        };

        if iface.trim() == "lo" {
            continue;
        }

        let values = counters
            .split_whitespace()
            .take(16)
            .map(|value| value.parse::<u64>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|_| "Failed to parse /proc/net/dev counters".to_string())?;

        if values.len() >= 10 {
            stats.received = stats.received.saturating_add(values[0]);
            stats.sent = stats.sent.saturating_add(values[8]);
        }
    }

    Ok(stats)
}

fn calculate_network_mbps(
    previous: NetworkStats,
    current: NetworkStats,
    interval_seconds: f32,
) -> f32 {
    if interval_seconds <= 0.0 {
        return 0.0;
    }

    let received_delta = current.received.saturating_sub(previous.received);
    let sent_delta = current.sent.saturating_sub(previous.sent);
    let bits = (received_delta + sent_delta) as f64 * 8.0;
    (bits / 1_000_000.0 / interval_seconds as f64) as f32
}

fn percent_to_pwm(percent: f32) -> u8 {
    (percent.clamp(0.0, 100.0) * 255.0 / 100.0) as u8
}
