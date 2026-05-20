# Sysguage

Sysguage is the PC-side Rust driver for an Arduino-based system gauge. It
samples Linux CPU, memory, and network counters and streams compact binary
frames to the device over USB serial.

The device firmware is assumed to already be installed. This repository only
contains the host-side driver.

## Protocol

The driver sends one 6-byte frame per update:

```text
byte 0      CPU usage mapped to PWM, 0-255
byte 1      RAM usage mapped to PWM, 0-255
bytes 2-5   network usage in Mbps as uint32, big-endian, capped at 100
```

Default serial settings:

```text
baud: 9600
format: 8N1
flow control: none
interval: 250 ms
```

## Build

Install Rust with rustup, then build:

```bash
cargo build --release
```

The optimized binary is:

```bash
target/release/sysguage
```

The Makefile wraps the same commands:

```bash
make release
```

## Repository Layout

```text
src/                 Rust driver source
deploy/systemd/      user and system service definitions
deploy/udev/         serial-device permission rule
scripts/             install helpers
.github/workflows/   CI and release automation
```

## Usage

Run with auto-detection:

```bash
target/release/sysguage
```

Sysguage first looks for Arduino-like devices under `/dev/serial/by-id`, then
falls back to serial ports reported by the OS.

Useful options:

```bash
target/release/sysguage --port /dev/sysguage
target/release/sysguage --interval-ms 250
target/release/sysguage --retry-ms 5000
target/release/sysguage --verbose
target/release/sysguage --once
```

If the Arduino is not connected, the driver keeps running and checks for it at
the retry interval. While waiting, it sleeps between checks to keep overhead low.
If the USB device is unplugged while running, the driver returns to the same
wait/reconnect loop.

## Serial Permissions

On Linux, the serial device is usually owned by `root:dialout`. There are two
good ways to avoid running the driver with sudo.

For terminal sessions and services, add your user to `dialout`, then log out and
back in:

```bash
sudo usermod -a -G dialout "$USER"
```

For desktop sessions, install the included udev rule:

```bash
./scripts/install-udev-rule.sh
```

The rule matches the Arduino Uno R3 USB ID, grants active-user access through
`uaccess`, keeps `dialout` group access, and creates a stable `/dev/sysguage`
symlink. Unplug and reconnect the Arduino after installing it.

The udev rule requires sudo once because it writes to `/etc/udev/rules.d`. After
that, the driver itself should run without sudo.

## Service Install

Build the optimized binary:

```bash
cargo build --release
```

For a per-user systemd service:

```bash
./scripts/install-user-service.sh
```

This installs the binary to `~/.local/bin/sysguage`, enables
`sysguage.service`, and starts it immediately. To allow the user service to start
at boot before you log in, enable linger once:

```bash
sudo loginctl enable-linger "$USER"
```

For a system-wide boot service:

```bash
./scripts/install-system-service.sh
```

This installs the binary to `/usr/local/bin/sysguage`, installs
`/etc/systemd/system/sysguage.service`, enables it, and starts it. The included
system service is rendered for the current user and group `dialout`.

Useful service commands:

```bash
systemctl --user status sysguage.service
systemctl --user restart sysguage.service
journalctl --user -u sysguage.service -f
```

## Background Use

The default mode is quiet and lightweight: it wakes four times per second, reads
Linux procfs counters, writes 6 bytes to serial, and sleeps. Use `--verbose`
only when debugging because it logs every sample. When the device is absent, it
only wakes at the retry interval.

## Releases

CI runs formatting, Clippy, and tests on pushes to `main` and pull requests.

Release builds are automatic for version tags:

```bash
git tag v0.2.0
git push origin v0.2.0
```

The release workflow builds the optimized Linux binary, packages it with the
deployment files, writes a SHA-256 checksum, and publishes both files to a
GitHub Release.
