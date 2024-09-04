import serial
import psutil
import time
import serial.tools.list_ports
import os
import json

# Configuration file to save the last used port
config_file = 'arduino_config.json'


def save_config(port):
    """Save the last used port to a configuration file."""
    with open(config_file, 'w') as f:
        json.dump({'port': port}, f)


def load_config():
    """Load the last used port from the configuration file."""
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            return json.load(f).get('port', None)
    return None


def detect_ports():
    """Detect available serial ports."""
    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 0:
        print("No serial ports found. Please connect your Arduino and try again.")
        return None
    return ports


def choose_port():
    """Allow the user to choose a serial port, using the last saved port if available."""
    ports = detect_ports()
    if ports is None:
        return None

    last_port = load_config()
    if last_port and any(port.device == last_port for port in ports):
        print(f"\nLast used port: {last_port}")
        use_last = input("Do you want to use the last used port? (y/n): ").strip().lower()
        if use_last == 'y':
            return last_port

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")

    try:
        choice = int(input("Select the port number: "))
        selected_port = ports[choice].device
        save_config(selected_port)
        return selected_port
    except (ValueError, IndexError):
        print("Invalid selection. Please run the script again.")
        return None


def get_network_usage():
    """Calculate the network usage in Mbps."""
    net_io = psutil.net_io_counters()
    bytes_sent = net_io.bytes_sent
    bytes_recv = net_io.bytes_recv

    time.sleep(1)  # Sleep for 1 second to calculate the network usage per second

    net_io_after = psutil.net_io_counters()
    bytes_sent_after = net_io_after.bytes_sent
    bytes_recv_after = net_io_after.bytes_recv

    # Calculate the number of bytes sent and received in 1 second, then convert to Mbps
    bytes_sent_per_sec = (bytes_sent_after - bytes_sent) * 8 / (1024 * 1024)  # Convert to megabits
    bytes_recv_per_sec = (bytes_recv_after - bytes_recv) * 8 / (1024 * 1024)  # Convert to megabits

    # Return the total network speed in Mbps
    return bytes_sent_per_sec + bytes_recv_per_sec


def main():
    port = choose_port()
    if not port:
        return

    baud_rate = 9600  # Must match the Arduino's baud rate

    try:
        # Establish the serial connection
        ser = serial.Serial(port, baud_rate)
        print(f"Connected to {port} at {baud_rate} baud.")
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        return

    try:
        while True:
            # Get CPU and RAM usage percentages (0-100)
            cpu_usage = psutil.cpu_percent(interval=1 / 4)  # Update 4 times per second
            ram_usage = psutil.virtual_memory().percent     # Get RAM usage

            # Map CPU and RAM usage (0-100) to PWM range (0-255)
            pwm_cpu = int(cpu_usage * 255 / 100)
            pwm_ram = int(ram_usage * 255 / 100)

            # Get network usage in Mbps
            network_usage = get_network_usage()

            print(f"CPU: {pwm_cpu}, RAM: {pwm_ram}, Network: {network_usage} Mbps")

            # Send the data to Arduino
            send_data(ser, pwm_cpu, pwm_ram, network_usage)

            # Wait a bit before sending the next value
            time.sleep(0.25)  # 4 times per second

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")


def send_data(ser, pwm_cpu, pwm_ram, raw_network_usage):
    """Send data to Arduino."""
    # Send CPU, RAM, and raw network data
    ser.write(bytes([pwm_cpu, pwm_ram]))  # Send CPU and RAM values as bytes
    ser.write(int(raw_network_usage).to_bytes(4, 'big'))  # Send raw network usage as 4-byte integer


if __name__ == '__main__':
    main()
