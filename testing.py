import tkinter as tk
import serial
import time
import serial.tools.list_ports
import os
import json
import psutil

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


class CPULoadSimulator:
    def __init__(self, master, ser):
        self.master = master
        self.ser = ser

        self.last_bytes_sent = psutil.net_io_counters().bytes_sent
        self.last_bytes_recv = psutil.net_io_counters().bytes_recv

        master.title("System Usage Simulator")

        self.cpu_scale = tk.Scale(master, from_=0, to=100, orient=tk.HORIZONTAL, label="CPU Usage (%)")
        self.cpu_scale.pack()

        self.ram_scale = tk.Scale(master, from_=0, to=100, orient=tk.HORIZONTAL, label="RAM Usage (%)")
        self.ram_scale.pack()

        self.update_interval = 250  # Update 4 times per second
        self.update_usage()

    def send_data(self, pwm_cpu, pwm_ram, raw_network_usage):
        """Send data to Arduino."""
        # Send CPU, RAM, and raw network data
        self.ser.write(bytes([pwm_cpu, pwm_ram]))  # Send CPU and RAM values as bytes
        self.ser.write(raw_network_usage.to_bytes(4, 'big'))  # Send raw network usage as 4-byte integer

    def update_usage(self):
        """Update system usage and send the corresponding PWM values."""
        # CPU usage
        cpu_usage = self.cpu_scale.get()
        pwm_cpu = int(cpu_usage * 255 / 100)

        # RAM usage
        ram_usage = self.ram_scale.get()
        pwm_ram = int(ram_usage * 255 / 100)

        # Network usage
        net_usage = self.get_network_usage()
        print(f"CPU: {pwm_cpu}, RAM: {pwm_ram}, Network: {net_usage} Mbps")

        # Send data to Arduino
        self.send_data(pwm_cpu, pwm_ram, int(net_usage))

        # Schedule the next update
        self.master.after(self.update_interval, self.update_usage)

    def get_network_usage(self):
        """Calculate the network usage in Mbps."""
        net_io = psutil.net_io_counters()
        bytes_sent = net_io.bytes_sent
        bytes_recv = net_io.bytes_recv

        # Calculate network usage in Mbps
        sent_speed = (bytes_sent - self.last_bytes_sent) * 8 / (1024 * 1024)  # Convert bytes to megabits
        recv_speed = (bytes_recv - self.last_bytes_recv) * 8 / (1024 * 1024)  # Convert bytes to megabits

        # Update the last recorded bytes sent/recv
        self.last_bytes_sent = bytes_sent
        self.last_bytes_recv = bytes_recv

        # Return combined network speed (sent + received)
        return sent_speed + recv_speed


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

    root = tk.Tk()
    app = CPULoadSimulator(root, ser)
    root.mainloop()

    if ser.is_open:
        ser.close()
        print("Serial port closed.")


if __name__ == '__main__':
    main()
