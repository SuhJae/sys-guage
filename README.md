# Sysguage

The **Sysguage** project is a system monitoring tool that uses an Arduino and a serial connection to monitor your computer's CPU and RAM usage. The data is sent to the Arduino, which can be used to control external devices like LEDs or displays.

## **Features**

- Real-time monitoring of CPU, RAM, and network usage.
- Cross-platform support for Linux, macOS, and Windows.
- Simple serial communication with an Arduino.

## **Project Structure**

```
.
├── arduino/
│   ├── main.ino         # Arduino firmware
├── src/
│   ├── sysguage.cpp     # Main C++ application code
├── Makefile             # Build file for Linux/macOS
├── CMakeLists.txt       # Build file for cross-platform support
├── README.md            # Project documentation
```

## **Getting Started**

Follow the steps below to set up and run the Sysguage project.

### **1. Clone the Repository**

Clone the repository to your local machine:

```bash
git clone https://github.com/SuhJae/sys-guage.git
cd sys-guage
```

### **2. Upload Arduino Code**

1. Open the Arduino IDE.
2. Load the `arduino/main.ino` file.
3. Connect your Arduino and select the appropriate board and port.
4. Upload the firmware to the Arduino.

### **3. Compile and Run the C++ Application**

#### **For Linux/macOS**

1. Ensure you have a C++ compiler and `make` installed.

   - On Ubuntu:
     ```bash
     sudo apt update
     sudo apt install build-essential
     ```

2. Compile the project using the `Makefile`:

   ```bash
   make
   ```

3. Run the application:

   ```bash
   ./sysguage
   ```

4. To clean the build files:
   ```bash
   make clean
   ```

#### **For Cross-Platform (Linux/macOS/Windows)**

1. Ensure you have CMake installed.

   - On Ubuntu:
     ```bash
     sudo apt install cmake
     ```
   - On macOS:
     ```bash
     brew install cmake
     ```
   - On Windows, download and install CMake from [cmake.org](https://cmake.org/).

2. Build the project:

   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. Run the application:
   ```bash
   ./sysguage
   ```

### **4. Permissions (Linux/macOS)**

To avoid using `sudo` when accessing the serial port, add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

After running the above command, log out and log back in for the changes to take effect.

## **Dependencies**

### **Linux/macOS**

- `gcc` or `clang` (C++ compiler)
- `make` (for Makefile)
- `cmake` (optional for cross-platform builds)

Install dependencies on Ubuntu:

```bash
sudo apt install build-essential cmake
```

### **Windows**

- [MinGW](https://www.mingw-w64.org/) or another C++ compiler
- [CMake](https://cmake.org/)

## **Usage**

1. Connect the Arduino to your computer.
2. Run the `sysguage` application:
   ```bash
   ./sysguage
   ```
3. The application will:
   - Calculate CPU RAM, and network usage.
   - Send data to the Arduino in real-time via the serial port.
