#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <sys/sysinfo.h>
#include <cstring>

// Structure to hold network statistics
struct NetworkStats {
    unsigned long long bytesSent;
    unsigned long long bytesReceived;
};

// Open the serial port
int openSerialPort(const std::string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Error opening serial port");
        return -1;
    }
    fcntl(fd, F_SETFL, 0);  // Set file descriptor for blocking mode
    return fd;
}

// Configure the serial port
void configureSerialPort(int fd, int baudRate) {
    struct termios options;
    tcgetattr(fd, &options);  // Get current port settings

    // Set baud rate
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);

    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;           // No parity
    options.c_cflag &= ~CSTOPB;           // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;               // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable flow control
    options.c_oflag &= ~OPOST;                          // Raw output mode

    tcsetattr(fd, TCSANOW, &options);  // Apply the settings
}

// Flush serial port
void flushSerialPort(int fd) {
    tcflush(fd, TCIOFLUSH);
}

// Send data to the serial port
void sendData(int fd, uint8_t pwmCPU, uint8_t pwmRAM, uint32_t networkUsage) {
    // Create a buffer to send
    uint8_t buffer[6];

    // CPU and RAM usage (1 byte each)
    buffer[0] = pwmCPU;
    buffer[1] = pwmRAM;

    // Network usage (4 bytes, big-endian format)
    buffer[2] = (networkUsage >> 24) & 0xFF;
    buffer[3] = (networkUsage >> 16) & 0xFF;
    buffer[4] = (networkUsage >> 8) & 0xFF;
    buffer[5] = networkUsage & 0xFF;

    ssize_t bytesWritten = write(fd, buffer, sizeof(buffer));
    if (bytesWritten < 0) {
        perror("Error writing to serial port");
    }
}

// Read current network statistics
NetworkStats readNetworkStats() {
    NetworkStats stats = {0, 0};
    std::ifstream file("/proc/net/dev");
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open /proc/net/dev" << std::endl;
        return stats;
    }

    std::string line;
    // Skip the first two lines
    std::getline(file, line);
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string iface;
        unsigned long long bytesRecv, bytesSent;

        // Parse interface name and relevant statistics
        iss >> iface >> bytesRecv;
        for (int i = 0; i < 7; ++i) iss >> bytesSent;  // Skip irrelevant columns
        stats.bytesReceived += bytesRecv;
        stats.bytesSent += bytesSent;
    }
    file.close();
    return stats;
}

// Calculate network usage in Mbps, adjusted for interval
float calculateNetworkUsage(const NetworkStats& prev, const NetworkStats& current, double intervalSeconds) {
    unsigned long long sentDelta = current.bytesSent - prev.bytesSent;
    unsigned long long recvDelta = current.bytesReceived - prev.bytesReceived;

    // Convert bytes to megabits (1 byte = 8 bits, 1 Mbps = 1,000,000 bits)
    double bits = static_cast<double>(sentDelta + recvDelta) * 8.0;
    double megabits = bits / 1000000.0;

    // Adjust for the interval to get per-second Mbps
    return static_cast<float>(megabits / intervalSeconds);
}

// Calculate CPU usage
float getCPUUsage() {
    static long long prevIdle = 0, prevTotal = 0;
    long long idle, total, idleDelta, totalDelta;
    long long user, nice, system, irq, softirq, steal, iowait;

    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open /proc/stat" << std::endl;
        return 0;
    }
    std::string line;
    std::getline(file, line);
    file.close();

    sscanf(line.c_str(), "cpu  %lld %lld %lld %lld %lld %lld %lld %lld",
           &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);

    long long idleTime = idle + iowait;
    total = user + nice + system + irq + softirq + steal + idleTime;

    idleDelta = idleTime - prevIdle;
    totalDelta = total - prevTotal;

    prevIdle = idleTime;
    prevTotal = total;

    return (totalDelta > 0) ? (100.0 * (totalDelta - idleDelta) / totalDelta) : 0.0;
}

// Get RAM usage
float getRAMUsage() {
    struct sysinfo memInfo;
    if (sysinfo(&memInfo) != 0) {
        std::cerr << "Error: Unable to retrieve RAM information." << std::endl;
        return 0.0;
    }
    float totalRAM = memInfo.totalram * memInfo.mem_unit;
    float freeRAM = memInfo.freeram * memInfo.mem_unit;
    return ((totalRAM - freeRAM) / totalRAM) * 100.0;
}

// Main function
int main() {
    std::string port = "/dev/ttyACM0";  // Replace with your serial port
    int baudRate = B9600;              // Baud rate

    // Open the serial port
    int fd = openSerialPort(port);
    if (fd == -1) {
        return 1;
    }

    // Configure the serial port
    configureSerialPort(fd, baudRate);

    // Flush any existing data
    flushSerialPort(fd);

    std::cout << "Connected to " << port << " at 9600 baud." << std::endl;

    // Initialize network statistics
    NetworkStats prevNetStats = readNetworkStats();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Wait 1 second for initial calculation
    prevNetStats = readNetworkStats(); // Update after initial sleep

    // Define the measurement interval in seconds
    const double intervalSeconds = 0.25; // 250ms

    while (true) {
        auto startTime = std::chrono::steady_clock::now();

        float cpuUsage = getCPUUsage();
        float ramUsage = getRAMUsage();

        // Calculate network usage
        NetworkStats currentNetStats = readNetworkStats();
        float networkUsage = calculateNetworkUsage(prevNetStats, currentNetStats, intervalSeconds);
        prevNetStats = currentNetStats;  // Update for the next iteration

        std::cout << "CPU Usage: " << cpuUsage << "%, RAM Usage: " << ramUsage
                  << "%, Network Usage: " << networkUsage << " Mbps" << std::endl;

        // Map CPU and RAM usage to PWM values (0-255)
        uint8_t pwmCPU = static_cast<uint8_t>(std::min(cpuUsage * 255.0f / 100.0f, 255.0f));
        uint8_t pwmRAM = static_cast<uint8_t>(std::min(ramUsage * 255.0f / 100.0f, 255.0f));

        // Convert network usage to Mbps and scale it as uint32_t
        // Ensure that networkUsage does not exceed 100 Mbps as per firmware scaling
        uint32_t networkUsageRaw = static_cast<uint32_t>(std::min(networkUsage, 100.0f));

        // Send data to Arduino
        sendData(fd, pwmCPU, pwmRAM, networkUsageRaw);

        // Calculate elapsed time and sleep for the remaining interval
        auto endTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        double sleepTime = intervalSeconds - elapsed.count();
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
        }
    }

    close(fd);
    return 0;
}
