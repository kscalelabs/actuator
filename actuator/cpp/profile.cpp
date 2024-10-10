#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <signal.h>
#include <fstream>

#define CAN_ID_MASTER (0X00)        // Control host address - SPIE
#define CAN_ID_MOTOR_DEFAULT (0X7F) // Default motor address - Unconfigured ID
#define CAN_ID_BROADCAST (0XFE) // Broadcast address - Default receive address
#define CAN_ID_DEBUG_UI (0XFD)  // Debug address - Upper computer address

#define BAUDRATE 921600

#define MOTOR_ID 1

#if __linux__
#define TTY_PORT "/dev/ttyCH341USB0"
// #define TTY_PORT "/dev/ttyUSB0"
#elif __APPLE__
#define TTY_PORT "/dev/tty.wchusbserial110"
#else
#error "Unsupported platform"
#endif

#define MOTOR_TYPE 3

#if MOTOR_TYPE == 1

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#elif MOTOR_TYPE == 3

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -20.0f
#define V_MAX 20.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -60.0f
#define T_MAX 60.0f

#elif MOTOR_TYPE == 4

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -15.0f
#define V_MAX 15.0f
#define KP_MIN 0.0f
#define KP_MAX 5000.0f
#define KD_MIN 0.0f
#define KD_MAX 100.0f
#define T_MIN -120.0f
#define T_MAX 120.0f

#else

#error "Unsupported motor type"

#endif

enum canComMode {
  CANCOM_ANNOUNCE_DEVID = 0,
  CANCOM_MOTOR_CTRL,
  CANCOM_MOTOR_FEEDBACK,
  CANCOM_MOTOR_IN,
  CANCOM_MOTOR_RESET,
  CANCOM_MOTOR_CALI,
  CANCOM_MOTOR_ZERO,
  CANCOM_MOTOR_ID,
  CANCOM_PARA_WRITE,
  CANCOM_PARA_READ,
  CANCOM_PARA_UPDATE,
  CANCOM_OTA_START,
  CANCOM_OTA_INFO,
  CANCOM_OTA_ING,
  CANCOM_OTA_END,
  CANCOM_CALI_ING,
  CANCOM_CALI_RST,
  CANCOM_SDO_READ,
  CANCOM_SDO_WRITE,
  CANCOM_PARA_STR_INFO,
  CANCOM_MOTOR_BRAKE,
  CANCOM_FAULT_WARN,
  CANCOM_MODE_TOTAL,
};

enum motorMode {
  MT_MODE_RESET = 0,
  MT_MODE_CALI,
  MT_MODE_MOTOR,
  MT_MODE_BRAKE,
};

struct motorStatus {
  bool underVoltFault;
  bool overCurFault;
  bool overTempFault;
  bool encoderFault;
  bool hallFault;
  bool noCaliFault;
  enum motorMode mode;
};

enum runMode {
  MIT_MODE = 0,
  POSITION_MODE,
  SPEED_MODE,
  CURRENT_MODE,
  TO_ZERO_MODE,
  CSP_POSITION_MODE,
};

struct ExId {
  uint32_t id : 8;
  uint32_t data : 16;
  canComMode mode : 5;
  uint32_t res : 3;
};

struct CanPack {
  ExId exId;
  uint8_t len;
  uint8_t data[8];
};

int my_serialport;

void txdPack(CanPack *pack) {
  std::vector<uint8_t> Pack;

  if (pack->len < 1) {
    pack->len = 1;
    pack->data[0] = 0;
  } else if (pack->len > 8) {
    pack->len = 8;
  }
  pack->exId.res = 0;

  Pack.resize(9 + pack->len);

  Pack[0] = 'A';
  Pack[1] = 'T';

  uint32_t addr;
  memcpy(&addr, &(pack->exId), 4);
  addr = (addr << 3) | (0x00000004);

  Pack[2] = (uint8_t)((addr & 0xFF000000) >> 24);
  Pack[3] = (uint8_t)((addr & 0x00FF0000) >> 16);
  Pack[4] = (uint8_t)((addr & 0x0000FF00) >> 8);
  Pack[5] = (uint8_t)((addr & 0x000000FF));
  Pack[6] = pack->len;

  for (uint8_t i = 0; i < pack->len; i++)
    Pack[7 + i] = pack->data[i];

  Pack[7 + pack->len] = '\r';
  Pack[8 + pack->len] = '\n';

  // Print the full string of bits being sent
  std::cout << "tx ";
  for (size_t i = 0; i < Pack.size(); ++i) {
    std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)Pack[i]
              << " ";
  }
  std::cout << std::dec << std::endl;

  // Write the data
  ssize_t bytes_written =
      write(my_serialport, Pack.data(), (uint16_t)Pack.size());
  if (bytes_written < 0) {
    std::cerr << "Error writing to serial port: " << strerror(errno)
              << std::endl;
  } else if (bytes_written != Pack.size()) {
    std::cerr << "Warning: Not all bytes written. Expected " << Pack.size()
              << ", wrote " << bytes_written << std::endl;
  }

  // Clear the buffer
  tcflush(my_serialport, TCIFLUSH);
}

int initSerialPort(const char *device) {
  std::cout << "Initializing serial port: " << device << std::endl;

  int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1) {
    std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
    return -1;
  }

  // Clear the O_NONBLOCK flag to make read/write blocking again
  int flags = fcntl(fd, F_GETFL, 0);
  flags &= ~O_NONBLOCK;
  fcntl(fd, F_SETFL, flags);

  struct termios options;
  tcgetattr(fd, &options);

  // Configure port settings (baud rate, data bits, stop bits, parity, etc.)
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);

  // 8 data bits, no parity, 1 stop bit
  options.c_cflag &= ~PARENB; // No parity
  options.c_cflag &= ~CSTOPB; // One stop bit
  options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;     // 8 data bits

  // Enable the receiver and set local mode
  options.c_cflag |=
      CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Set raw input/output mode
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag = 0;
  options.c_oflag &= ~OPOST;

  // Set VMIN and VTIME
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10; // Timeout in deciseconds (1 second)

  // Apply the settings
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    std::cerr << "Error setting terminal attributes: " << strerror(errno)
              << std::endl;
    close(fd);
    return -1;
  }

  // Verify the settings
  struct termios verifyOptions;
  if (tcgetattr(fd, &verifyOptions) != 0) {
    std::cerr << "Error getting terminal attributes: " << strerror(errno)
              << std::endl;
    close(fd);
    return -1;
  }

  if (verifyOptions.c_cflag != options.c_cflag ||
      verifyOptions.c_iflag != options.c_iflag ||
      verifyOptions.c_oflag != options.c_oflag ||
      verifyOptions.c_lflag != options.c_lflag) {
    std::cerr << "Serial port settings do not match the requested configuration"
              << std::endl;
    close(fd);
    return -1;
  }

  // Flush the serial port
  tcflush(fd, TCIOFLUSH);

  std::cout << "Serial port initialized successfully" << std::endl;
  return fd;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

struct MotorFeedback {
  uint32_t canId;
  float position;
  float velocity;
  float torque;
  motorMode mode;
  uint16_t faults;
  bool isSet;
};

MotorFeedback read_bytes() {
  MotorFeedback feedback = {0, 0.0f, 0.0f, 0.0f, MT_MODE_RESET,
                            0, false}; // Initialize isSet to false
  std::vector<uint8_t> response;
  char buffer[17];
  ssize_t bytes_read = read(my_serialport, buffer, 17);
  if (bytes_read == 0) {
    std::cout << "No bytes read" << std::endl;
    return feedback;
  }

  std::cout << "rx ";
  for (ssize_t i = 0; i < bytes_read; i++) {
    std::cout << std::hex << std::setw(2) << std::setfill('0')
              << (int)(unsigned char)buffer[i] << ' ' << std::dec;
  }
  std::cout << std::endl;

  if (bytes_read == 17 && buffer[0] == 'A' && buffer[1] == 'T') {
    CanPack rxFrame;

    uint32_t addr = buffer[5] & 0x000000FF;
    addr |= (buffer[4] << 8) & 0x0000FF00;
    addr |= (buffer[3] << 16) & 0x00FF0000;
    addr |= (buffer[2] << 24) & 0xFF000000;
    addr = addr >> 3;
    memcpy(&(rxFrame.exId), &addr, 4);

    rxFrame.len = buffer[6];
    for (uint8_t i = 0; i < rxFrame.len; i++) {
      rxFrame.data[i] = buffer[7 + i];
    }

    // Parse the data
    feedback.canId = rxFrame.exId.data & 0x00FF;
    feedback.faults = (rxFrame.exId.data & 0x3F00) >> 8;
    feedback.mode = (enum motorMode)((rxFrame.exId.data & 0xC000) >> 14);

    uint16_t posIntGet = ((int)rxFrame.data[0] << 8) | rxFrame.data[1];
    uint16_t velIntGet = ((int)rxFrame.data[2] << 8) | rxFrame.data[3];
    uint16_t torqueIntGet = ((int)rxFrame.data[4] << 8) | rxFrame.data[5];

    feedback.position = uint_to_float(posIntGet, P_MIN, P_MAX, 16);
    feedback.velocity = uint_to_float(velIntGet, V_MIN, V_MAX, 16);
    feedback.torque = uint_to_float(torqueIntGet, T_MIN, T_MAX, 16);

    feedback.isSet = true;

    // Print parsed data
    std::cout << "Parsed data:" << std::endl;
    std::cout << "  Motor ID: " << feedback.canId << std::endl;
    std::cout << "  Position: " << feedback.position << std::endl;
    std::cout << "  Velocity: " << feedback.velocity << std::endl;
    std::cout << "  Torque: " << feedback.torque << std::endl;
    std::cout << "  Mode: " << feedback.mode << std::endl;
    std::cout << "  Faults: " << (feedback.faults & 0x01 ? "UnderVolt " : "")
              << (feedback.faults & 0x02 ? "OverCurrent " : "")
              << (feedback.faults & 0x04 ? "OverTemp " : "")
              << (feedback.faults & 0x08 ? "Encoder " : "")
              << (feedback.faults & 0x10 ? "Hall " : "")
              << (feedback.faults & 0x20 ? "NoCali" : "") << std::endl;
  }

  return feedback;
}

MotorFeedback send_set_mode(runMode runmode, uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_SDO_WRITE;
  pack.exId.res = 0;

  uint16_t index = 0x7005;
  memcpy(&pack.data[0], &index, 2);
  memcpy(&pack.data[4], &runmode, 1);
  pack.len = 8;

  txdPack(&pack);
  return read_bytes();
}

MotorFeedback send_reset(uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_MOTOR_RESET;
  pack.exId.res = 0;

  txdPack(&pack);
  return read_bytes();
}

MotorFeedback send_start(uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_MOTOR_IN;
  pack.exId.res = 0;

  txdPack(&pack);
  return read_bytes();
}

MotorFeedback send_set_speed_limit(uint8_t id, float speed) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_SDO_WRITE;
  pack.exId.res = 0;

  uint16_t index = 0x7017;
  memcpy(&pack.data[0], &index, 2);
  memcpy(&pack.data[4], &speed, 4);

  txdPack(&pack);
  return read_bytes();
}

MotorFeedback send_set_location(uint8_t id, float location) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_SDO_WRITE;
  pack.exId.res = 0;

  uint16_t index = 0x7016;
  memcpy(&pack.data[0], &index, 2);
  memcpy(&pack.data[4], &location, 4);

  txdPack(&pack);
  return read_bytes();
}

// Ai generated - not tested @aaron
MotorFeedback send_set_zero(uint8_t id) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.len = 8;
  pack.exId.id = id;
  pack.exId.data = CAN_ID_DEBUG_UI;
  pack.exId.mode = CANCOM_MOTOR_ZERO;
  pack.exId.res = 0;

  pack.data[0] = 1;  // Set Byte[0] to 1 as specified in the manual

  txdPack(&pack);
  return read_bytes();
}

// Add this function to convert float to uint
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

MotorFeedback send_motor_control(uint8_t id, float posSet, float velSet,
                                 float kpSet, float kdSet, float torqueSet) {
  CanPack pack;
  memset(&pack, 0, sizeof(CanPack));
  pack.exId.id = id;
  pack.len = 8;
  pack.exId.mode = CANCOM_MOTOR_CTRL;
  pack.exId.res = 0;

  uint16_t posIntSet = float_to_uint(posSet, P_MIN, P_MAX, 16);
  uint16_t velIntSet = float_to_uint(velSet, V_MIN, V_MAX, 16);
  uint16_t kpIntSet = float_to_uint(kpSet, KP_MIN, KP_MAX, 16);
  uint16_t kdIntSet = float_to_uint(kdSet, KD_MIN, KD_MAX, 16);
  uint16_t torqueIntSet = float_to_uint(torqueSet, T_MIN, T_MAX, 16);

  pack.exId.data = torqueIntSet;

  pack.data[0] = posIntSet >> 8;
  pack.data[1] = posIntSet & 0xFF;
  pack.data[2] = velIntSet >> 8;
  pack.data[3] = velIntSet & 0xFF;
  pack.data[4] = kpIntSet >> 8;
  pack.data[5] = kpIntSet & 0xFF;
  pack.data[6] = kdIntSet >> 8;
  pack.data[7] = kdIntSet & 0xFF;

  txdPack(&pack);
  return read_bytes();
} 

MotorFeedback send_position_control(uint8_t id, float posSet, float kpSet, float kdSet) {
  return send_motor_control(id, posSet, 0, kpSet, kdSet, 0);
}

MotorFeedback send_torque_control(uint8_t id, float torqueSet) {
  return send_motor_control(id, 0, 0, 0, 0, torqueSet);
}

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // Send reset message to the motor
    MotorFeedback feedback = send_reset(MOTOR_ID);
    if (feedback.isSet) {
        std::cout << "Reset feedback received" << std::endl;
    }

    // Close the serial port
    std::cout << "Closing serial port" << std::endl;
    close(my_serialport);

    // Terminate program
    exit(signum);
}

int main() {
  // Register signal handler
  signal(SIGINT, signalHandler);

  std::cout << "Starting program" << std::endl;

  // Initialize serial ports
  my_serialport = initSerialPort(TTY_PORT);
  if (my_serialport == -1)
    return -1;

  MotorFeedback feedback;

  // Set mode to calibration
  feedback = send_set_mode(MIT_MODE, MOTOR_ID); // MIT mode
  if (feedback.isSet) {
    std::cout << "Set mode feedback received" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  send_reset(MOTOR_ID);

  // Start the motor
  feedback = send_start(MOTOR_ID);
  if (feedback.isSet) {
    std::cout << "Start feedback received" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Set speed limit
  feedback = send_set_speed_limit(MOTOR_ID, 10);
  if (feedback.isSet) {
    std::cout << "Set speed limit feedback received" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // // Set location.
  // feedback = send_set_location(1, 0);

  // Set zero position 
  feedback = send_set_zero(MOTOR_ID);
  if (feedback.isSet) {
    std::cout << "Set zero feedback received" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // Open a CSV file to log the positions
  std::ofstream logFile("position_log.csv");
  if (!logFile.is_open()) {
    std::cerr << "Error opening log file" << std::endl;
    return -1;
  }

  // Write CSV header
  logFile << "Step,Desired Position,Actual Position\n";

  // Sine wave torque control loop
  const double pi = 3.14159265358979323846;
  const double magnitude = pi / 2;
  const double period = 5.0;
  const double duration = 20.0;
  const double kp = 5;
  const double kd = 0.2;

  auto start_time = std::chrono::steady_clock::now();
  double elapsed_time = 0.0;
  int instruction_count = 0;

  double desired_torque = 0.0;
  double desired_position = 0.0;
  int step = 0; // Initialize step counter

  const std::chrono::milliseconds loop_duration(4); // 4 milliseconds for 250 Hz

  while (elapsed_time < duration) {
    auto loop_start_time = std::chrono::steady_clock::now();

    if (feedback.isSet) {
      desired_position = magnitude * std::sin(2 * pi * elapsed_time / period);
      double current_position = feedback.position;
      double position_error = desired_position - current_position;
      desired_torque = kp * position_error + kd * (0 - feedback.velocity);

      // Log the desired and current positions in CSV format
      logFile << step << "," << desired_position << "," << current_position << "\n";
      step++; // Increment step counter
    }

    // Send torque control command
    feedback = send_torque_control(MOTOR_ID, desired_torque);
    instruction_count++;

    // Calculate elapsed time for the loop
    auto loop_end_time = std::chrono::steady_clock::now();
    auto loop_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);

    // Sleep for the remaining time to maintain 250 Hz
    if (loop_elapsed_time < loop_duration) {
      std::this_thread::sleep_for((loop_duration - loop_elapsed_time)*0.99);
    }

    // Update elapsed time
    auto current_time = std::chrono::steady_clock::now();
    elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
    if (elapsed_time > 0) {
      std::cout << "Instructions per second: " << (float)instruction_count / elapsed_time << std::endl;
    }
  }

  // Reset the motor after the loop
  feedback = send_reset(MOTOR_ID);
  if (feedback.isSet) {
    std::cout << "Reset feedback received" << std::endl;
  }

  // Close the CSV file
  logFile.close();

  // Close the serial port when done
  std::cout << "Closing serial port" << std::endl;
  close(my_serialport);

  std::cout << "Program finished" << std::endl;
  return 0;
}
