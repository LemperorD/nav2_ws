#include "communication/Com.h"
#include "communication/termcolor.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

uint8_t SerialCommunicationClass::crc8_calc(const uint8_t* p, size_t len)
{
  uint8_t crc = CRC8_INIT;
  while (len--) {
    crc = CRC8_TABLE[crc ^ *p++];
  }
  return crc;
}

bool SerialCommunicationClass::isSupportedCommand(uint8_t cmd)
{
  return (cmd == COMMAND_CODE_MOTION || cmd == COMMAND_CODE_REFEREE);
}

uint8_t* SerialCommunicationClass::receiveRefereeFrame()
{
  return referee_frame_buffer_.data();
}

bool SerialCommunicationClass::hasNewRefereeFrame() const
{
  return referee_frame_ready_.load();
}

void SerialCommunicationClass::clearRefereeFrameFlag()
{
  referee_frame_ready_.store(false);
}

// ---------------- 构造/析构 ----------------
SerialCommunicationClass::SerialCommunicationClass(
  rclcpp::Node* node,
  const std::string& serial_port,
  int baud_rate)
: node_(node)
{
  serial_port_ = serial_port;
  baud_rate_ = baud_rate;

  if (!serial_port_.empty()) {
    openSerialPort(serial_port_, baud_rate_);
  } else {
    std::cout << termcolor::yellow
              << "No serial port specified, auto-detecting..."
              << termcolor::reset << std::endl;
    std::string port = findSerialPort();
    if (!port.empty()) {
      openSerialPort(port, baud_rate_);
    } else {
      std::cerr << termcolor::red
                << "No serial port found."
                << termcolor::reset << std::endl;
    }
  }

  running_ = true;
  timer_thread_ = std::thread(&SerialCommunicationClass::timerThread, this);

  last_received_time_ = std::chrono::steady_clock::now();
  last_reconnect_time_ = std::chrono::steady_clock::now();
}

SerialCommunicationClass::~SerialCommunicationClass()
{
  running_ = false;
  if (timer_thread_.joinable()) {
    timer_thread_.join();
  }
  if (fd_ >= 0) {
    close(fd_);
  }
}

void SerialCommunicationClass::openSerialPort(const std::string& port_name, int baud_rate)
{
  fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    std::cerr << termcolor::red
              << "Failed to open serial port: " << strerror(errno)
              << termcolor::reset << std::endl;
  } else {
    printf("\033[32mSerial port opened: %s\033[0m\n", port_name.c_str());
    configureSerialPort(baud_rate);
    printf("\033[32mSerial initialized: %s\033[0m\n", port_name.c_str());
  }
}

std::string SerialCommunicationClass::findSerialPort()
{
  DIR* dir = opendir("/dev");
  if (!dir) {
    std::cerr << termcolor::red
              << "Failed to open /dev directory"
              << termcolor::reset << std::endl;
    return "";
  }

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strstr(entry->d_name, "ttyUSB") != nullptr ||
        strstr(entry->d_name, "ttyACM") != nullptr) {
      closedir(dir);
      return std::string("/dev/") + entry->d_name;
    }
  }
  closedir(dir);
  return "";
}

void SerialCommunicationClass::configureSerialPort(int baud_rate)
{
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    std::cerr << termcolor::red
              << "Failed to get serial attributes"
              << termcolor::reset << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  speed_t speed;
  switch (baud_rate) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    default:
      std::cerr << termcolor::red
                << "Unsupported baud rate: " << baud_rate
                << termcolor::reset << std::endl;
      return;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::cerr << termcolor::red
              << "Failed to set serial attributes"
              << termcolor::reset << std::endl;
    close(fd_);
    fd_ = -1;
    return;
  }

  tcflush(fd_, TCIOFLUSH);
}

// 帧结构：42 52 | CMD | LEN(n) | PAYLOAD[n] | CRC8
// 这里上位机主动发给下位机的仍然是旧运动控制帧，所以命令码继续用 0xCD
void SerialCommunicationClass::sendDataFrame(const uint8_t* data, size_t len)
{
  if (fd_ < 0) {
    std::cerr << termcolor::red
              << "Serial port not available"
              << termcolor::reset << std::endl;
    return;
  }

  const size_t frame_len = len + FRAME_MIN_SIZE;
  std::vector<uint8_t> frame(frame_len);

  frame[0] = FRAME_HEADER1;
  frame[1] = FRAME_HEADER2;
  frame[2] = COMMAND_CODE_MOTION;
  frame[3] = static_cast<uint8_t>(len);
  std::memcpy(&frame[4], data, len);

  frame[frame_len - 1] = crc8_calc(frame.data(), 4 + len);

  const ssize_t written = write(fd_, frame.data(), frame_len);
  if (written != static_cast<ssize_t>(frame_len)) {
    std::cerr << termcolor::red
              << "TX failed " << written << "/" << frame_len
              << termcolor::reset << std::endl;
  }
}

uint8_t* SerialCommunicationClass::receiveDataFrame()
{
  return frame_buffer_.data();
}

// ---------------- 接收：将有效帧交给 processFrame ----------------
void SerialCommunicationClass::processBuffer()
{
  size_t frames_processed = 0;

  while (buffer_index_ >= FRAME_MIN_SIZE &&
         frames_processed < MAX_FRAMES_PER_LOOP) {
    size_t pos = 0;
    bool found = false;

    while (pos + 2 <= buffer_index_) {
      if (buffer_[pos] == FRAME_HEADER1 && buffer_[pos + 1] == FRAME_HEADER2) {
        found = true;
        break;
      }
      ++pos;
    }

    if (!found) {
      buffer_index_ = 0;
      return;
    }

    if (pos > 0) {
      std::memmove(buffer_.data(), buffer_.data() + pos, buffer_index_ - pos);
      buffer_index_ -= pos;
    }

    if (buffer_index_ < 4) {
      return;
    }

    const uint8_t cmd = buffer_[2];
    if (!isSupportedCommand(cmd)) {
      std::cout << termcolor::yellow
                << "Unknown CMD=0x" << std::hex
                << static_cast<int>(cmd)
                << " (expected 0x"
                << static_cast<int>(COMMAND_CODE_MOTION)
                << " or 0x"
                << static_cast<int>(COMMAND_CODE_REFEREE)
                << ")"
                << std::dec << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    const uint8_t len = buffer_[3];
    const size_t frame_len = static_cast<size_t>(len) + FRAME_MIN_SIZE;

    if (frame_len > BUFFER_SIZE || frame_len > MAX_FRAME_LEN) {
      std::cout << termcolor::red
                << "Invalid frame length: " << frame_len
                << ", drop buffer"
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    if (buffer_index_ < frame_len) {
      return;
    }

    const uint8_t calc = crc8_calc(buffer_.data(), 4 + len);
    if (calc != buffer_[frame_len - 1]) {
      std::cout << termcolor::yellow
                << "CRC8 failed: calc=0x" << std::hex
                << static_cast<int>(calc)
                << ", recv=0x"
                << static_cast<int>(buffer_[frame_len - 1])
                << ", len=" << std::dec << static_cast<int>(len)
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    processFrame(buffer_.data());
    ++frames_processed;

    if (frame_len < buffer_index_) {
      std::memmove(
        buffer_.data(),
        buffer_.data() + frame_len,
        buffer_index_ - frame_len);
      buffer_index_ -= frame_len;
    } else {
      buffer_index_ = 0;
    }
  }
}

// ---------------- 对有效帧做分发 ----------------
void SerialCommunicationClass::processFrame(const uint8_t* data)
{
  const uint8_t cmd = data[2];
  const uint8_t len = data[3];
  const uint8_t* pl = &data[4];

  if (cmd == COMMAND_CODE_REFEREE) {
    if (len == referee_frame_buffer_.size()) {
      std::memcpy(referee_frame_buffer_.data(), pl, referee_frame_buffer_.size());
      referee_frame_ready_.store(true);
    } else {
      std::cout << termcolor::yellow
                << "Referee frame len mismatch: " << static_cast<int>(len)
                << " (expected " << referee_frame_buffer_.size() << ")"
                << termcolor::reset << std::endl;
    }
    return;
  }

  if (cmd == COMMAND_CODE_MOTION) {
    if (len <= frame_buffer_.size()) {
      std::memcpy(frame_buffer_.data(), pl, len);
    } else {
      std::cout << termcolor::yellow
                << "Motion frame too long: " << static_cast<int>(len)
                << " (buffer " << frame_buffer_.size() << ")"
                << termcolor::reset << std::endl;
    }
    return;
  }

  std::cout << termcolor::yellow
            << "Unknown CMD=0x" << std::hex
            << static_cast<int>(cmd)
            << " LEN=" << std::dec << static_cast<int>(len)
            << " (ignored)"
            << termcolor::reset << std::endl;
}

void SerialCommunicationClass::timerCallback()
{
  if (fd_ < 0) {
    if (std::chrono::steady_clock::now() - last_reconnect_time_ >
        std::chrono::seconds(3)) {
      std::cerr << "Serial port not available, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  if (std::chrono::steady_clock::now() - last_received_time_ >
      std::chrono::seconds(3)) {
    if (std::chrono::steady_clock::now() - last_reconnect_time_ >
        std::chrono::seconds(3)) {
      std::cerr << "No data received, trying reconnect" << std::endl;
      tryReconnect();
    }
    return;
  }

  if (buffer_index_ >= BUFFER_SIZE - 64) {
    std::cout << termcolor::yellow
              << "Buffer near full, clearing"
              << termcolor::reset << std::endl;
    buffer_index_ = 0;
  }

  uint8_t temp[128];
  const ssize_t n = read(fd_, temp, sizeof(temp));
  if (n > 0) {
    last_received_time_ = std::chrono::steady_clock::now();

    if (buffer_index_ + static_cast<size_t>(n) > BUFFER_SIZE) {
      std::cout << termcolor::red
                << "Buffer overflow, drop"
                << termcolor::reset << std::endl;
      buffer_index_ = 0;
      return;
    }

    std::memcpy(buffer_.data() + buffer_index_, temp, static_cast<size_t>(n));
    buffer_index_ += static_cast<size_t>(n);
    processBuffer();
  }
}

void SerialCommunicationClass::tryReconnect()
{
  last_reconnect_time_ = std::chrono::steady_clock::now();

  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }

  buffer_index_ = 0;
  referee_frame_ready_.store(false);

  if (!serial_port_.empty()) {
    openSerialPort(serial_port_, baud_rate_);
  } else {
    std::cout << termcolor::yellow
              << "No serial port specified, auto-detecting..."
              << termcolor::reset << std::endl;
    std::string port = findSerialPort();
    if (!port.empty()) {
      openSerialPort(port, baud_rate_);
    } else {
      std::cerr << termcolor::red
                << "waiting..."
                << termcolor::reset << std::endl;
    }
  }

  last_reconnect_time_ = std::chrono::steady_clock::now();
  last_received_time_ = std::chrono::steady_clock::now();
}

void SerialCommunicationClass::timerThread()
{
  while (running_) {
    const auto start = std::chrono::steady_clock::now();
    timerCallback();
    std::this_thread::sleep_until(start + std::chrono::microseconds(1000));
  }
}

void SerialCommunicationClass::writeFloatLE(uint8_t *dst, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(float));

  dst[0] = static_cast<uint8_t>(bits & 0xFFu);
  dst[1] = static_cast<uint8_t>((bits >> 8) & 0xFFu);
  dst[2] = static_cast<uint8_t>((bits >> 16) & 0xFFu);
  dst[3] = static_cast<uint8_t>((bits >> 24) & 0xFFu);
}

float SerialCommunicationClass::readFloatLE(const uint8_t *src)
{
  const uint32_t bits =
    (static_cast<uint32_t>(src[0])) |
    (static_cast<uint32_t>(src[1]) << 8) |
    (static_cast<uint32_t>(src[2]) << 16) |
    (static_cast<uint32_t>(src[3]) << 24);

  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(float));
  return value;
}