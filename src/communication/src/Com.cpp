#include "communication/Com.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <dirent.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

// ---------------- CRC8（RM常用表驱动：poly=0x31, init=0xFF） ----------------
static const uint8_t CRC8_INIT = 0xFF;
static const uint8_t CRC8_TABLE[256] = {
  0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
  0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
  0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
  0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
  0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
  0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
  0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
  0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
  0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
  0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
  0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
  0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
  0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
  0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
  0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
  0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35
};

uint8_t SerialCommunicationClass::crc8_calc(const uint8_t* p, size_t len) {
  uint8_t crc = CRC8_INIT;
  while (len--) crc = CRC8_TABLE[crc ^ *p++];
  return crc;
}

// ---------------- 构造/析构 ----------------
SerialCommunicationClass::SerialCommunicationClass(rclcpp::Node* node, const std::string& serial_port)
  : node_(node)
{
  if (!serial_port.empty()) {
    openSerialPort(serial_port);
  } else {
    std::string port = findSerialPort();
    if (!port.empty()) {
      openSerialPort(port);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "No serial port found.");
    }
  }

  running_ = true;
  timer_thread_ = std::thread(&SerialCommunicationClass::timerThread, this);
}

SerialCommunicationClass::~SerialCommunicationClass() {
  running_ = false;
  if (timer_thread_.joinable()) timer_thread_.join();
  if (fd_ >= 0) ::close(fd_);
}

void SerialCommunicationClass::openSerialPort(const std::string& port_name)
{
    fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == -1) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open serial port: %s", strerror(errno));
    } else {
        RCLCPP_INFO(node_->get_logger(), "Serial port opened: %s", port_name.c_str());
        configureSerialPort();
    }
}

std::string SerialCommunicationClass::findSerialPort()
{
    DIR* dir = opendir("/dev");
    if (!dir) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open /dev directory");
        return "";
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, "ttyUSB") != nullptr || strstr(entry->d_name, "ttyACM") != nullptr) {
            closedir(dir);
            return std::string("/dev/") + entry->d_name;
        }
    }
    closedir(dir);
    return "";
}

void SerialCommunicationClass::configureSerialPort()
{
    struct termios options;
    tcgetattr(fd_, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    tcsetattr(fd_, TCSANOW, &options);
}

// 帧结构（BR协议）：42 52 | CMD(0xE1) | LEN(n) | PAYLOAD[n] | CRC8
void SerialCommunicationClass::sendDataFrame(const uint8_t* data, size_t len)
{
  if (fd_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Serial port not available");
  }

  size_t frame_len = len + FRAME_MIN_SIZE;

  uint8_t frame[frame_len];
  frame[0] = FRAME_HEADER1;
  frame[1] = FRAME_HEADER2;
  frame[2] = COMMAND_CODE_ARRAY25;
  frame[3] = static_cast<uint8_t>(len);
  std::memcpy(&frame[4], data, len);

  // CRC8 覆盖 Header(4) + Payload(25)
  frame[frame_len - 1] = crc8_calc(frame, 4 + len);

  ::write(fd_, frame, sizeof(frame));
}

uint8_t* SerialCommunicationClass::receiveDataFrame()
{
    return frame_buffer_.data();
}

// ---------------- 接收：将有效帧交给 processFrame ----------------
void SerialCommunicationClass::processBuffer() {

  size_t frames_processed = 0;

  while (buffer_index_ >= FRAME_MIN_SIZE && frames_processed < MAX_FRAMES_PER_LOOP) {
    // 1) 找 SOF
    size_t pos = 0;
    bool found = false;
    while (pos + 2 <= buffer_index_) {
      if (buffer_[pos] == FRAME_HEADER1 && buffer_[pos + 1] == FRAME_HEADER2) {
        found = true;
        break;
      }
      ++pos;
    }

    if (!found) { buffer_index_ = 0; return; }

    // 丢掉头前垃圾
    if (pos > 0) {
      std::memmove(buffer_.data(), buffer_.data() + pos, buffer_index_ - pos);
      buffer_index_ -= pos;
    }

    // 至少要有 Header(4) 才能读 LEN
    if (buffer_index_ < 4) return;

    const uint8_t len = buffer_[3];
    const size_t frame_len = static_cast<size_t>(len) + FRAME_MIN_SIZE; // 4+len+1

    // 合理性检查
    if (frame_len > BUFFER_SIZE || frame_len > MAX_FRAME_LEN) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid frame length: %zu, drop buffer", frame_len);
      buffer_index_ = 0;
      return;
    }

    if (buffer_index_ < frame_len) return; // 等待更多数据

    // CRC8 校验（Header+Payload）
    uint8_t calc = crc8_calc(buffer_.data(), 4 + len);
    if (calc != buffer_[frame_len - 1]) {
      RCLCPP_WARN(node_->get_logger(), "CRC8 failed: calc=0x%02X, recv=0x%02X", calc, buffer_[frame_len - 1]);
      // 丢弃一个字节，继续
      std::memmove(buffer_.data(), buffer_.data() + 1, buffer_index_ - 1);
      buffer_index_ -= 1;
      continue;
    }

    // 交给帧处理
    processFrame(buffer_.data(), frame_len);
    ++frames_processed;

    // 移走已处理的帧
    if (frame_len < buffer_index_) {
      std::memmove(buffer_.data(), buffer_.data() + frame_len, buffer_index_ - frame_len);
      buffer_index_ -= frame_len;
    } else {
      buffer_index_ = 0;
    }
  }
}

// ---------------- 对有效帧做分发 ----------------
void SerialCommunicationClass::processFrame(const uint8_t* data, size_t n) {
  (void)n;
  const uint8_t cmd = data[2];
  const uint8_t len = data[3];
  const uint8_t* pl = &data[4];

  if (cmd == COMMAND_CODE_ARRAY25 && len == 25) {
    std::memcpy(frame_buffer_.data(), pl, 25);
    return;
  }

  // 常见电控上报：0xCD / 15 —— 先不上结构化，避免刷 Unknown 日志
  if (cmd == 0xCD && len == 15) {
    return;
  }

  RCLCPP_WARN(node_->get_logger(), "Unknown CMD=0x%02X LEN=%u (ignored)", cmd, len);
}

void SerialCommunicationClass::timerCallback() {
  if (fd_ < 0) return;

  if (buffer_index_ >= BUFFER_SIZE - 64) {
    RCLCPP_WARN(node_->get_logger(), "Buffer near full, clearing");
    buffer_index_ = 0;
  }

  uint8_t temp[128];
  ssize_t n = ::read(fd_, temp, sizeof(temp));
  if (n > 0) {
    size_t copy = static_cast<size_t>(n);
    if (buffer_index_ + copy > BUFFER_SIZE) {
      RCLCPP_WARN(node_->get_logger(), "Buffer overflow, drop");
      buffer_index_ = 0;
      return;
    }
    std::memcpy(buffer_.data() + buffer_index_, temp, copy);
    buffer_index_ += copy;
    processBuffer();
  }
}

void SerialCommunicationClass::timerThread() {
  while (running_) {
    auto start = std::chrono::steady_clock::now();
    timerCallback();
    std::this_thread::sleep_until(start + std::chrono::microseconds(1000)); // ~1ms
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
  uint32_t bits =
      (static_cast<uint32_t>(src[0])) |
      (static_cast<uint32_t>(src[1]) << 8) |
      (static_cast<uint32_t>(src[2]) << 16) |
      (static_cast<uint32_t>(src[3]) << 24);

  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(float));
  return value;
}
