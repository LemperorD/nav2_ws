#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <functional>
#include <string>
#include <thread>

using Array25 = std::array<uint8_t, 25>;

// ================= BR 串口协议常量 =================
// SOF = 'B' 'R'  (0x42 0x52)
static constexpr uint8_t FRAME_HEADER1 = 0x42;
static constexpr uint8_t FRAME_HEADER2 = 0x52;

// 自定义命令码：25字节裸数组（两端统一）
static constexpr uint8_t COMMAND_CODE_ARRAY25 = 0xCD;

// 缓冲与帧尺寸（最小帧 = 2B SOF + 1B CMD + 1B LEN + 1B CRC8 = 5）
static constexpr size_t BUFFER_SIZE    = 256;
static constexpr size_t FRAME_MIN_SIZE = 5;
static constexpr size_t MAX_FRAME_LEN  = 128;
static constexpr size_t MAX_FRAMES_PER_LOOP = 10; // 避免长时间占用：一次最多处理10帧

class SerialCommunicationClass {
public:
  explicit SerialCommunicationClass(rclcpp::Node* node, const std::string& serial_port = "", int baud_rate = 115200);
  ~SerialCommunicationClass();

  void writeFloatLE(uint8_t *dst, float value);
  float readFloatLE(const uint8_t *src);

  void sendDataFrame(const uint8_t* data, size_t len);
  uint8_t* receiveDataFrame();

private:
  // 轮询线程（~1ms）
  void timerThread();
  void timerCallback();

  // 解析
  void processBuffer();
  void processFrame(const uint8_t* data);

  // ---- CRC8（电控同款：RM常用表驱动，poly=0x31, init=0xFF） ----
  static uint8_t crc8_calc(const uint8_t* p, size_t len);

private:
  void openSerialPort(const std::string& port_name, int baud_rate);
  std::string findSerialPort();
  void configureSerialPort(int baud_rate);

  rclcpp::Node* node_{nullptr};
  int fd_ = -1;
  bool running_ = false;
  std::thread timer_thread_;
  std::array<uint8_t, BUFFER_SIZE> buffer_{};
  size_t buffer_index_ = 0;
  std::vector<uint8_t> frame_buffer_{};
};