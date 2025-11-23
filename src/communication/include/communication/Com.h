#pragma once
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <functional>
#include <string>
#include <thread>

// ================= BR 串口协议常量 =================
// SOF = 'B' 'R'  (0x42 0x52)
static constexpr uint8_t FRAME_HEADER1 = 0x42;
static constexpr uint8_t FRAME_HEADER2 = 0x52;

// 自定义命令码：25字节裸数组（两端统一）
static constexpr uint8_t COMMAND_CODE_ARRAY25 = 0xE1;

// 缓冲与帧尺寸（最小帧 = 2B SOF + 1B CMD + 1B LEN + 1B CRC8 = 5）
static constexpr size_t BUFFER_SIZE    = 256;
static constexpr size_t FRAME_MIN_SIZE = 5;
static constexpr size_t MAX_FRAME_LEN  = 128;  // 防御性限制

class SerialCommunicationClass {
public:
  // 接收回调：把收到的 25 字节数组交给上层
  using Array25Callback = std::function<void(const std::array<uint8_t,25>&)>;
  // 原始帧回调：把任意 CMD/LEN 的负载直接上抛（抓包/调试用）
  using RawFrameCallback = std::function<void(uint8_t /*cmd*/, const uint8_t* /*payload*/, uint8_t /*len*/)>;

  explicit SerialCommunicationClass(rclcpp::Node* node);
  ~SerialCommunicationClass();

  // 发送一帧 25 字节数组（PC->电控）
  bool sendArray25(const std::array<uint8_t,25>& payload);

  // 设置接收回调（电控->PC）
  void setArray25Callback(Array25Callback cb) { array25_callback_ = std::move(cb); }
  void setRawFrameCallback(RawFrameCallback cb) { raw_callback_ = std::move(cb); }

private:
  // 串口初始化/找口
  void initializeSerial();
  std::string findAvailableSerialPort();

  // 轮询线程（~1ms）
  void timerThread();
  void timerCallback();

  // 解析
  void processBuffer();
  void processFrame(const uint8_t* data, size_t n);

  // ---- CRC8（电控同款：RM常用表驱动，poly=0x31, init=0xFF） ----
  static uint8_t crc8_calc(const uint8_t* p, size_t len);

private:
  rclcpp::Node* node_{nullptr};
  int fd_ = -1;
  bool running_ = false;
  std::thread timer_thread_;

  std::array<uint8_t, BUFFER_SIZE> buffer_{};
  size_t buffer_index_ = 0;

  Array25Callback array25_callback_{};
  RawFrameCallback raw_callback_{};
};