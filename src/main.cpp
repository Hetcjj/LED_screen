#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>
#include <Adafruit_NeoPixel.h>

// -----------------------------------------------------------------------------
// 硬件与参数配置
// -----------------------------------------------------------------------------
#define PIN_I2S_BCLK 2
#define PIN_I2S_WS   3
#define PIN_I2S_SD   10
#define I2S_PORT     I2S_NUM_0
#define SAMPLE_RATE  16000
#define BUF_PAIRS    128

// WS2812 参数
#define LED_PIN      5      // WS2812 数据引脚（自行修改）
#define LED_COLS     10     // 10 列
#define LED_ROWS     8      // 8 行
#define NUM_LEDS     (LED_COLS * LED_ROWS)

// 全局样本数组
static float rightSamples[BUF_PAIRS];
#define HISTORY_SIZE 10
static float amplitudeHistory[HISTORY_SIZE] = {0.0f};

// 初始化 WS2812
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// -----------------------------------------------------------------------------
// I2S 初始化函数
// -----------------------------------------------------------------------------
void setupI2S()
{
  Serial.begin(115200);

  i2s_config_t cfg = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = BUF_PAIRS * 2,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  i2s_pin_config_t pins = {
      .bck_io_num = PIN_I2S_BCLK,
      .ws_io_num = PIN_I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = PIN_I2S_SD};

  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
}

// -----------------------------------------------------------------------------
// 简单一阶高通滤波（去直流）
// -----------------------------------------------------------------------------
float removeDC(float x, float alpha = 0.995f)
{
  static float dc = 0.0f;
  dc = alpha * dc + (1.0f - alpha) * x;
  return x - dc;
}

// -----------------------------------------------------------------------------
// WS2812 控制：显示 10x8 音量柱
// -----------------------------------------------------------------------------
// 绘制 10列 × 8行 音量柱（相邻行方向相同）
void drawLevels(int ledLevels[HISTORY_SIZE])
{
  strip.clear();

  // Hue 随时间滚动实现彩虹流动（每毫秒偏移 5 单位）
  uint16_t baseHue = (millis() * 5) % 65536;

  for (int col = 0; col < LED_COLS; col++)
  {
    int level = ledLevels[col]; // 当前列的亮灯数量（0~8）
    if (level <= 0) continue;

    // 每列稍微偏移一点色相，形成横向彩虹带
    uint16_t colHueOffset = col * 6000;

    for (int row = 0; row < LED_ROWS; row++)
    {
      int realRow = LED_ROWS - 1 - row;  // 从下往上显示
      int idx = realRow * LED_COLS + col;

      // 只点亮在当前音量等级以内的灯
      if (row < level)
      {
        // 垂直方向色相变化（row 越高颜色越热）
        uint16_t hue = baseHue + colHueOffset + row * 900;

        // 用 gamma 校正，让渐变更平滑自然
        uint32_t color = strip.gamma32(strip.ColorHSV(hue, 255, 255));

        // 行越高亮度越高（平滑渐变）
        uint8_t brightness = map(row, 0, LED_ROWS - 1, 200, 255);
        uint8_t r = (uint8_t)(((color >> 16) & 0xFF) * brightness / 255);
        uint8_t g = (uint8_t)(((color >> 8) & 0xFF) * brightness / 255);
        uint8_t b = (uint8_t)((color & 0xFF) * brightness / 255);

        strip.setPixelColor(idx, r, g, b);
      }
      else
      {
        strip.setPixelColor(idx, 0, 0, 0);
      }
    }
  }

  strip.show();
}
// -----------------------------------------------------------------------------
// Arduino 初始化
// -----------------------------------------------------------------------------
void setup()
{
  setupI2S();
  strip.begin();
  strip.clear();
  strip.show();
  Serial.println("I2S + WS2812 Volume Meter Started");
}

// -----------------------------------------------------------------------------
// 主循环
// -----------------------------------------------------------------------------
void loop()
{
  const int maxWords = BUF_PAIRS * 2;
  static int32_t buf[maxWords];
  size_t bytes_read = 0;

  // 从 I2S 读取音频
  i2s_read(I2S_PORT, (void *)buf, sizeof(buf), &bytes_read, portMAX_DELAY);
  size_t count = bytes_read / sizeof(int32_t);
  size_t frames = count / 2;
  if (frames > BUF_PAIRS) frames = BUF_PAIRS;

  // 提取一个声道
  for (size_t i = 0, j = 0; i + 1 < count && j < frames; i += 2, ++j)
  {
    rightSamples[j] = (float)(buf[i] >> 8);
  }

  // 去直流分量
  for (size_t j = 0; j < frames; ++j)
  {
    rightSamples[j] = removeDC(rightSamples[j]);
  }

  // 计算平均振幅
  float sum = 0.0f;
  for (size_t j = 0; j < frames; ++j)
  {
    sum += fabsf(rightSamples[j]);
  }
  float meanAmplitude = sum / frames;

  // 保存历史振幅
  for (int i = 0; i < HISTORY_SIZE - 1; ++i)
  {
    amplitudeHistory[i] = amplitudeHistory[i + 1];
  }
  amplitudeHistory[HISTORY_SIZE - 1] = meanAmplitude;
// 对数映射到灯数量
static int ledLevels[HISTORY_SIZE] = {0};
for (int i = 0; i < HISTORY_SIZE; ++i)
{
  float amp = amplitudeHistory[i];
  const float EPS = 1e-6f;
  const float refAmp = 1.0e6f;   // 你原来的参考值
  float norm = amp / refAmp;
  float db = 20.0f * log10f(norm + EPS);

  // ====== 新增：底噪门限（自己调） ======
  const float noiseFloorDb = -53.0f;  // 小于这个当静音，先试 -45dB 或 -50dB
  int level;
  if (db <= noiseFloorDb) {
    level = 0;   // 完全不亮
  } else {
    // 只把 noiseFloorDb ~ 0dB 映射到 0~LED_ROWS
    level = map((int)db, (int)noiseFloorDb, 0, 0, LED_ROWS);
  }
  // ====================================

  level = constrain(level, 0, LED_ROWS);
  ledLevels[i] = level;
}
  // 显示在灯板上
  drawLevels(ledLevels);



  delay(50); // 控制刷新频率（约33Hz）
}
