#ifndef _M5SOUND_H_
#define _M5SOUND_H_

#include <Arduino.h>

// Full-duplex does not work on systems that share the clock pins between input and output.
// (Or so I believe. If you have it working, please file an issue to let me know how.)
// #define I2S_FULL_DUPLEX

#define SAMPLERATE  16000

#if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE) || defined (ARDUINO_LOLIN_D32_PRO)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT              2
  #define TX_BCLK             12
  #define TX_LRC              19
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN )
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB)
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         true
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              34
  #define RX_BCLK             12
  #define RX_LRC              19
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_16BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_LSB
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_M5Stick_C)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT             26
  #define TX_BCLK              0
  #define TX_LRC              36
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN )
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB)
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         true
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              34
  #define RX_BCLK              0
  #define RX_LRC              36
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_M5Stick_C_Plus)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT             26
  #define TX_BCLK              0
  #define TX_LRC              36
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN )
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB)
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         true
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              34
  #define RX_BCLK              0
  #define RX_LRC              36
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_M5STACK_Core2)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT              2
  #define TX_BCLK             12
  #define TX_LRC               0
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX)
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         true
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              34
  #define RX_BCLK             12
  #define RX_LRC               0
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_TWatch)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT             33
  #define TX_BCLK             26
  #define TX_LRC              25
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX)
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         true
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              14
  #define RX_BCLK             15
  #define RX_LRC              13
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_RIGHT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_ESP32_DEV)
  #define TX_DEV               I2S_NUM_0
  #define TX_DOUT             12  //DIN on WM8978 PCB
  #define TX_BCLK             14
  #define TX_LRC              15
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX)
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB) //Philips standard
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_LEFT
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL        false
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              13  //DOUT on WM8978 PCB
  #define RX_BCLK             14
  #define RX_LRC              15
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB) //Philips standard
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_LEFT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#elif defined (ARDUINO_Piranha) //K46 with MAX98357A
  #define TX_DEV               I2S_NUM_0
  #define TX_DOARDUINO_FROG_ESP3218
  #define TX_BCLK             19
  #define TX_LRC              12
  #define TX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX)
  #define TX_FORMAT            (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB) //Philips standard
  #define TX_CHANNELS          1
  #define TX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_LEFT  //Rsd = 0 Ohm to VDD
  #define TX_DMA_BUF_COUNT     4
  #define TX_DMA_BUF_LEN     128
  #define TX_USE_APLL         false
  #define TX_CHUNKSIZE        32

  #define RX_DEV               I2S_NUM_0
  #define RX_DIN              17
  #define RX_BCLK              0
  #define RX_LRC              12
  #define RX_MODE              (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX |I2S_MODE_PDM)
  #define RX_BPS               I2S_BITS_PER_SAMPLE_32BIT
  #define RX_FORMAT            (i2s_comm_format_t)I2S_COMM_FORMAT_I2S
  #define RX_CHANNELS          1
  #define RX_CHANNEL_FORMAT    I2S_CHANNEL_FMT_ONLY_LEFT
  #define RX_DMA_BUF_COUNT     4
  #define RX_DMA_BUF_LEN     128
#endif


// switches off amplifier chip after 1 sec. Set to 0 to keep amplifier on.
#define AMP_STANDBY     1000


// defaults
#define ATTACK            10      // prevents clicks
#define DECAY              0
#define SUSTAIN            1.0
#define RELEASE           10      // prevents clicks
#define GAIN               0.5


// Musical notes, 440 Hz tuning
#define NOTE_C3     130.81
#define NOTE_Db3    138.59
#define NOTE_D3     146.83
#define NOTE_Eb3    155.56
#define NOTE_E3     164.81
#define NOTE_F3     174.61
#define NOTE_Gb3    185.00
#define NOTE_G3     196.00
#define NOTE_Ab3    207.65
#define NOTE_A3     220
#define NOTE_Bb3    233.08
#define NOTE_B3     246.94
#define NOTE_C4     261.63
#define NOTE_Db4    277.18
#define NOTE_D4     293.66
#define NOTE_Eb4    311.13
#define NOTE_E4     329.63
#define NOTE_F4     349.23
#define NOTE_Gb4    369.99
#define NOTE_G4     392.00
#define NOTE_Ab4    415.30
#define NOTE_A4     440
#define NOTE_Bb4    466.19
#define NOTE_B4     493.88
#define NOTE_C5     523.25
#define NOTE_Db5    554.37
#define NOTE_D5     587.33
#define NOTE_Eb5    622.25
#define NOTE_E5     659.25
#define NOTE_F5     698.46
#define NOTE_Gb5    739.99
#define NOTE_G5     783.99
#define NOTE_Ab5    830.61
#define NOTE_A5     880
#define NOTE_Bb5    932.33
#define NOTE_B5     987.77
#define NOTE_C6    1046.50
#define NOTE_Db6   1108.73
#define NOTE_D6    1174.66
#define NOTE_Eb6   1244.51
#define NOTE_E6    1318.51
#define NOTE_F6    1396.91
#define NOTE_Gb6   1479.98
#define NOTE_G6    1567.98
#define NOTE_Ab6   1661.22
#define NOTE_A6    1760
#define NOTE_Bb6   1864.66
#define NOTE_B6    1975.53

enum waveform_t {
  SINE,
  SQUARE,
  TRIANGLE,
  SAWTOOTH,
  NOISE
};

class M5SoundSource {
 public:
  static std::vector<M5SoundSource*> instances;
  M5SoundSource();
  ~M5SoundSource();
  virtual uint16_t read(int16_t* buffer, uint16_t size);
  bool playing();
 protected:
  bool _playing = false;
};

#define M5SOUND M5SoundClass::instance()
class M5SoundClass{
 // Singleton stuff
 public:
  static M5SoundClass& instance() {
    static M5SoundClass INSTANCE;
    return INSTANCE;
  }
  M5SoundClass(M5SoundClass const&)    = delete;
  void operator=(M5SoundClass const&)  = delete;
 private:
  M5SoundClass() {}

 public:
  void begin();
  void start();
  void stop();
  void update();
  void reg(M5SoundSource* source);
  void dereg(M5SoundSource* source);
  void delay(uint16_t msec);
  void waitForSilence(uint16_t msec = 0);
  bool silence(uint16_t msec = 0);
  void amplifier(bool state, bool force = false);
  int samplerate = SAMPLERATE;
  bool running = false;

 protected:
  friend class Synth;   // for _sineTable
  float _sineTable[100];
  uint32_t _silentSince;
  int16_t _tmpbuf[TX_CHUNKSIZE];
  int16_t _outbuf[TX_CHUNKSIZE * TX_CHANNELS];
  int32_t _mixbuf[TX_CHUNKSIZE];
  size_t _bytes_left          = 0;
  bool _amp_on                = false;
};

extern M5SoundClass& Sound;


class Synth : public M5SoundSource {
 public:
  Synth(waveform_t waveform_ = SINE, float freq_ = 0,
          uint16_t attack_ = ATTACK, uint16_t decay_ = DECAY,
          float sustain_ = SUSTAIN, uint16_t release_ = RELEASE,
          float gain_ = GAIN);
  virtual uint16_t read(int16_t* buffer, uint16_t size);
  void start();
  void stop();
  void playFor(uint32_t msec);
  waveform_t waveform;
  float gain, freq, envelope, sustain, phase;
  uint16_t attack, decay, release;
  uint32_t startTime, stopTime;
 protected:
  float _startEnvelope;
};


class M5SoundSink {
 public:
  static M5SoundSink* recordingInstance;
  ~M5SoundSink();
  virtual void update();
  bool recording();
};


#define M5SOUNDIN M5SoundInClass::instance()
class M5SoundInClass : public M5SoundSource {
 // Singleton stuff
 public:
  static M5SoundInClass& instance() {
    static M5SoundInClass INSTANCE;
    return INSTANCE;
  }
  M5SoundInClass(M5SoundInClass const&)  = delete;
  void operator=(M5SoundInClass const&)  = delete;
 private:
  M5SoundInClass() {}

 public:
  void begin();
  void start();
  void stop();
  void update();
  virtual uint16_t read(int16_t* buffer, uint16_t size);
  int samplerate = SAMPLERATE;
  bool running = false;

 protected:
  int16_t _read_tmp[RX_DMA_BUF_LEN / 2];
};

extern M5SoundInClass& SoundIn;

#endif /* _M5SOUND_H_ */
