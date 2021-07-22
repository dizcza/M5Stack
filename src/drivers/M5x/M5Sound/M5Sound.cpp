#include "M5Sound.h"

#include <M5StX.h>
#include <FreeRTOS.h>
#include <driver/i2s.h>
#include "utility/Config.h"
#if defined (ARDUINO_M5STACK_Core2)
  #include "drivers/M5x/AXP192/AXP192.h"
#endif
#if defined (ARDUINO_ESP32_DEV) //M35
  #include "drivers/M5x/WM8978/WM8978.h"
#endif


/////////////////////////////////////////////////////////////////////////////
//
// M5SoundSource
//
/////////////////////////////////////////////////////////////////////////////

/* static */ std::vector<M5SoundSource*> M5SoundSource::instances;

M5SoundSource::M5SoundSource() {
  instances.push_back(this);
}

M5SoundSource::~M5SoundSource() {
  for (int i = 0; i < instances.size(); ++i) {
    if (instances[i] == this) {
      instances.erase(instances.begin() + i);
      return;
    }
  }
}

/* virtual */ uint16_t M5SoundSource::read(int16_t* buffer, uint16_t size) {
  return 0;
}

bool M5SoundSource::playing() {
  return _playing;
}



/////////////////////////////////////////////////////////////////////////////
//
// M5SoundClass
//
/////////////////////////////////////////////////////////////////////////////

void M5SoundClass::begin() {
  // populate sinewave table
  for (uint8_t n = 0; n < 100; n++) {
    _sineTable[n] = sin((float)n * TWO_PI / 100);
  }
  start();
}

void M5SoundClass::start() {
  #ifndef I2S_FULL_DUPLEX
    M5SOUNDIN.stop();
  #endif
  // Set up I2S driver
  i2s_driver_uninstall(TX_DEV);
  i2s_config_t tx_config = {
    .mode = TX_MODE,
    .sample_rate = samplerate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = TX_CHANNEL_FORMAT,
    .communication_format = TX_FORMAT,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = TX_DMA_BUF_COUNT,
    .dma_buf_len = TX_DMA_BUF_LEN,
    .use_apll = TX_USE_APLL,
    .tx_desc_auto_clear = true,
    .fixed_mclk = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(TX_DEV, &tx_config, 0, NULL);
  // Configure pins
	#if defined (ARDUINO_M5Stack_Core_ESP32) || defined (ARDUINO_M5STACK_FIRE)
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
  #elif defined (ARDUINO_LOLIN_D32_PRO)
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    digitalWrite(SPEAKER_EN_PIN, HIGH);
  #elif defined (ARDUINO_M5Stick_C)
    i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
    digitalWrite(SPEAKER_EN_PIN, HIGH);
  #elif defined (ARDUINO_M5Stick_C_Plus)
    i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
    digitalWrite(SPEAKER_EN_PIN, HIGH);
  #elif defined (ARDUINO_M5STACK_Core2)
    i2s_pin_config_t tx_pin_config= {
      .bck_io_num = TX_BCLK,
      .ws_io_num = TX_LRC,
      .data_out_num = TX_DOUT,
      .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(TX_DEV, &tx_pin_config);
    // Turn on system speaker
    AXP->SetSpkEnable(true);
	#elif defined (ARDUINO_ESP32_DEV) //WM8978
    i2s_pin_config_t tx_pin_config= {
      .bck_io_num = TX_BCLK,
      .ws_io_num = TX_LRC,
      .data_out_num = TX_DOUT,
      .data_in_num = RX_DIN
    };
    i2s_set_pin(TX_DEV, &tx_pin_config);
    // Configure WM8978 MCLK pin
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
    // DAC->begin();
    DAC->setSPKvol(60); /* max 63 */
    DAC->setHPvol(40, 40);
	#endif
  i2s_set_sample_rates(TX_DEV, samplerate);
  _silentSince = millis();
  amplifier(!(AMP_STANDBY), true);
  running = true;
}

void M5SoundClass::stop() {
  i2s_driver_uninstall(TX_DEV);
  amplifier(false, true);
  running  = false;
}

void M5SoundClass::update() {
  bool silent = true;
  if (running) {
    do {
      // If last packet is gone, make a new one
      if (!_bytes_left) {
        // Ask synths what they have and mix in 32-bit signed mix buffer
        memset(_mixbuf, 0, TX_CHUNKSIZE * 4);   // 32 bits
        for (auto source : M5SoundSource::instances) {
          if (source->playing() && source->read(_tmpbuf, TX_CHUNKSIZE)) {
            silent = false;
            for (uint16_t i = 0; i < TX_CHUNKSIZE; i++) {
              _mixbuf[i] += _tmpbuf[i];
            }
          }
        }
        for (uint16_t i = 0; i < TX_CHUNKSIZE; i++) {
          int32_t m = _mixbuf[i];
          // clip to 16-bit signed so we get "real" distortion not noise
          m = max(min(m, 32767), -32768);
          _outbuf[i * TX_CHANNELS] = m;
          if (TX_CHANNELS == 2) {
            _outbuf[(i * TX_CHANNELS) + 1] = m;
          }
        }
        _bytes_left = TX_CHUNKSIZE * 2 * TX_CHANNELS;
      }
      if (_bytes_left) {
        // Send what can be sent but don't hang around, send rest next time.
        size_t bytes_written = 0;
        i2s_write(TX_DEV, _outbuf + (TX_CHUNKSIZE * 2 * TX_CHANNELS) - _bytes_left, _bytes_left,
                  &bytes_written, 0);
        _bytes_left -= bytes_written;
      }
    } while (!_bytes_left);
  }

  // Processing that happens even if not 'running': amplifier timeout
  if (!silent) {
    _silentSince = 0;
    amplifier(true);
  }
  if (silent && !_silentSince) _silentSince = millis();
  if (AMP_STANDBY && _silentSince && millis() - _silentSince > AMP_STANDBY) {
    amplifier(false);
  }

  // Call recording sink if present
  if(M5SoundSink::recordingInstance) {
    M5SoundSink::recordingInstance->update();
  }
}

void M5SoundClass::delay(uint16_t msec) {
  uint32_t start = millis();
  while (millis() - start < msec) {
    update();
    yield();
  }
}

void M5SoundClass::waitForSilence(uint16_t msec /* = 0 */) {
  while (!silence(msec)) {
    update();
    yield();
  }
}

bool M5SoundClass::silence(uint16_t msec /* = 0 */) {
  if (!_silentSince) return false;
  if (millis() - _silentSince >= msec) return true;
  return false;
}

void M5SoundClass::amplifier(bool state, bool force /* = false */) {
  if (state && (!_amp_on || force)) {
    log_d("Audio amplifier on");
    #if defined (ARDUINO_M5STACK_Core2)
      AXP->SetSpkEnable(true);
    #elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3
      digitalWrite(SPEAKER_EN_PIN, HIGH);
    #elif defined (ARDUINO_ESP32_DEV) //M35
      DAC->setSPKvol(60); /* max 63 */
      DAC->setHPvol(40, 40);
    #endif
    _amp_on = true;
  }
  if (!state && (_amp_on || force)) {
    log_d("Audio amplifier off");
    #if defined (ARDUINO_M5STACK_Core2)
      AXP->SetSpkEnable(false);
    #elif defined (ARDUINO_M5Stick_C) || defined (ARDUINO_LOLIN_D32_PRO) //TTGO T4 v1.3
      digitalWrite(SPEAKER_EN_PIN, LOW);
    #elif defined (ARDUINO_ESP32_DEV) //M35
      DAC->setSPKvol(0); /* max 63 */
      DAC->setHPvol(0, 0);
    #endif
    _amp_on = false;
  }
}



/////////////////////////////////////////////////////////////////////////////
//
// Synth
//
/////////////////////////////////////////////////////////////////////////////

Synth::Synth(waveform_t waveform_ /* = SINE */, float freq_ /* = 0 */,
             uint16_t attack_ /* = ATTACK */, uint16_t decay_ /* = DECAY */,
             float sustain_ /* = SUSTAIN */, uint16_t release_ /* = RELEASE */,
             float gain_ /* = GAIN */) {
  waveform = waveform_;
  freq = freq_;
  attack = attack_;
  decay = decay_;
  sustain = sustain_;
  release = release_;
  gain = gain_;
  phase = 0;
  startTime = stopTime = 0;
}

uint16_t Synth::read(int16_t* buffer, uint16_t size) {
  if (!freq || !startTime || size != TX_CHUNKSIZE) return 0;

  // Envelope
  uint32_t duration = millis() - startTime;
  if (duration < attack) {
    envelope = _startEnvelope +
               (((float)duration / attack) * (1 - _startEnvelope));
  } else if (decay && sustain < 1 && duration < attack + decay) {
    envelope = 1 - (((float)(duration - attack) / decay) * (1 - sustain));
  } else {
    envelope = sustain;
  }
  if (stopTime && millis() > stopTime) {
    uint32_t stopping_for = millis() - stopTime;
    if (stopping_for < release) {
      envelope = sustain - (((float)stopping_for / release) * sustain);
    } else {
      startTime = stopTime = 0;
      envelope = 0;
      phase = 0;
      _playing = false;
      return 0;
    }
  }

  // Waveform
  // Gain efffect quadratic. 181 is (almost) the square root of 32768.
  uint16_t amplitude = pow((float)((gain * envelope) * 181), 2);
  float steps = ((float)freq / M5SOUND.samplerate);
  switch (waveform) {
    case SINE:
      for(uint16_t i = 0; i < size; i++) {
        uint8_t t = (uint16_t)((phase + (i * steps)) * 100) % 100;
        buffer[i] = M5SOUND._sineTable[t] * amplitude;
      }
      break;
    case SQUARE:
      for(uint16_t i = 0; i < size; i++) {
        phase + (i * steps);
        buffer[i] = (phase + (i * steps) > 50 ? -1 : 1) * amplitude;
      }
      break;
    case TRIANGLE:
      for(uint16_t i = 0; i < size; i++) {
        float t = phase + (i * steps);
        t -= (int)t;
        if (t < 0.25) t *= 4;
         else if (t < 0.75) t = 2 - (t * 4);
          else t = -4 + (t * 4);
        buffer[i] = t * amplitude;
      }
      break;
    case SAWTOOTH:
      for(uint16_t i = 0; i < size; i++) {
        float t = phase + (i * steps);
        t -= (int)t;
        if (t < 0.5) t *= 2;
         else t = -2 + (t * 2);
        buffer[i] = t * amplitude;
      }
      break;
    case NOISE:
      for(uint16_t i = 0; i < size; i++) {
        buffer[i] = (rand() % (2 * amplitude)) - amplitude;
      }
      break;
  }
  phase += (float)(size) * steps;
  phase -= (int)phase;
  return size;
}

void Synth::start() {
  startTime = millis();
  _startEnvelope = envelope;
  stopTime = 0;
  _playing = true;
}

void Synth::stop() {
  // At least attack, decay and release get to play
  stopTime = startTime + attack + decay;
  if (millis() > stopTime) stopTime = millis() + 10;
}

void Synth::playFor(uint32_t msec) {
  uint32_t duration = attack + decay + release;
  if (msec > duration) duration = msec;
  startTime = millis();
  stopTime  = millis() + duration;
  _playing = true;
}



/////////////////////////////////////////////////////////////////////////////
//
// GunShot
// Hacıhabiboğlu, H. (2017). Procedural Synthesis of Gunshot Sounds Based on
// Physically Motivated Models. In Game Dynamics (pp. 47-69). Springer, Cham.
//
/////////////////////////////////////////////////////////////////////////////


/* GunShot constructor
 *
 * @param projDiam  - the projectile diameter in mm
 * @param projLen   - the projectile length in mm
 * @param barrelLen - the barrel length in mm
 * @param exitP     - the muzzle pressure in kg/cm2
 * @param exitV     - the exit velocity of the projectile in m/s
 * @param micDist   - the distance between the gun and the microphone in m
 * @param micTheta  - the angle between the boreline and the microphone position in degree
 * @param csnd      - the speed of sound in cm/s
 */
GunShot::GunShot(gunshot_t waveform_ /* = GS_MB */,
        uint8_t projDiam_ /* = 152 */, uint16_t projLen_ /* = 860 */,
        uint16_t barrelLen_ /* = 6184 */, uint16_t exitP_ /* = 3600 */, uint16_t exitV_ /* = 560 */,
        uint16_t micDist_ /* = 10000 */, uint8_t micTheta_ /* = 15 */,
        uint16_t csnd_ /* = 33130 */, float gain_ /* = GAIN */) {
    waveform = waveform_;
    projDiam = projDiam_ / 1000.f;  // in m
    projLen = projLen_ / 1000.f;    // in m
    boreArea = M_PI * projDiam_ * projDiam_ / 4;
    barrelLen = barrelLen_ / 1000.f;// in m
    exitP = 98066.5f * exitP_;      // in Pa
    exitV = (float)exitV_;          // in m/s
    r = (float)micDist_;            // in m
    theta = micTheta_ * M_PI / 180; // in radians
    csnd = csnd_ / 100.f;
    M = exitV / csnd;               // the Mach number
    coneAngle = asin(1.f / M);
    switch (waveform) {
        case GS_MB:
            // Calculates the momentum index defined as the ratio of the sound
            // pressure at the front and at the rear of the firing position
            float peb, xmod, mu;
            peb = exitP / ATMOSPHERIC_PRESSURE;
            xmod = M * sqrt(GAMMA * peb / 2);
            mu = 0.83f - 0.0063f * xmod;

            // Calculate the scaling length `scalingLen` and the direction weighted scaling length `lp`
            // Energy deposition rate, eq. 2
            float dEdT, scalingLen, ratio, lp, rb, rb_inv, X;
            dEdT = (GAMMA * peb * exitV) / (GAMMA - 1) * (1 + (GAMMA - 1) / 2 * M * M) * boreArea;
            scalingLen = sqrt(dEdT / (ATMOSPHERIC_PRESSURE * csnd));
            // TODO find a better constant for the scaling length
            scalingLen *= 10;
            ratio = mu * cos(theta) + sqrt(1 - pow(mu * sin(theta), 2));
            // @param lp   - the direction weighted scaling length in m
            lp = scalingLen * ratio;
            rb = r / lp;
            rb_inv = lp / r;
            X = sqrt(rb * rb + 1.04f * rb + 1.88f);

            // Calculate the time of arrival of the muzzle blast in s
            float ta_norm, ta;
            ta_norm = X - 0.52f * log(2 * X + 2 * rb + 1.04f) - 0.56f;  // Eq. 27
            ta = ta_norm * lp / csnd;
            toa = (uint32_t)(ta * 1000);

            // Calculate the positive phase duration of the muzzle blast tau in s
            // and the peak overpressure of the muzzle blast Pb in Pa
            float delta, G, tau_norm, Pb;
            delta = (barrelLen * csnd) / (exitV * scalingLen);
            G = 0.09 - 0.00379 * delta + 1.07 * (1 - 1.36 * exp(-0.049 * rb)) * scalingLen / lp;  // Eq. 28
            if (rb < 50.) {
                tau_norm = rb - X + 0.52 * log(2 * X + 2 * rb + 1.04) + 0.56 + G;
                Pb = 0.89f * rb_inv + 1.61f * rb_inv * rb_inv;
            } else {
                tau_norm = 2.99 * sqrt(log(33119 * rb)) - 8.534 + G;
                Pb = 3.48975f / (rb * sqrt(log(33119 * rb)));
            }
            tau = tau_norm * lp / csnd;
            pressAmp = Pb * ATMOSPHERIC_PRESSURE;
            Td = 255;
            // We got:
            // @param ta       - the time of MB arrival in s
            // @param toa      - the time of MB arrival in ms
            // @param tau      - positive phase duration in s
            // @param pressAmp - the pressure amplitude in Pa
            break;
        case GS_SW:
            float coneAngle = asin(1.f / M);
            if ((exitV > csnd) && (coneAngle <= M_PI - theta)) {
                float xmiss = r * sin(theta);
                // Calculate the N-wave amplitude in Pa
                pmax = 0.53 * ATMOSPHERIC_PRESSURE * projDiam * pow(M * M - 1, 0.125) / (pow(xmiss, 0.75) * pow(projLen, 0.25));
                // Calculate the N-wave arrival time
                float projectileTravel = r * cos(theta);
                float soundTravel = xmiss * cos(coneAngle);
                float ta = projectileTravel / exitV + soundTravel / csnd;
                toa = (uint32_t)(ta * 1000);
                // Calculate the N-wave total duration time in s
                float L = 1.82 * projDiam * M * pow(xmiss, 0.25) / (pow(M * M - 1, 0.375) * pow(projLen, 0.25));
                Td = L / csnd;
                // Calculate the N-wave rise time in s
                tr = (AIR_MOLECULAR_MEAN_FREE_PATH / csnd) * (ATMOSPHERIC_PRESSURE / pmax);
            }
            // We got: pmax, ta, Td, tr
            // @param pmax - the SW amplitude in Pa
            // @param ta   - the time of SW arrival in s
            // @param toa  - the time of SW arrival in ms
            // @param Td   - the SW total duration in s
            // @returns tr - the rise time in s
            break;
    }
    gain = gain_;
    phase = 0;
    startTime = stopTime = 0;
}

uint16_t GunShot::read(int16_t* buffer, uint16_t size) {
  if (!startTime || size != TX_CHUNKSIZE) return 0;
  //Serial.println(theta, DEC);
  if (stopTime && millis() > stopTime) {
      startTime = stopTime = 0;
      phase = 0;
      _playing = false;
      return 0;
  }
  // Waveform
  // Gain efffect quadratic. 181 is (almost) the square root of 32768.
  uint16_t amplitude = pow((float)(gain * 181), 2);
  float steps = (1.0 / M5SOUND.samplerate);
  switch (waveform) {
    case GS_MB:
        // Calculate Friedlander model of a muzzle blast wave
        // @param pressAmp - the pressure amplitude in Pa
        // @param tau      - positive phase duration in s
        float x, tauNorm;
        tauNorm = tau / steps;
        for (uint16_t i = 0; i < size; i++) {
          x = i / tauNorm;
          buffer[i] = amplitude * pressAmp * (1 - x) * exp(-x);  // Pmb
        }
        break;
    case GS_SW:
        float t, p;
        if ((exitV <= csnd) || (coneAngle > M_PI - theta)) {
            // supersonic speed is required
            memset(buffer, 0, size * sizeof(float));
            break;
        }
        for (uint16_t i = 0; i < size; i++) {
          t = phase + (i * steps);
          p = 0;
          if (t <= tr) {
              p = pmax * t / tr;
          } else if ((t > tr) && (t <= (Td - tr))) {
              p = pmax * (1 - 2 * (t - tr) / (Td - 2 * tr));
          } else if ((t > (Td - tr)) & (t < Td)) {
              p = pmax * ((t - (Td - tr)) / tr - 1);
          }
          buffer[i] = amplitude * p;
        }
        break;
  }
  phase += (float)(size) * steps;
  phase -= (int)phase;
  return size;
}

void GunShot::start() {
  startTime = millis();
  stopTime = 0;
  _playing = true;
}

void GunShot::stop() {
  // At least 250ms get to play
  stopTime = startTime + 256;
  if (millis() > stopTime) stopTime = millis() + 10;
}

void GunShot::playFor(uint32_t msec) {
  uint32_t duration = 255;
  if (msec > duration) duration = msec;
  startTime = millis();
  stopTime  = millis() + duration;
  _playing = true;
}

void GunShot::play() {
  playFor(Td);
}

/////////////////////////////////////////////////////////////////////////////
//
// M5SoundSink
//
/////////////////////////////////////////////////////////////////////////////

/* static */ M5SoundSink* M5SoundSink::recordingInstance = NULL;

M5SoundSink::~M5SoundSink() {
  if (recordingInstance == this) {
    recordingInstance = NULL;
  }
}

// virtual, overridden by inheriting classes
/* static */ void M5SoundSink::update() {}

bool M5SoundSink::recording() {
  return (recordingInstance == this);
}



/////////////////////////////////////////////////////////////////////////////
//
// M5SoundInClass
//
/////////////////////////////////////////////////////////////////////////////

void M5SoundInClass::begin() {
  #ifndef I2S_FULL_DUPLEX
    start();
  #endif
}

void M5SoundInClass::start() {
  #ifndef I2S_FULL_DUPLEX
    M5SOUND.stop();
  #endif
  i2s_driver_uninstall(RX_DEV);
  // Set up I2S driver
  i2s_config_t rx_config = {
    .mode = RX_MODE,
    .sample_rate = samplerate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = RX_CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = RX_DMA_BUF_COUNT,
    .dma_buf_len = RX_DMA_BUF_LEN,
    .use_apll = true
  };
  i2s_driver_install(RX_DEV, &rx_config, 0, NULL);
  // Configure pins
  i2s_pin_config_t rx_pin_config= {
    .bck_io_num = RX_BCLK,
    .ws_io_num = RX_LRC,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = RX_DIN
  };
  i2s_set_pin(RX_DEV, &rx_pin_config);
  i2s_set_sample_rates(RX_DEV, samplerate);
  running = true;
}

void M5SoundInClass::stop() {
  i2s_driver_uninstall(RX_DEV);
  running = false;
}

uint16_t M5SoundInClass::read(int16_t* buffer, uint16_t size) {
  if (!running) return 0;
  size_t bytes_read = 0;
  i2s_read(RX_DEV, buffer, size, &bytes_read, 0);
  return (uint16_t)bytes_read / 2;
}


M5SoundClass& Sound = M5SOUND;
M5SoundInClass& SoundIn = M5SOUNDIN;