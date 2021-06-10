#include <M5StX.h>

// Waveform, Frequency, Attack, Decay, Sustain, Release, Gain
Synth c5(SINE, NOTE_C5, 50, 300, 0.7, 1000);
Synth e5(SINE, NOTE_E5, 50, 300, 0.7, 1000);
Synth g5(SINE, NOTE_G5, 50, 300, 0.7, 1000);

Button playButton(50, 80, 220, 80, false, "C major", {TFT_YELLOW, TFT_BLACK, NODRAW});

void setup() {
  M5.begin();
  playButton.setFont(FSSB24);
  playButton.draw();
}

void loop() {
  M5.update();
  if (playButton.wasPressed()) playChord();
}

void playChord() {
  playButton.hide(TFT_BLACK);
  c5.playFor(5000);
  Sound.delay(1000);
  e5.playFor(4000);
  Sound.delay(1000);
  g5.playFor(3000);
  Sound.waitForSilence();
  //playButton.show();
}
