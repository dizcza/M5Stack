#include <M5StX.h>

// Waveform, Frequency, Attack, Decay, Sustain, Release, Gain
Synth c5(SINE, NOTE_C5, 50, 300, 0.7, 1000);
Synth e5(SINE, NOTE_E5, 50, 300, 0.7, 1000);
Synth g5(SINE, NOTE_G5, 50, 300, 0.7, 1000);

void setup() {
  M5.begin();
}

void loop() {
  M5.update();
  if(M5.BtnA.wasPressed()) playChord();
}

void playChord() {
  c5.playFor(5000);
  Sound.delay(1000);
  e5.playFor(4000);
  Sound.delay(1000);
  g5.playFor(3000);
  Sound.waitForSilence();
}
