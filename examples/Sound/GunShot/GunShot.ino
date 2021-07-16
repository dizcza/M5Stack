#include <M5StX.h>

// Waveforms
GunShot SW152(GS_SW, 152, 860, 6184, 3600, 560); //ShockWave
//GunShot SB152(GS_SB, 152, 860, 6184, 3600, 560); //Shell blast
GunShot MB152(GS_MB, 152, 860, 6184, 3600, 560); //Muzzle Blast

Button playButton(50, 80, 220, 80, false, "GunShot", {TFT_YELLOW, TFT_BLACK, NODRAW});

void setup() {
  M5.begin();
  playButton.setFont(FSSB24);
  playButton.draw();
}

void loop() {
  M5.update();
  if (playButton.wasPressed()) playGunShot();
}

void playGunShot() {
  playButton.hide(TFT_BLACK);
  //Sound.delay(SW152.toa);     //toa calculated for given distance and theta
  SW152.play();
  //Sound.delay(SB152.toa);
  //SB152.play();
  //Sound.delay(MB152.toa);
  Sound.delay(1000);
  MB152.play();
  Sound.waitForSilence();
}