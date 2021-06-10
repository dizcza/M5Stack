#include <M5StX.h>

Synth a(SINE, 1000);

void setup() {
	M5.begin();
}

void loop() {
  M5.update();
  if(M5.BtnA.wasPressed()) a.start();
  if(M5.BtnB.wasPressed()) a.stop(); 
}