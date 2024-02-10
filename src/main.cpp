#include "M5StX.h"

void setup() {
    M5.begin();
    M5.Lcd.drawCircle(30, 30, 30, TFT_RED);
}

void loop()
{
    M5.BtnA.read();
    M5.BtnB.read();
    M5.BtnC.read();
    if (M5.BtnA.wasPressed())
    {
        log_i("BTN A was pressed");
    }
    if (M5.BtnB.wasPressed())
    {
        log_i("BTN B was pressed");
    }
    if (M5.BtnC.wasPressed())
    {
        log_i("BTN C was pressed");
    }

    delay(10);
}