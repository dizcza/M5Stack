#include "Button.h"


MCPBtn::MCPBtn(MCPXManager& mcpMan, uint8_t pin, uint8_t invert) : ButtonGeneral(pin, invert), mcpMan(mcpMan) {
}

bool MCPBtn::stateChanged(uint16_t val1, uint16_t val2) {
    val1 = (val1 >> _pin) & 1;
    val2 = (val2 >> _pin) & 1;
    return (val1 == val2) && (val1 != _state);
}

/*----------------------------------------------------------------------*
 * read() returns the state of the button, 1==pressed, 0==released,     *
 * does debouncing, captures and maintains times, previous states, etc. *
 *----------------------------------------------------------------------*/
uint8_t MCPBtn::read(void) {
  std::pair<uint16_t, uint16_t> val_pair = mcpMan.read();
  if (_invert != 0) val_pair = std::make_pair(val_pair.first, val_pair.second);

  uint32_t ms = millis();
  _lastTime = _time;
  _time = ms;

  // Debounce: check if the state hasn't been changed in 10 ms
  // and also check that the new state != old state of this button
  if (stateChanged(val_pair.first, val_pair.second)) {
    _changed = 1;
    _lastChange = ms;
    uint8_t pinVal = (val_pair.second >> _pin) & 1;  // same as first
    _lastState = _state;
    _state = pinVal;
    if (_state) { _pressTime = _time; }
  } else {
    _changed = 0;
  }

  return _state;
}
