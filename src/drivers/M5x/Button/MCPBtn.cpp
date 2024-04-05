#include "Button.h"


MCPBtn::MCPBtn(uint8_t pin, uint8_t invert) : ButtonGeneral(pin, invert) {
  statePair.val1 = _state << _pin;
  statePair.val2 = _state << _pin;
}


void MCPBtn::setState(ButtonDebounceState pair) {
  log_d("BTN %u new state %u", _pin, (pair.val1 >> _pin) & 1);
  memmove(&statePair, &pair, sizeof(ButtonDebounceState));

  uint32_t ms = millis();
  _lastTime = _time;
  _time = ms;

  // Debounce: check if the state hasn't been changed in 10 ms
  // and also check that the new state != old state of this button
  if (stateChanged(pair)) {
    _changed = 1;
    _lastChange = ms;
    updateStateFromPair(pair);
    if (isPressed()) { _pressTime = _time; }
  } else {
    _changed = 0;
  }

}


bool MCPBtn::stateChanged(ButtonDebounceState val_pair) {
  uint16_t val1 = (val_pair.val1 >> _pin) & 1;
  uint16_t val2 = (val_pair.val2 >> _pin) & 1;
  if ((val1 == val2) && (val1 != _state)) {
    log_d("BTN %u state changed", _pin);
  }
  return (val1 == val2) && (val1 != _state);
}


void MCPBtn::updateStateFromPair(ButtonDebounceState val_pair) {
  uint8_t pinVal = (val_pair.val1 >> _pin) & 1;
  _lastState = _state;
  _state = pinVal;
}


/*----------------------------------------------------------------------*
 * read() returns the state of the button, 1==pressed, 0==released,     *
 * does debouncing, captures and maintains times, previous states, etc. *
 *----------------------------------------------------------------------*/
uint8_t MCPBtn::read(void) {
  // uint32_t ms = millis();
  // _lastTime = _time;
  // _time = ms;

  // // Debounce: check if the state hasn't been changed in 10 ms
  // // and also check that the new state != old state of this button
  // if (stateChanged(statePair)) {
  //   _changed = 1;
  //   _lastChange = ms;
  //   updateStateFromPair(statePair);
  //   if (isPressed()) { _pressTime = _time; }
  // } else {
  //   _changed = 0;
  // }

  return _state;
}
