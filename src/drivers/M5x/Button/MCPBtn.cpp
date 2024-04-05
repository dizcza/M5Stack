#include "Button.h"


MCPBtn::MCPBtn(uint8_t pin, uint8_t invert) : ButtonGeneral(pin, invert) {
  stateQueue = xQueueCreate( 1, sizeof(ButtonDebounceState) );
}


void MCPBtn::setState(ButtonDebounceState val_pair) {
  xQueueGenericSend(stateQueue, &val_pair, 0, queueOVERWRITE);
}


bool MCPBtn::stateChanged(ButtonDebounceState val_pair) {
  uint16_t val1 = (val_pair.val1 >> _pin) & 1;
  uint16_t val2 = (val_pair.val2 >> _pin) & 1;
  return (val1 == val2) && (val1 != _state);
}


bool MCPBtn::getState(ButtonDebounceState* state) {
  return xQueuePeek(stateQueue, state, 0) == pdTRUE;
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
  uint32_t ms = millis();
  _lastTime = _time;
  _time = ms;

  ButtonDebounceState val_pair;
  if (!getState(&val_pair)) {
    return _state;
  }

  // Debounce: check if the state hasn't been changed in 10 ms
  // and also check that the new state != old state of this button
  if (stateChanged(val_pair)) {
    _changed = 1;
    _lastChange = ms;
    updateStateFromPair(val_pair);
    if (isPressed()) { _pressTime = _time; }
  } else {
    _changed = 0;
  }

  return _state;
}
