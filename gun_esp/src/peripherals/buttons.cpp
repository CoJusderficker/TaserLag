#include "buttons.h"

Button2 button[4];

void buttons::init() {
  int buttpins[4] = {PIN_BUTTON_OK, PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT,
                     PIN_BUTTON_TRIGGER};

  int i = 0;
  for (Button2 butt : button) {
    butt.begin(buttpins[i]);
    i++;
  }
}

Button2 buttons::get_button(buttons::Buttons butt) { return button[butt]; }

void buttons::spin() {
  for (Button2 butt : button) {
    butt.loop();
  }
}

void buttons::spin_only(Buttons butt) { button[butt].loop(); }