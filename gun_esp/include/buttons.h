#include "config.h"
#include <Button2.h>

namespace buttons {

enum Buttons {
  OK = 0,
  LEFT,
  RIGHT,
  TRIGGER,
};

// initialize buttons
void init();

// run internal button functions
// @note runs `.loop()` functions of all buttons
void spin();

Button2 get_button(Buttons butt);

// run internal functions of single button
// @note runs `.loop()` function of the specified button
void spin_only(Buttons button);
} // namespace buttons