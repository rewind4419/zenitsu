#include "maths.h"

struct GamepadButton {
  bool down;
  bool held;
  bool up;
};

struct GamepadInput {
  v2 joystick_left;
  v2 joystick_right;
  float trigger_left;
  float trigger_right;

  int dpad;

  GamepadButton x;
  GamepadButton y;
  GamepadButton a;
  GamepadButton b;
  
  GamepadButton bumper_left;
  GamepadButton bumper_right;

  GamepadButton big_button;

  GamepadButton share_button;
  GamepadButton option_button;
};

struct Input
{
  GamepadInput driver;
  GamepadInput mate;
};



void updateGamepad(Input* input);