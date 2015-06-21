#ifndef JOYPAD_BUTTONS_AXES_H_
#define JOYPAD_BUTTONS_AXES_H_

//The joypad should be tun in X-mode (rear switch), mode-button OFF

namespace joypad_node
{
  namespace axes
  {
    const uint32_t X = 4; // 1
    const uint32_t Y = 3; // 0
    const uint32_t Z = 1; // 4
    const uint32_t YAW = 0; // 3
    const uint32_t SPEED_ASSISTED = 5;
    const uint32_t YAW_ASSISTED = 2;
  }
  namespace buttons
  {
    const uint32_t GREEN=0;
    const uint32_t RED=1;
    const uint32_t BLUE=2;
    const uint32_t YELLOW=3;
    const uint32_t LB=4;
    const uint32_t RB=5;
  }
} // namespace joypad_node



#endif /* JOYPAD_BUTTONS_AXES_H_ */
