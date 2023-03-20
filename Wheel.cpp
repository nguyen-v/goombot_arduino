#include "Wheel.h"

Wheel::~Wheel() {
  set_speed(0);  
}

void Wheel::set_speed(float speed) {
    controller.set_rpm_shaft(60. * speed / (M_PI * diameter));
}

float get_diameter() const {
  return diameter;
}
