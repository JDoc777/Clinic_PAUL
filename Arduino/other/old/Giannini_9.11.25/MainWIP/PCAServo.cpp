#include "PCAServo.h"

namespace PCAServo {

  void begin(int PWMHz){
    pwm.begin();
    pwm.setPWMFreq(PWMHz);
    defaultAngles();

  }

  
}

