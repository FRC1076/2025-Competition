// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDState;

public class LEDIOSim implements LEDBase {
    @Override
    public void setState(LEDState state) {
        // do nothing because there are no simulated LEDs
    }
}
