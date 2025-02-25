// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
    @AutoLog
    public static class IndexIOInputs {
        public double leadMotorAppliedVoltage = 0;
        public double leadMotorCurrent = 0;
    }

    public default void updateInputs(IndexIOInputs inputs) {}

    public default void runVolts(double volts) {}
}
