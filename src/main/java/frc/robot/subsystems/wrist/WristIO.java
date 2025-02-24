// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    public static record WristControlConstants (
        Double kP,
        Double kI,
        Double kD,
        Constraints kProfileConstraints,
        Double kS,
        Double kG,
        Double kV,
        Double kA
    ) {}
    
    @AutoLog
    public static class WristIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract WristControlConstants getControlConstants();

    public abstract void updateInputs(WristIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default void simulationPeriodic() {}

}
