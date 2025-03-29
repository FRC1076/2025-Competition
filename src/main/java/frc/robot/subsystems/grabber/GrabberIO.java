// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

/**
 * A common interface that allows the subsystem code to interact with a grabber,
 * while abstracting away implementation, to allow for polymorphism (sim or hardware implementations)
 */
public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorCurrent = 0;

        public double rightMotorAppliedVoltage = 0;
        public double rightMotorCurrent = 0;

        public double motorPositionRadians = 0;

        public boolean hasCoral;
    }

    /** Update values for logging */
    public abstract void updateInputs(GrabberIOInputs inputs);

    /** Sets both motors to the same voltage
     * @param volts voltage to run the motors at
     */
    public abstract void runVolts(double volts);

    /** Sets the left and right motors to different voltages
     * @param leftMotorVolts voltage to run the left motor at
     * @param rightMotorVolts voltage to run the right motor at
     */
    public abstract void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts);

    /** Default implementation for io layers that don't have actual current (like simulation) */
    public default double getOutputCurrent() {
        return -1;
    }
}