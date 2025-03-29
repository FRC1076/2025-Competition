// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

/** 
 * A simple boilerplate class to enable full superstructure simulation
 */
public class GrabberIOSim implements GrabberIO {

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    public GrabberIOSim() {}

    @Override
    public void runVolts(double volts) {
        leftAppliedVolts = volts;
        rightAppliedVolts = volts;
    }

    @Override
    public void runVoltsDifferential(double leftMotorVoltage, double rightMotorVoltage) {
        leftAppliedVolts = leftMotorVoltage;
        rightAppliedVolts = rightMotorVoltage;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = leftAppliedVolts;
        inputs.rightMotorAppliedVoltage = rightAppliedVolts;
        inputs.motorPositionRadians += leftAppliedVolts;
        inputs.hasCoral = true;
    }
    
}
