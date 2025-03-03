// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.grabber;

/** 
 * A simple boilerplate class to enable full superstructure simulation
 */
public class GrabberIOSim implements GrabberIO {
    //TODO: Add Current and full motor simulation

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
    }
    
    @Override
    public boolean algaePossession() {
        // TODO Auto-generated method stub, IMPLEMENT WITH DEBOUNCER AND LINEAR FILTER
        return false;
    }
}
