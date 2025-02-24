// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.index;

/** 
 * A simple boilerplate class to enable full superstructure simulation
 */
public class IndexIOSim implements IndexIO {
    //TODO: Add Current and full motor simulation

    private double leadAppliedVolts = 0.0;
    private double followAppliedVolts = 0.0;

    public IndexIOSim() {}

    @Override
    public void runVolts(double volts) {
        leadAppliedVolts = volts;
        followAppliedVolts = volts;
    }


    @Override
    public void updateInputs(IndexIOInputs inputs) {
        inputs.leadMotorAppliedVoltage = leadAppliedVolts;
        inputs.followMotorAppliedVoltage = followAppliedVolts;
    }
    
}
