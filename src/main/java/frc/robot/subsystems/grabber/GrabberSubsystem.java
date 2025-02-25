// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class GrabberSubsystem extends SubsystemBase{
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
    }

    /** Sets both motors to the same voltage */
    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    /** Sets the left and right motors to different voltages */
    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        this.io.runVoltsDifferential(leftMotorVolts, rightMotorVolts);
    }

    public void stop() {
        runVolts(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    /* ######################################################################## */
    /* # Public Command Factories                                             # */
    /* ######################################################################## */

    /** Returns a command that sets the left and right motors to the same voltage 
     * @param volts The voltage to set the motors to
    */
    public Command applyVoltage(double volts) {
        return runOnce(() -> runVolts(volts));
    }
    
    /** Returns a command that sets the left and right motors to different voltages 
     * @param leftMotorVolts The voltage to set the left motor to
     * @param rightMotorVolts The voltage to set the right motor to
    */
    public Command applyDifferentialVolts(double leftMotorVolts, double rightMotorVolts) {
        return runOnce(() -> runVoltsDifferential(leftMotorVolts, rightMotorVolts));
    }

    public double getAppliedCurrent() {
        return inputs.leftMotorCurrent;
    }

    /**
     * NOTE: RADIANS ARE RELATIVE, CONTROL IS BASED OFF THE LEFT MOTOR'S ENCODER
     * @param volts
     * @param radians
     * @return
     *  a command that applies a certain number of rotations to the grabber via a simple Bang-Bang controller.
     */
    public Command applyRotationsBangBang(double volts, double radians) {
        double setpoint = inputs.motorPositionRadians + radians;
        boolean positiveDirection = (setpoint > inputs.motorPositionRadians);
        return new FunctionalCommand(
            () -> runVolts(volts),
            () -> {},
            (interrupted) -> stop(),
            positiveDirection 
                ? () -> inputs.motorPositionRadians >= setpoint
                : () -> inputs.motorPositionRadians <= setpoint,
            this
        );
        
    }
    
}