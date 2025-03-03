// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.grabber;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.grabber.GrabberIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Grabber {
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public Grabber(GrabberIO io) {
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

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    public double getRotations() {
        return inputs.motorPositionRadians;
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
    /* 
    public Command applyRadiansBangBang(double volts, double radians) {
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
    */
    
}