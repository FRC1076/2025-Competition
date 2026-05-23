// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class IndexSubsystem extends SubsystemBase {
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    /** Set voltage of the index motors */
    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    public void stop() {
        this.io.runVolts(0);
    }

    /** Returns a command that sets the voltage of the index motors
     * @param volts voltage to set the index motors to
     */
    public Command applyVolts(double volts) {
        return runOnce(() -> runVolts(volts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Index", inputs);
    }
}
