// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DaemonCommand extends Command {
    
    Command command;
    /**
     * Constructs a new DaemonCommand. NOTE: This class follows the same semantics as Command Compositions, and the command passed to this class cannot be scheduled or composed independently
     * @param command the command to run as a Daemon
     * @param endCondition the condition when the DaemonCommand should end
     */
    public DaemonCommand(Command command, BooleanSupplier endCondition) {
        this.command = command.until(endCondition);
    }

    public DaemonCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
