// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* A wrapper that executes a command in the background without blocking execution of a CommandSequence 
NOTE: If possible, use parallel and sequential command groups instead of this class. */
public class DaemonCommand extends Command {
    Command command;
    BooleanSupplier endCondition;
    /**
     * Constructs a new DaemonCommand
     * @param command the command to run as a Daemon
     * @param endCondition the condition when the DaemonCommand should end
     */
    public DaemonCommand(Command command, BooleanSupplier endCondition) {
        this.command = command;
        this.endCondition = endCondition;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(command.until(endCondition));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

