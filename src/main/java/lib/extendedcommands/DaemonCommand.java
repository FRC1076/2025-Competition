// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* A wrapper that executes a command in the background without blocking execution of a CommandSequence 
NOTE: If possible, use parallel and sequential command groups instead of this class. */
public class DaemonCommand extends Command {
    Supplier<Command> commandSupplier;
    BooleanSupplier endCondition;
    /**
     * Constructs a new DaemonCommand
     * @param commandSupplier the command to run as a Daemon
     * @param endCondition the condition when the DaemonCommand should end
     */
    public DaemonCommand(Supplier<Command> commandSupplier, BooleanSupplier endCondition) {
        this.commandSupplier = commandSupplier;
        this.endCondition = endCondition;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(commandSupplier.get().until(endCondition));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

