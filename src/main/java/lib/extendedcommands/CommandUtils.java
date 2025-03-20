// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.extendedcommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.apache.commons.lang3.NotImplementedException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandUtils {
    private CommandUtils() {
        throw new NotImplementedException("This is a utility class!");
    }

    /** Tells the robot to run a command in the background with an arbitrary end condition */
    public static Command makeDaemon(Command command, BooleanSupplier endCondition) {
        return new DaemonCommand(command, endCondition);
    }

    /** Tells the robot to run a command in the background */
    public static Command makeDaemon(Command command) {
        return new DaemonCommand(command);
    }

    public static Command runAsDaemon(Runnable action, BooleanSupplier endCondition, Subsystem... requirements) {
        return new DaemonCommand(Commands.run(action,requirements),endCondition);
    }

    public static Command runAsDaemon(Runnable action, Subsystem... requirements) {
        return new DaemonCommand(Commands.run(action,requirements));
    }

    /** tells the robot to periodically run a runnable that is not associated with any particular subsystem */
    public static void makePeriodic(Runnable action, boolean runWhenDisabled) {
        CommandScheduler.getInstance().schedule(Commands.run(action).ignoringDisable(runWhenDisabled));
    }

    /** tells the robot to periodically run a runnable that is not associated with any particular subsystem */
    public static void makePeriodic(Runnable action) {
        CommandScheduler.getInstance().schedule(Commands.run(action).ignoringDisable(false));
    }

    public static Command waitUntilDebounced(BooleanSupplier signal,double debounceTimeSeconds) {
        return new DebounceDelayCommand(signal, debounceTimeSeconds);
    }
}
