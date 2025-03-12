package lib.extendedcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DaemonCommand extends Command {
    
    Command command;
    /**
     * Constructs a new DaemonCommand. NOTE: This class follows the same semantics as Command Compositions, and the command passed to this class cannot be schjeduled or composed independently
     * @param command the command to run as a Daemon
     * @param endCondition the condition when the DaemonCommand should end
     */
    public DaemonCommand(Command command, BooleanSupplier endCondition) {
        this.command = command.until(endCondition);
    }

    @Override
    public void initialize() {
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }
    
}
