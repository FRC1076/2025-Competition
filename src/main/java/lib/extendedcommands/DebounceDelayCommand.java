package lib.extendedcommands;

import static frc.robot.Constants.DriveConstants.DriverControlConstants.singleClutchRotationFactor;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

//Delays until a booleanSupplier continuously returns true for a given time period
public class DebounceDelayCommand extends Command {
    private final BooleanSupplier signal;
    private final Debouncer debouncer;
    private boolean debouncedSignal = false;

    public DebounceDelayCommand(BooleanSupplier signal,double debounceTimeSeconds,Debouncer.DebounceType debounceType){
        this.signal = signal;
        this.debouncer = new Debouncer(debounceTimeSeconds,debounceType);
    }

    public DebounceDelayCommand(BooleanSupplier signal,double debouncedTimeSeconds){
        this(signal,debouncedTimeSeconds,Debouncer.DebounceType.kRising);
    }

    @Override
    public void execute(){
        debouncedSignal = debouncer.calculate(signal.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        debouncedSignal = false;
    }

    @Override
    public boolean isFinished(){
        return debouncedSignal;
    }
}
