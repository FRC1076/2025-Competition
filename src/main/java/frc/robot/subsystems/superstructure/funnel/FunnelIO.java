package frc.robot.subsystems.superstructure.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    public static class FunnelIOInputs {
        public double appliedVolts = 0;
        public double appliedCurrent = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(FunnelIOInputs inputs);
}
