package frc.robot.subsystems.superstructure.funnel;

public class FunnelIOSim implements FunnelIO {
    private double voltage;

    @Override
    public void setVoltage(double volts) {
        voltage = volts;
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.appliedVolts = voltage;
    }
}
