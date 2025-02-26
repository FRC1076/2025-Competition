package frc.robot.subsystems.superstructure.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel {
    private final FunnelIO io;
    private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

    public Funnel(FunnelIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Funnel/", inputs);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
}
