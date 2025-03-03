package frc.robot.subsystems.superstructure.state;

public enum FunnelState {
    DEFAULT(-2),
    INTAKE(2);
    
    public final double voltage;
    private FunnelState(double voltage) {
        this.voltage = voltage;
    }
}
