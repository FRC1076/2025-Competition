package frc.robot.subsystems.superstructure.state;

public enum GrabberState {
    IDLE(0,0);
    public final double leftVolts;
    public final double rightVolts;
    private GrabberState(double leftVolts, double rightVolts) {
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }
}
