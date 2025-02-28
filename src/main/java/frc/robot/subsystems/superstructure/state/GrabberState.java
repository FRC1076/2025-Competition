package frc.robot.subsystems.superstructure.state;

public enum GrabberState {
    IDLE(0,0),
    CORAL_INTAKE(12,12),
    CORAL_OUTTAKE(12,12),
    L1_CORAL_OUTTAKE(12,12); //TODO: UPDATE
    public final double leftVolts;
    public final double rightVolts;
    private GrabberState(double leftVolts, double rightVolts) {
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }
}
