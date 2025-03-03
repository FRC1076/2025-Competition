package frc.robot.subsystems.superstructure.state;

import static frc.robot.Constants.GrabberConstants.kCoralIntakeVoltage;

public enum GrabberState {
    IDLE(0,0),
    CORAL_INTAKE(kCoralIntakeVoltage,kCoralIntakeVoltage),
    CORAL_OUTTAKE(12,12),
    L1_CORAL_OUTTAKE(12,12); //TODO: UPDATE
    public final double leftVolts;
    public final double rightVolts;
    private GrabberState(double leftVolts, double rightVolts) {
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }
}
