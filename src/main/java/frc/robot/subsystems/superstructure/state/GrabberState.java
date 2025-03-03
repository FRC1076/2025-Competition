package frc.robot.subsystems.superstructure.state;

import static frc.robot.Constants.GrabberConstants.kCoralEffectorVoltage;

import static frc.robot.Constants.GrabberConstants.kAlgaeEffectorVoltage;

import static frc.robot.Constants.GrabberConstants.kAlgaeHoldingVoltage;

public enum GrabberState {
    IDLE(0,0),
    CORAL_INTAKE(kCoralEffectorVoltage,kCoralEffectorVoltage),
    CORAL_OUTTAKE(kCoralEffectorVoltage,kCoralEffectorVoltage),
    ALGAE_INTAKE(-kAlgaeEffectorVoltage,-kAlgaeEffectorVoltage),
    ALGAE_OUTTAKE(kAlgaeEffectorVoltage,kAlgaeEffectorVoltage),
    ALGAE_HOLD(kAlgaeHoldingVoltage,kAlgaeHoldingVoltage),
    REVERSE_CORAL_INTAKE(-kCoralEffectorVoltage,-kCoralEffectorVoltage),
    L1_CORAL_OUTTAKE(12,12); //TODO: UPDATE

    public final double leftVolts;
    public final double rightVolts;
    private GrabberState(double leftVolts, double rightVolts) {
        this.leftVolts = leftVolts;
        this.rightVolts = rightVolts;
    }
}
