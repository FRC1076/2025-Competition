package frc.robot.subsystems.superstructure.state;

public enum SuperState {

    OVERRIDE(null,null,null,null), // Operator override

    PRE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.IDLE,FunnelState.DEFAULT),
    SCORE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.L1_CORAL_OUTTAKE,FunnelState.DEFAULT),

    PRE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT),
    SCORE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT),

    PRE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT),
    SCORE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT),

    PRE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.IDLE,FunnelState.DEFAULT),
    SCORE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT),

    CORAL_INTAKE(ElevatorHeight.CORAL_INTAKE,WristAngle.CORAL_INTAKE,GrabberState.CORAL_INTAKE,FunnelState.INTAKE),

    CORAL_TRAVEL(ElevatorHeight.TRAVEL,WristAngle.CORAL_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT),
    ALGAE_TRAVEL(ElevatorHeight.TRAVEL,WristAngle.ALGAE_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT);

    public final ElevatorHeight height;
    public final WristAngle angle;
    public final GrabberState grabberState;
    public final FunnelState funnelState;

    private SuperState(ElevatorHeight height, WristAngle angle, GrabberState grabberState, FunnelState funnelState) {
        this.height = height;
        this.angle = angle;
        this.grabberState = grabberState;
        this.funnelState = funnelState;
    }
}
