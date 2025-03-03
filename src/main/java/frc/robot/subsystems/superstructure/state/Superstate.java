package frc.robot.subsystems.superstructure.state;

public enum SuperState {
    

    OVERRIDE(null,null,null,null,null), // Operator override

    PRE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.CORAL),
    SCORE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.L1_CORAL_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    PRE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.CORAL),
    SCORE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    PRE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.CORAL),
    SCORE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    PRE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.CORAL),
    SCORE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    PRE_GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.EMPTY),
    GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,PossessionState.ALGAE),
    POST_GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE),

    PRE_LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.EMPTY),
    LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,PossessionState.ALGAE),
    POST_LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE),

    PRE_HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.EMPTY),
    HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,PossessionState.ALGAE),
    POST_HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE),

    PRE_NET(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE),
    NET_SCORE(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    PRE_PROCESSOR(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE),
    PROCESSOR_SCORE(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,PossessionState.EMPTY),

    CORAL_INTAKE(ElevatorHeight.CORAL_INTAKE,WristAngle.CORAL_INTAKE,GrabberState.CORAL_INTAKE,FunnelState.INTAKE,PossessionState.EMPTY),

    CORAL_TRAVEL(ElevatorHeight.CORAL_TRAVEL,WristAngle.CORAL_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.CORAL),
    EMPTY_TRAVEL(ElevatorHeight.EMPTY_TRAVEL,WristAngle.EMPTY_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT,PossessionState.EMPTY),
    ALGAE_TRAVEL(ElevatorHeight.ALGAE_TRAVEL,WristAngle.ALGAE_TRAVEL,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,PossessionState.ALGAE);

    public static enum PossessionState {
        EMPTY,
        CORAL,
        ALGAE
    }

    public final ElevatorHeight height;
    public final WristAngle angle;
    public final GrabberState grabberState;
    public final FunnelState funnelState;
    public final PossessionState expectedPossession;

    private SuperState(ElevatorHeight height, WristAngle angle, GrabberState grabberState, FunnelState funnelState, PossessionState expectedPossession) {
        this.height = height;
        this.angle = angle;
        this.grabberState = grabberState;
        this.funnelState = funnelState;
        this.expectedPossession = expectedPossession;
    }
}
