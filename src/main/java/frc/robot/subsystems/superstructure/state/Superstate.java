package frc.robot.subsystems.superstructure.state;

public enum SuperState {

    OVERRIDE(null,null,null,null,null), // Operator override

    PRE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    SCORE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.L1_CORAL_OUTTAKE,FunnelState.DEFAULT,false),

    PRE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    SCORE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,false),

    PRE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    SCORE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,false),

    PRE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    SCORE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,false),

    PRE_GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,true),

    PRE_LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,true),

    PRE_HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,false),
    HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,true),

    PRE_NET(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,true),
    NET_SCORE(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,false),

    PRE_PROCESSOR(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,true),
    PROCESSOR_SCORE(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,false),

    CORAL_INTAKE(ElevatorHeight.CORAL_INTAKE,WristAngle.CORAL_INTAKE,GrabberState.CORAL_INTAKE,FunnelState.INTAKE,false),

    CORAL_TRAVEL(ElevatorHeight.TRAVEL,WristAngle.CORAL_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT,false),
    ALGAE_TRAVEL(ElevatorHeight.TRAVEL,WristAngle.ALGAE_TRAVEL,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,true);

    public final ElevatorHeight height;
    public final WristAngle angle;
    public final GrabberState grabberState;
    public final FunnelState funnelState;
    public final boolean possessAlgae;

    private SuperState(ElevatorHeight height, WristAngle angle, GrabberState grabberState, FunnelState funnelState, Boolean possessAlgae) {
        this.height = height;
        this.angle = angle;
        this.grabberState = grabberState;
        this.funnelState = funnelState;
        this.possessAlgae = possessAlgae;
    }
}
