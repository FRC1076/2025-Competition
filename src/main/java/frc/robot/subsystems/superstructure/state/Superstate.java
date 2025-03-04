package frc.robot.subsystems.superstructure.state;

import static frc.robot.subsystems.superstructure.state.Possession.*;

public enum SuperState {
    
    OVERRIDE(null,null,null,null,kNoChange), 

    ALIGN_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    SCORE_L1(ElevatorHeight.L1,WristAngle.L1_SCORE,GrabberState.L1_CORAL_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    ALIGN_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    SCORE_L2(ElevatorHeight.L2,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    ALIGN_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    SCORE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    ALIGN_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    SCORE_L4(ElevatorHeight.L4,WristAngle.L4_SCORE,GrabberState.CORAL_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    PRE_GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,kAlgaeIntake),
    POST_GROUND_INTAKE(ElevatorHeight.ALGAE_INTAKE_GROUND,WristAngle.ALGAE_GROUND_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,kAlgaeIntake),

    PRE_LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    LOW_INTAKE(ElevatorHeight.ALGAE_INTAKE_LOW,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,kAlgaeIntake),

    PRE_HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    HIGH_INTAKE(ElevatorHeight.ALGAE_INTAKE_HIGH,WristAngle.ALGAE_INTAKE,GrabberState.ALGAE_INTAKE,FunnelState.DEFAULT,kAlgaeIntake),
    
    PRE_NET(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,kNoChange),
    NET_SCORE(ElevatorHeight.NET,WristAngle.NET,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    PRE_PROCESSOR(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,kNoChange),
    PROCESSOR_SCORE(ElevatorHeight.PROCESSOR,WristAngle.PROCESSOR,GrabberState.ALGAE_OUTTAKE,FunnelState.DEFAULT,kOuttake),

    CORAL_INTAKE(ElevatorHeight.CORAL_INTAKE,WristAngle.CORAL_INTAKE,GrabberState.CORAL_INTAKE,FunnelState.INTAKE,kCoralIntake),

    CORAL_TRAVEL(ElevatorHeight.CORAL_TRAVEL,WristAngle.CORAL_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    EMPTY_TRAVEL(ElevatorHeight.EMPTY_TRAVEL,WristAngle.EMPTY_TRAVEL,GrabberState.IDLE,FunnelState.DEFAULT,kNoChange),
    ALGAE_TRAVEL(ElevatorHeight.ALGAE_TRAVEL,WristAngle.ALGAE_TRAVEL,GrabberState.ALGAE_HOLD,FunnelState.DEFAULT,kNoChange);

    public final ElevatorHeight height;
    public final WristAngle angle;
    public final GrabberState grabberState;
    public final FunnelState funnelState;
    public final PossessionMap possessionMap;

    private SuperState(ElevatorHeight height, WristAngle angle, GrabberState grabberState, FunnelState funnelState, PossessionMap possessionMap) {
        this.height = height;
        this.angle = angle;
        this.grabberState = grabberState;
        this.funnelState = funnelState;
        this.possessionMap = possessionMap;
    }
}
