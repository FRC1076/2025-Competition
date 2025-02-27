package frc.robot.subsystems.superstructure.state;

public enum SuperState {
    PRE_L3(ElevatorHeight.L3,WristAngle.L2L3_SCORE,GrabberState.IDLE,FunnelState.DEFAULT);
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
