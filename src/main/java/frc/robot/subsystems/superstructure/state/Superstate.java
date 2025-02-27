package frc.robot.subsystems.superstructure.state;

public enum SuperState {
    PRE_L3()
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
