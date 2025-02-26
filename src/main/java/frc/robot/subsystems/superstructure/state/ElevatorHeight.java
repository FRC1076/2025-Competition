package frc.robot.subsystems.superstructure.state;

public enum ElevatorHeight { //TODO: Update these wrt the actual conversion factor
    
    ALGAE_INTAKE_GROUND(0.184277),
    ALGAE_INTAKE_LOW(1.13789),
    ALGAE_INTAKE_HIGH(1.7440645),
    CORAL_INTAKE(0.08128),
    TRAVEL(0.08128),
    L1(0.08128),
    L2(0.910),
    L3(1.348 + 2 * 0.00889),
    L4(2.109649 + 3 * 0.00635),
    NET(2.109649 + 3 * 0.00635),
    PROCESSOR(0.184277 + 0.15);

    public final double meters;

    private ElevatorHeight(double meters){
        this.meters = meters;
    }
}
