package frc.robot.subsystems.superstructure.state;

import edu.wpi.first.math.util.Units;

public enum WristAngle {
    ALGAE_TRAVEL(65),
    ALGAE_INTAKE(-35),
    ALGAE_GROUND_INTAKE(-20),
    PROCESSOR(0),
    NET(65),
    CORAL_TRAVEL(-90),
    CORAL_INTAKE(-23.5),
    EMPTY_TRAVEL(90),
    L1_SCORE(0), // TODO: UPDATE
    L2L3_SCORE(-35),
    L4_SCORE(-40.4130051);
    
    public final double degrees;
    public final double radians;
    private WristAngle(double degrees) {
        this.degrees = degrees;
        this.radians = Units.degreesToRadians(degrees);
    }
}
