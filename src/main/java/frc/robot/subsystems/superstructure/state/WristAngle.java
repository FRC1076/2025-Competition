package frc.robot.subsystems.superstructure.state;

import edu.wpi.first.math.util.Units;

public enum WristAngle {
    ALGAE_TRAVEL(65),
    CORAL_TRAVEL(90),
    L2L3_SCORE(-35);
    
    public final double degrees;
    public final double radians;
    private WristAngle(double degrees) {
        this.degrees = degrees;
        this.radians = Units.degreesToRadians(degrees);
    }
}
