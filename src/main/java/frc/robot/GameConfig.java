package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class GameConfig {

    public static final Alliance alliance = Alliance.Red;
    public static final AutonSides autonSide = AutonSides.Left;
    public static final String defaultAuton = "Grabber J4_K4_L4 - E4_D4_C4";
        
    // States describing whether the auton is on the left or right side of the alliance
    public enum AutonSides {
        Left(false),
        Right(true);

        public final boolean isRightSide;

        private AutonSides (boolean isRightSide) {
            this.isRightSide = isRightSide;
        }
    }
}
