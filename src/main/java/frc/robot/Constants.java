// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.apache.commons.lang3.NotImplementedException;
import java.util.Set;

import static frc.robot.utils.Localization.flipPose;
import static frc.robot.utils.Localization.mirrorPose;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean+
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
     
     public static class VisionConstants {
        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Set<Integer> filteredTargetIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static class Photonvision {

            public static final String driverCamName = "DRIVER_CAM"; //PV name of the driver camera

            public static final Vector<N3> kDefaultSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
            public static final Vector<N3> kDefaultMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

            /** Contains configs for all photonvision localization cameras */
            public static enum PhotonConfig {
                
                FRONT_LEFT_CAM(
                    "FRONT_LEFT_CAM", 
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    13.75, /*12.389,*/ 11.683, 11.25,//10.875,//11.513639, // 15 - 7.163, 15 - 2.892, 19.162,
                    0, 0, -20 //11.385, 17.961, -40 
                ),
                FRONT_RIGHT_CAM(
                    "FRONT_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    13.75/*12.389*/, -11.683, 11.25,//10.875, // 15 - 7.163, -(15 - 2.892), 19.162, 
                    0, 0, 20 // -11.385, 17.961, 40
                ),
                BACK_RIGHT_CAM(
                    "BACK_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    -12.785, -12.565, 6.25+4.411,
                    0, 0, -140
                );

                public final String name;
                public final Transform3d offset;
                public final Matrix<N3, N1> defaultSingleTagStdDevs;
                public final Matrix<N3, N1> defaultMultiTagStdDevs;
                public final PoseStrategy multiTagPoseStrategy;
                public final PoseStrategy singleTagPoseStrategy;
                private PhotonConfig(
                    String name, 
                    Matrix<N3, N1> defaultSingleTagStdDevs,
                    Matrix<N3, N1> defaultMultiTagStdDevs,
                    PoseStrategy multiTagPoseStrategy,
                    PoseStrategy singleTagPoseStrategy,
                    double xInch, double yInch, double zInch, 
                    double rollDeg, double pitchDeg, double yawDeg
                ) {
                    this.name = name;
                    this.offset = new Transform3d(
                        Units.inchesToMeters(xInch),
                        Units.inchesToMeters(yInch),
                        Units.inchesToMeters(zInch),
                        new Rotation3d(
                            Units.degreesToRadians(rollDeg),
                            Units.degreesToRadians(pitchDeg),
                            Units.degreesToRadians(yawDeg)
                        )
                    );
                    this.multiTagPoseStrategy = multiTagPoseStrategy;
                    this.singleTagPoseStrategy = singleTagPoseStrategy;
                    this.defaultMultiTagStdDevs = defaultMultiTagStdDevs;
                    this.defaultSingleTagStdDevs = defaultSingleTagStdDevs;
                }
            }
        }
    }
  
    /** Contains starting position and team */
    public static class GameConstants {

        public static Alliance teamColor = Alliance.Blue;
        public static AutonSides autonSide = AutonSides.Left;
        public static boolean rearRightCameraEnabledAuton = false; // Only set to true if running algae auton

        // Autonomous command is selected in getAutonomousCommand() in RobotContainer
        
        public enum TeamColors {
            kTeamColorBlue("BLUE"),
            kTeamColorRed("RED");

            public final String color;

            private TeamColors(String color) {
                this.color = color;
            }
        }
        
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

    public static class OIConstants {
        public static final boolean kUseDroperatorController = true; // Overrides alternate driver/operator controller
        public static final boolean kUseOperiverColorController = true; // Use the color controller (requires droperator to be enabled)

        public static final boolean kUseAlternateDriverController = true;
        public static final boolean kUseAlternateOperatorController = false; // Switches controls for ground algae intake and processor

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kDroperatorControllerPort = 2;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class SystemConstants {
        /*
         * Correct constants for competition:
         * currentMode = 0
         * operatorSysId = false
         * driverSysId = false
         * logOdometry = false
         * logCTRE = false
         */
        public static final int currentMode = 0; // 0 is real, 1 is sim
        public static final boolean operatorSysID = false;
        public static final boolean driverSysID = false;
        public static final boolean logOdometry = false;
        public static final boolean logCTRE = false; // Whether CTRE hoot logging should be enabled
        // public static final boolean raiseThreadPriority = true; // Whether the main thread should have its priority raised
    }
    
    public static class DriveConstants {

        public static class DriverControlConstants {
            public static final double singleClutchTranslationFactor = 0.5;
            public static final double singleClutchRotationFactor = 0.5;
            public static final double doubleClutchTranslationFactor = 0.3;
            public static final double doubleClutchRotationFactor = 0.35;
            public static final double FPVClutchTranslationFactor = 0.1;
            public static final double FPVClutchRotationFactor = 0.1;
            public static final double ElevatorClutchTransFactor = 0.5; // Clutch activated when the elevator is above L3
            public static final double ElevatorClutchRotFactor = 0.5;
            public static final double maxTranslationSpeedMPS = 3.0; // 5.0 is default
            public static final double maxRotationSpeedRadPerSec = 3.0; // 5.0 is default

            // Calculations from: https://docs.google.com/spreadsheets/d/1ht2fXTaIHJL2nEzKbsCiHQ5DaE7uzN4XHMsRuwxwwmQ/edit?usp=sharing
            // Input: elevator height from bottom of the elevator in meters
            // Output: max translation acceleration in m/s/s

            public static final InterpolatingDoubleTreeMap elevatorAccelerationTable = new InterpolatingDoubleTreeMap(); // A table that maps elevator heights to slew rate limits
            static {
                elevatorAccelerationTable.put(0.0,100000.0);
                elevatorAccelerationTable.put(1.0,100000.0); // Deadzone with no acceleration limiting between 0.0 and 1.348 (THE END OF THIS DEADZONE *MUST* BE SLIGHTLY LOWER THAN THE POINT WHERE WE ACTUALLY WANT ELEVATOR ACCELERATION LIMITING TO BEGIN)
                // elevatorAccelerationTable.put(0.0, 12.66793578);
                // elevatorAccelerationTable.put(0.253, 100000.0);
                // elevatorAccelerationTable.put(0.254, 10.15773958 / 5);
                // elevatorAccelerationTable.put(0.508, 8.477828029 / 5);
                // elevatorAccelerationTable.put(0.762, 7.274717623 / 5);
                elevatorAccelerationTable.put(1.016, 6.370643237 / 5);
                elevatorAccelerationTable.put(1.27, 5.666439564 / 6);
                elevatorAccelerationTable.put(1.524, 5.102204373 / 7);
                elevatorAccelerationTable.put(1.778, 4.640342002 / 8);
                elevatorAccelerationTable.put(1.8288, 4.557930098 / 8);
            }
        }

        public static class DirectDriveConstants {
            public static final Constraints translationConstraints = new Constraints(2, 2);
            public static final Constraints headingConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360));
        }

        public static class PathPlannerConstants {
            public static final PathConstraints pathConstraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
            public static final PathConstraints netConstraints = new PathConstraints(2, 2.6788, Units.degreesToRadians(360), Units.degreesToRadians(360));
            public static final Transform2d robotOffset = new Transform2d(0.508, 0, Rotation2d.kZero);
            public static final Transform2d robotLeftL1Offset = new Transform2d(0.508, Units.inchesToMeters(-12), Rotation2d.kZero);
            public static final Transform2d robotCenterL1Offset = new Transform2d(0.508, Units.inchesToMeters(0), Rotation2d.kZero);
            public static final Transform2d robotRightL1Offset = new Transform2d(0.508, Units.inchesToMeters(2), Rotation2d.kZero);
            public static final double pathGenerationToleranceMeters = 0.011; // Technically it's anything larger than 0.01, but I'm adding .001 just to be safe
            public static final double LEDpathToleranceMeters = 0.03;

            public static class Control {
                public static final PIDConstants transPID = new PIDConstants(5, 0, 0);
                public static final PIDConstants rotPID = new PIDConstants(5, 0, 0);
            }
        }

        public static final double kOdometryUpdateFrequency = 250.0;
    }

    public static class SuperstructureConstants {

        public static Rotation2d algaeTravelAngle = Rotation2d.fromDegrees(70);
        public static Rotation2d coralTravelAngle = Rotation2d.fromDegrees(90);

        // Grabber Possession State
        public enum GrabberPossession {
            EMPTY(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            CORAL(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            ALGAE(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            TRANSFERRING(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG
                    : ElevatorSimConstants.Control.kG);

            public final double wrist_kG;
            public final double elevator_kG;

            private GrabberPossession(double wrist_kG, double elevator_kG) {
                this.wrist_kG = wrist_kG;
                this.elevator_kG = elevator_kG;
            }
        }

        // Grabber State
        public enum GrabberState {

            IDLE(0, 0),
            
            ALGAE_INTAKE(-12, -12),
            ALGAE_HOLD(-2, -2),
            CORAL_INTAKE(12, 12),
            REVERSE_CORAL_INTAKE(-12, -12),
            GRABBER_CORAL_INTAKE(-12, -12),

            ALGAE_OUTTAKE(12, 12),
            CORAL_OUTTAKE(12, 12),
            L1_OUTTAKE(8, 5),
            DEFAULT_OUTTAKE(12, 12);

            public final double leftVoltage;
            public final double rightVoltage;

            private GrabberState(double leftVoltage, double rightVoltage) {
                this.leftVoltage = leftVoltage;
                this.rightVoltage = rightVoltage;
            }
        }

        // Index State
        public enum IndexState {
            TRANSFER(6),
            IDLE(0), // Never implemented, just an option
            BACKWARDS(-1);
            
            public final double volts;

            private IndexState(double volts) {
                this.volts = volts;
            }
        }

        // Represent the elevator height and wrist angle for different positions, the full position of the grabber
        // Should we have an eject state with an optional elevator height? just to immediately eject if a game piece is stuck
        public enum WristevatorState {
            
            TRAVEL(0.1349839121 + 0.00635, 90),//90),
            //POST_SCORE_TRAVEL(0.5, 90),
            ALGAE_TRAVEL(0.134983912 + 0.00635, 80),
            PRE_AUTOMATIC_NET(0.1349839121 + 0.00635, 45),

            CORAL_TRANSFER(0.1349839121 + 0.00635, -15.57789 + 2), // Same as CORAL_DIRECT_INTAKE
            GRABBER_CORAL_INTAKE(0.784, 36.956),
            GRABBER_CORAL_INTAKE_TRAVEL(0.784, 90),//90),
            HIGH_TRAVEL(0.3, -90), //with grabber down state

            L1(0.394, 23.9365), // Placeholder
            L1_STACK(0.3048 + 3 * 0.0254, 15),
            L2(0.910, -35), //0.71628, -35),
            L3(1.348 + 2 * 0.00889, -35), //1.11252, -35),
            L4(2.11455, -38),//-40.4130051, true), //1.8161, -45),

            GROUND_INTAKE(0, 0),
            LOLLIPOP_INTAKE(0.1349839121 + 0.00635, 26.7),
            LOW_INTAKE(0.98407,-27.02),
            HIGH_INTAKE(1.44998, -27.02),

            PROCESSOR(0.2536, 0),
            NET(2.109649 + 3 * 0.00635, 67.5); // 80

            public final double elevatorHeightMeters;
            public final Rotation2d wristAngle;
            
            private WristevatorState(double elevatorHeightMeters, double wristAngleDegrees) {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.wristAngle = Rotation2d.fromDegrees(wristAngleDegrees);
            }
        }
    }

    /** Contains data about the field */
    public static class FieldConstants {
        public static final double fieldWidthMeters = Units.inchesToMeters(316.63); // Distance from one processor side to the other. From AprilTag coordinates in https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf
        public static final double fieldLengthMeters = 17.54; // Distance in meters from one drive station to the other side

        private static final double branchOffset = Units.inchesToMeters(6.469);
        private static final Transform2d leftBranchTransform = new Transform2d(0.0, -branchOffset, Rotation2d.kZero);
        private static final Transform2d rightBranchTransform = new Transform2d(0.0, branchOffset, Rotation2d.kZero);

        // Trough width is 37.04 inches
        // Coral is 11.875 inches long
        private static final Transform2d leftL1Transform = new Transform2d(0.0, Units.inchesToMeters(-19), Rotation2d.kZero);
        private static final Transform2d middleL1Transform = new Transform2d(0.0, Units.inchesToMeters(-7), Rotation2d.kZero);
        private static final Transform2d rightL1Transform = new Transform2d(0.0, Units.inchesToMeters(5), Rotation2d.kZero);
        private static final Transform2d leftL1StackTransform = new Transform2d(0.0, Units.inchesToMeters(-13), Rotation2d.kZero);
        private static final Transform2d rightL1StackTransform = new Transform2d(0.0, Units.inchesToMeters(-1), Rotation2d.kZero); 

        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {
            // IMPORTANT: Fudge factors are always positive and should be in meters (use the Units.inchesToMeters() method)

            // Blue Reef
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null, true),
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null, false),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null, true),
            BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null, false),
            BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null, true),
            BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null, false),

            // Red Reef
            RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null, true),
            RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null, false),
            RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null, true),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null, false),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null, true),
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null, false);


            public final Double leftBranchFudgeTransform;
            public final Double rightBranchFudgeTransform;
            public final Pose2d leftBranch;
            public final Pose2d rightBranch;

            public final Pose2d leftL1;
            public final Pose2d middleL1;
            public final Pose2d rightL1;
            public final Pose2d leftStackL1;
            public final Pose2d rightStackL1;
            private int L1Index;

            public final Pose2d AprilTag;
            public final int aprilTagID;
            public final boolean algaeHigh;

            //AT stands for AprilTag
            private ReefFace(int aprilTagID, double aprilTagX, double aprilTagY, double aprilTagTheta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform, boolean algaeHigh) {
                this.aprilTagID = aprilTagID;
                this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
                this.leftBranchFudgeTransform = leftBranchFudgeTransform;
                this.rightBranchFudgeTransform = rightBranchFudgeTransform;

                this.leftL1 = AprilTag.transformBy(leftL1Transform);
                this.middleL1 = AprilTag.transformBy(middleL1Transform);
                this.rightL1 = AprilTag.transformBy(rightL1Transform);
                this.leftStackL1 = AprilTag.transformBy(leftL1StackTransform);
                this.rightStackL1 = AprilTag.transformBy(rightL1StackTransform);
                this.L1Index = 0;

                this.algaeHigh = algaeHigh;

                if (this.leftBranchFudgeTransform == null) {
                    this.leftBranch = AprilTag.transformBy(leftBranchTransform);
                } else {
                    this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
                }
                
                if (this.rightBranchFudgeTransform == null) {
                    this.rightBranch = AprilTag.transformBy(rightBranchTransform);
                } else {
                    this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
                }
            }

            public Pose2d getNextL1Position() {
                Pose2d L1Position;

                if (L1Index % 3 == 0) {
                    L1Position = rightL1;
                } else if (L1Index % 3 == 1) {
                    L1Position = middleL1;
                } else if (L1Index % 3 == 2) {
                    L1Position = leftL1;
                } else if (L1Index % 3 == 3) {
                    L1Position = rightStackL1;
                } else {
                    L1Position = leftStackL1;
                }

                return L1Position;
            }

            public WristevatorState getNextL1WristevatorState() {
                WristevatorState state;

                if (L1Index % 3 <= 2) {
                    state = WristevatorState.L1;
                } else {
                    state = WristevatorState.L1_STACK;
                }

                return state;
            }

            public void increaseL1Index() {
                L1Index++;
            }
        }

        // Generic rotation-agnostic points of interest
        public enum PointOfInterest {
            BLU_REEF(4.487, 4.010),
            RED_REEF(13.062, 4.010);

            public final Translation2d position;
            private PointOfInterest(double xMeters, double yMeters) {
                this.position = new Translation2d(xMeters, yMeters);
            }
        }

        // Blue left coral station poses
        private static Pose2d defaultOuterStation = new Pose2d(1.585, 7.550, Rotation2d.fromDegrees(-49.992));
        private static Pose2d defaultInnerStation = new Pose2d(0.596, 6.807, Rotation2d.fromDegrees(-55));

        // Poses of interest
        public enum PoseOfInterest {
            BLU_PROCESSOR(5.973318, -0.00381, 90), // Taken from April Tag coordinates
            RED_PROCESSOR(11.56081,	8.05561,	270), // Taken from April Tag coordinates

            BLUE_RIGHT_OUTER_STATION(mirrorPose(defaultOuterStation)), // Taken from PathPlanner coordinates
            BLUE_RIGHT_INNER_STATION(mirrorPose(defaultInnerStation)),
            BLUE_LEFT_OUTER_STATION(defaultOuterStation),
            BLUE_LEFT_INNER_STATION(defaultInnerStation),
            RED_RIGHT_OUTER_STATION(mirrorPose(flipPose(defaultOuterStation))),
            RED_RIGHT_INNER_STATION(mirrorPose(flipPose(defaultInnerStation))),
            RED_LEFT_OUTER_STATION(flipPose(defaultOuterStation)),
            RED_LEFT_INNER_STATION(flipPose(defaultInnerStation));

            // BLU_RIGHT_STATION(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), 55),
            // BLU_LEFT_STATION(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), 305),
            // RED_RIGHT_STATION(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), -125),
            // RED_LEFT_STATION(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), 125);
            // 7.618 blue x for net
            // 9.922 red x for net

            public final Pose2d pose;

            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(omegaDeg));
            }

            private PoseOfInterest(Pose2d pose) {
                this.pose = pose;
            }
        }
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = 31; // Left motor consistent with drivetrain left side
        public static final int kMotorPort1 = 32; // Right motor consistent with drivetrain right side
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(0.5);
        public static final double kMinElevatorHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(83.25); // 83.25 //81.15);
        
        public static final double defaultMaxOperatorControlVolts = 1.5;
        public static final double fasterMaxOperatorControlVolts = 4;

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        //public static final double minHeightMeters = 0;
        //public static final double maxHeightMeters = 0.85; // Temporary

        // https://wcproducts.com/collections/gearboxes/products/wcp-single-stage-gearbox  Inches.of(0.25).in(Meters)
        // Still set to WAPUR elevator units, need to be changed
        public static final double kGearRatio = 10.909;
        public static final double kElevatorStages = 3;
        public static final double kVelocityConversionFactor = (24.0/22.0) * kElevatorStages * (1/kGearRatio) * 24 * 0.00635 / 60.0; //Gear ratio & chain pitch & rpm -> m/s
        public static final double kPositionConversionFactor = (24.0/22.0) * kElevatorStages * (1/kGearRatio) * 24 * 0.00635; //Gear ratio & chain pitch

        public static class Electrical {
            public static final double kVoltageCompensation = 10.5; 
            public static final double kCurrentLimit = 60;
        }


        public static class Control {
            // PID constants
            public static final double kP = 30; //30;
            public static final double kI = 0.0;
            public static final double kD = 0;//2.5;//1;

            // Feedforward constant
            public static final double kS = 0.17213;  // 0.059004; //Static gain (voltage)
            public static final double kG = 0.77965;  // 0.77763; //Gravity gain (voltage)
            public static final double kV = 3.0126;  // 2.8829; // velocity game
            public static final double kA = 0.73282; // Acceleration Gain

            public static final Constraints kProfileConstraints = new Constraints(3, 5.25);//7);//5.25);//7);//5.25);
        }
    }

    public static class ElevatorSimConstants {
        // RANDOM ports
        public static final int kSimMotorPort0 = 20;
        public static final int kSimMotorPort1 = 21;
        
        public static final double kElevatorGearing = 60.0/11.0;
        public static final double kCarriageMass = Units.lbsToKilograms(30); //kg
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0); //taken from example, do not know what this is
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72.0);

        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;

        public static class Control {
            // PID constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kP = 6.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            // Feedforward constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0.0; // Static gain (voltage)
            public static final double kG = 2.8605; // Gravity gain (voltage)
            public static final double kV = 0.0; // Velocity gain
            public static final double kA = 0.0; // Acceleration Gain

            public static final Constraints kProfileConstraints = new Constraints(1, 3);
        }
    }

    public static final class GrabberConstants {
        public static final int kLeftMotorPort = 41;
        public static final int kRightMotorPort = 42;
        
        public static final double kCurrentLimit = 20; 
        public static final double kGearRatio = 20;
        public static final double kPositionConversionFactor = Math.PI * 2 * (1/kGearRatio);

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;
    }

    public static class WristConstants {
        public static final int kLeadMotorPort = 61; // Left motor consistent with drivetrain left side

        public static final double wristAngleToleranceRadians = Units.degreesToRadians(2);
        public static final double kMinWristAngleRadians = Units.degreesToRadians(-90);
        public static final double kMaxWristAngleRadians = Units.degreesToRadians(90);

        public static final double maxOperatorControlVolts = 1;
        public static final double kSmartCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = true;

        // Source: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2 * Math.PI; // (1.0/125.0) * (32.0/50.0) * (2 * Math.PI) // rotations to radians
        public static final double kVelocityConversionFactor = 2 * Math.PI / 60; // (1.0/125.0) * (32.0/50.0) * (2 * Math.PI) / 60; // rpm to radians/second
        public static final double kZeroOffsetRadians = -1.7236260000051038; // -0.6595923267948967;

        public static final class Control {
            // PID constants
            public static final double kP = 18;
            public static final double kI = 0.0;
            public static final double kD = 0;

            // Feedforward constants
            public static final double kS = 0.16629; //0.26649; // static gain in volts
            public static final double kG = 0.13459; // 0.13459; // gravity gain in volts
            public static final double kV = 1.8105; // 0.92013; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared

            public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI); // new Constraints(10 * Math.PI, 6 * Math.PI);
        }
    }

    public static class WristSimConstants {
        // Values are NOT CORRECT
        public static final double kWristGearingReductions = 125.0;
        public static final double kWristLength = Units.feetToMeters(1); // Excludes a 5 inch fixed piece
        public static final double kWristMass = 2.0;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class Control {
            public static final double kP = 18.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0553; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared

            public static final Constraints kProfileConstraints = new Constraints(2, 2);
        }


    }

    public static class IndexConstants {
        public static final int kLeadMotorPort = 51;

        public static final double kCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = true;
    }

    public static class BeamBreakConstants{
        public static final int transferBeamBreakPort = 0;
    }

    public static class CANRangeConstants {
        public static final int grabberCANRangeId = 6;
        public static final double grabberCANRangeTriggerDistanceMeters = 0.0762; // 3 inches
    }

    public static class LEDConstants {
        public static final double kHPSignalTime = 3.0;

        /// Digital input-output pins on the RIO
        public static class LEDDIOConstants {
            public static final int kDIOPort1 = 7;
            public static final int kDIOPort2 = 8;
            public static final int kDIOPort3 = 9;
        }

        public static class LEDOnRIOConstants {
            public static final int kPWMPort = 0;
            public static final int kLength = 72;

            public static final double kFlashSeconds = 0.1;
            public static final int kEmptyStateBrightness = 100;
            public static final int kFlashingStateBrightness = 100;
        }

        public static enum LEDStates {
            IDLE(false, false, false),
            CORAL_INDEXED(true, false, false),
            HUMAN_PLAYER_SIGNAL(false, true, false),
            ALGAE(true, true, false),
            AUTO_ALIGNED(false, false, true),
            AUTO_ALIGNING(false, false, false),
            OFF(true, false, true),
            ELEVATOR_ZEROED(false, true, true),
            
            // Rainbow Rumble specific color states for the rainbow bonus
            RED_HP_SIGNAL(true, true, true),
            ORANGE_HP_SIGNAL(false,false,false),
            YELLOW_HP_SIGNAL(false,false,false),
            GREEN_HP_SIGNAL(false,false,false),
            BLUE_HP_SIGNAL(false,false,false),
            PURPLE_HP_SIGNAL(false,false,false),
            WHITE_HP_SIGNAL(false,false,false);

            public final boolean onesPlace;
            public final boolean twosPlace;
            public final boolean foursPlace;
            private LEDStates(boolean onesPlace, boolean twosPlace, boolean foursPlace) {
                this.onesPlace = onesPlace;
                this.twosPlace = twosPlace;
                this.foursPlace = foursPlace;
            } 
        }
    }

    private Constants() {
        throw new NotImplementedException();
    }
}
