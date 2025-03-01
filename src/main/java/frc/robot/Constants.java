// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.apache.commons.lang3.NotImplementedException;

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
        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        public static class Photonvision {

            public static final String driverCamName = "DRIVER_CAM"; //PV name of the driver camera TODO: Add sim driver camera implementation with wireframe

            //TODO: Update all of these values
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
                    12.389, 11.683, 11.513639, // 15 - 7.163, 15 - 2.892, 19.162,
                    0, 0, -20 //11.385, 17.961, -40 
                ),
                FRONT_RIGHT_CAM(
                    "FRONT_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    12.389, -11.683, 11.513639, // 15 - 7.163, -(15 - 2.892), 19.162, 
                    0, 0, 20 // -11.385, 17.961, 40
                ); //,
                /*
                REAR_LEFT_CAM(
                    "REAR_LEFT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    0, 0, 0,
                    0, 0, 0
                ),
                REAR_RIGHT_CAM("REAR_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    0, 0, 0,
                    0, 0, 0
                );
                */

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

        public static TeamColors kTeamColor = TeamColors.kTeamColorBlue;
        public static FlippedAuton flippedAuton = FlippedAuton.kNotFlipped;
        
        public enum TeamColors {
            kTeamColorBlue("BLUE"),
            kTeamColorRed("RED");

            public final String color;

            private TeamColors(String color) {
                this.color = color;
            }
        }

        public enum FlippedAuton {
            kNotFlipped(false, "Not Flipped"),
            kFlipped(true, "Flipped");

            public final boolean isFlipped;
            public final String name;

            private FlippedAuton(boolean isFlipped, String name) {
                this.isFlipped = isFlipped;
                this.name = name;
            }
        }
    }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class SystemConstants {
        public static final int currentMode = 0; // 0 is real, 1 is sim
        public static final boolean operatorSysID = false;
        public static final boolean driverSysID = false;
        public static final boolean logOdometry = false;
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
            public static final double maxTranslationSpeedMPS = 5.0;
            public static final double maxRotationSpeedRadPerSec = 5.0;
        }

        public static class PathPlannerConstants {
            public static final PathConstraints pathConstraints = new PathConstraints(0.25, 5, Units.degreesToRadians(360), Units.degreesToRadians(360));
            public static final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.kZero);
            public static final double pathGenerationToleranceMeters = 0.011; // Technically it's anything larger than 0.01, but I'm adding .001 just to be safe

            public static class Control {
                public static final PIDConstants transPID = new PIDConstants(5, 0, 0);
                public static final PIDConstants rotPID = new PIDConstants(5, 0, 0);
            }
        }

        public static final double kOdometryUpdateFrequency = 250.0;
    }

    public static class SuperstructureConstants {

        public static double algaePossessionCurrentThreshold = 20.0;

        public static Rotation2d algaeTravelAngle = Rotation2d.fromDegrees(65);
        public static Rotation2d coralTravelAngle = Rotation2d.fromDegrees(65);

        // Grabber Possession State
        public enum GrabberPossession {
            EMPTY(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG,
                "EMPTY"),
            CORAL(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG,
                "CORAL"),
            ALGAE(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG,
                "ALGAE"),
            TRANSFERRING(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG
                    : ElevatorSimConstants.Control.kG,
                "TRANSFERRING"
            );

            public final double wrist_kG;
            public final double elevator_kG;
            public final String name;

            private GrabberPossession(double wrist_kG, double elevator_kG, String name) {
                this.wrist_kG = wrist_kG;
                this.elevator_kG = elevator_kG;
                this.name = name;
            }
        }

        /*
        // Index Possession State
        public enum IndexPossession {
            EMPTY("EMPTY"),
            CORAL("CORAL");

            public final String name;

            private IndexPossession(String name) {
                this.name = name;
            }
        }
        */

        // Grabber State
        public enum GrabberState {

            IDLE(0, 0),
            
            ALGAE_INTAKE(-12, -12),
            CORAL_INTAKE(12, 12),

            ALGAE_OUTTAKE(12, 12),
            CORAL_OUTTAKE(12, 12),
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
            TRANSFER(2),
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
            
            TRAVEL(0.08128, 90, false),
            ALGAE_TRAVEL(0.08128, 65 , false),

            CORAL_TRANSFER(0.08128, -23.5, false), // Same as CORAL_DIRECT_INTAKE

            L1(0.08128, 90, false), // Placeholder
            L2(0.910, -35, false), //0.71628, -35),
            L3(1.348 + 2 * 0.00889, -35, true), //1.11252, -35),
            L4(2.109649 + 3 * 0.00635, -40.4130051, true), //1.8161, -45),

            GROUND_INTAKE(0.184277, -20, false),
            LOW_INTAKE(1.13789, -35, false),
            HIGH_INTAKE(1.7440645, -35, true),

            PROCESSOR(0.184277 + 0.15, 0, false),
            NET(2.109649 + 3 * 0.00635, 65, true);

            public final double elevatorHeightMeters;
            public final Rotation2d wristAngle;
            public final boolean elevatorClutch;
            
            private WristevatorState(double elevatorHeightMeters, double wristAngleDegrees, boolean elevatorClutch) {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.wristAngle = Rotation2d.fromDegrees(wristAngleDegrees);
                this.elevatorClutch = elevatorClutch;
            }
        }
    }

    /** Contains data about the field */
    public static class FieldConstants {
        public static final double fieldWidthMeters = Units.inchesToMeters(316.63); // Distance from one processor side to the other. From AprilTag coordinates in https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf

        private static final double branchOffset = Units.inchesToMeters(6.469);
        private static final Transform2d leftBranchTransform = new Transform2d(0.0, -branchOffset, Rotation2d.kZero);
        private static final Transform2d rightBranchTransform = new Transform2d(0.0, branchOffset, Rotation2d.kZero);
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {
            // IMPORTANT: Fudge factors are always positive and should be in meters (use the Units.inchesToMeters() method)

            // Blue Reef
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null),
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null),
            BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null),
            BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null),
            BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null),

            // Red Reef
            RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null),
            RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null),
            RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null),
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null);


            public final Double leftBranchFudgeTransform;
            public final Double rightBranchFudgeTransform;
            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d AprilTag;
            public final int aprilTagID;

            //AT stands for AprilTag
            private ReefFace(int aprilTagID, double aprilTagX, double aprilTagY, double aprilTagTheta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform) {
                this.aprilTagID = aprilTagID;
                this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
                this.leftBranchFudgeTransform = leftBranchFudgeTransform;
                this.rightBranchFudgeTransform = rightBranchFudgeTransform;

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

        // Poses of interest
        public enum PoseOfInterest {
            BLU_PROCESSOR(5.973318, -0.00381, 90), //Taken from April Tag coordinates
            RED_PROCESSOR(11.56081,	8.05561,	270), //Taken from April Tag coordinates
            BLU_RIGHT_STATION(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), 55),
            BLU_LEFT_STATION(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), 305),
            RED_RIGHT_STATION(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), -125),
            RED_LEFT_STATION(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), 125);

            public final Pose2d pose;

            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(omegaDeg));
            }
        }
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = 31; // Left motor consistent with drivetrain left side
        public static final int kMotorPort1 = 32; // Right motor consistent with drivetrain right side
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(0.5);
        public static final double kMinElevatorHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(82.75);//81.15);
        public static final double maxOperatorControlVolts = 4;

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

        /*
        public static final double kMaxVelocityMeters = 1.0;
        public static final double kMaxAccelerationMetersSquared = 0.5;
        */
        public static class Electrical {
            public static final double kVoltageCompensation = 10.5;
            public static final double kCurrentLimit = 60;
        }


        public static class Control {
            // PID constants
            public static final double kP = 30;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            // Feedforward constant
            public static final double kS = 0.059004; //Static gain (voltage)
            public static final double kG = 0.77763;//1.2;//0.97369; // 0.6 //Gravity gain (voltage)
            public static final double kV = 2.8829; // 12.0 // velocity game
            public static final double kA = 0.0; //Acceleration Gain

            public static final Constraints kProfileConstraints = new Constraints(4, 6);
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
        
        public static final double kCurrentLimit = 10; 
        public static final double kGearRatio = 45;
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
        public static final double kSmartCurrentLimit = 15.0;

        public static final boolean kLeadMotorInverted = true;

        // Source: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = (1.0/125.0) * (32.0/50.0) * (2 * Math.PI);// (1/117.1875) * 2 * Math.PI; // rotations to radians
        public static final double kVelocityConversionFactor = (1.0/125.0) * (32.0/50.0) * (2 * Math.PI) / 60;// (1/117.1875) * (2 * Math.PI) / 60.0; // rpm to radians/second

        public static final class Control {
            // PID constants
            public static final double kP = 18;
            public static final double kI = 0.0;
            public static final double kD = 0;

            // Feedforward constants
            public static final double kS = 0.26649; // static gain in volts
            //TODO: comment back in kG when wrist works
            public static final double kG = 0; //0.13593 // gravity gain in volts
            public static final double kV = 0.92013; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared

            public static final Constraints kProfileConstraints = new Constraints(10 * Math.PI, 1 * Math.PI); 
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
        // public static final int kFollowMotorPort = 52;

        public static final double kCurrentLimit = 20.0;
        // public static final double kIndexVoltage = 12.0;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = true;
    }

    public static class BeamBreakConstants{
        public static final int transferBeamBreakPort = 0;
    }

    public static class LEDConstants {
        /// Digital input-output pins on the RIO
        public static class LEDDIOConstants {
            public static final int kDIOPort1 = 7;
            public static final int kDIOPort2 = 8;
            public static final int kDIOPort3 = 9;
        }

        public static class LEDOnRIOConstants {
            public static final int kPWMPort = 0;
            public static final int kLength = 100;

            public static final double kFlashSeconds = 0.75;
            public static final int kEmptyStateBrightness = 50;
            public static final int kFlashingStateBrightness = 100;
        }

        public static enum LEDStates {
            IDLE(false, false, false),
            CORAL_INDEXED(true, false, false),
            HUMAN_PLAYER_CAN_DROP(false, true, false),
            ALGAE(true, true, false),
            AUTO_ALIGNED(false, false, true);

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
