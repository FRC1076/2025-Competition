// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.apache.commons.lang3.NotImplementedException;

import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class Akit {
        public static final int currentMode = 0;
    }
    
    public static class DriveConstants {

        public static class DriverControlConstants {
            public static final double singleClutchTranslationFactor = 0.5;
            public static final double singleClutchRotationFactor = 0.5;
            public static final double doubleClutchTranslationFactor = 0.3;
            public static final double doubleClutchRotationFactor = 0.35;
            public static final double maxTranslationSpeedMPS = 5;
            public static final double maxRotationSpeedRadPerSec = 5;
        }

        public static class PathPlannerConstants {
            public static final PathConstraints pathConstraints = new PathConstraints(4.69, 25, Units.degreesToRadians(1080), Units.degreesToRadians(1080));
            public static final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.kZero);
            public static final double pathGenerationToleranceMeters = 0.011; // technically it's anything larger than 0.01, but I'm adding .001 just to be safe
        }

        public enum HeadingMode {
            NORMAL,
            POINT_TO_REEF,
            LEFT_CORAL_STATION,
            RIGHT_CORAL_STATION,
            STRAIGHT,
        }

        public enum ClutchMode {
            NORMAL,
            SINGLE_CLUTCH,
            DOUBLE_CLUTCH,
        }

    }

    public static class SuperstructureConstants {

        //Grabber Possession State
        public enum GrabberPossession {
            EMPTY,
            CORAL,
            ALGAE
        }

        //Index Possession State
        public enum IndexPossession {
            EMPTY,
            CORAL
        }

        //Grabber State
        public enum GrabberState {

            IDLE(0,0),
            
            ALGAE_INTAKE(-12,-12),
            CORAL_INTAKE(5,5),

            ALGAE_OUTTAKE(6,6),
            CORAL_OUTTAKE(12,12);


            public final double leftVoltage;
            public final double rightVoltage;

            private GrabberState(double leftVoltage, double rightVoltage) {
                this.leftVoltage = leftVoltage;
                this.rightVoltage = rightVoltage;
            }
        }

        //Index State
        public enum IndexState {
            EMPTY_IDLE(false),
            CORAL_INTAKE(true),
            CORAL_TRANSFER(true),
            CORAL_IDLE(false);
            public final boolean running; //Whether or not the indexer motors are running
            private IndexState(boolean running) {
                this.running = running;
            }
        }

        //Represent the elevator height and wrist angle for different positions, the full position of the grabber
        //Should we have an eject state with an optional elevator height? just to immediately eject if a game piece is stuck
        public enum GrabberPosition {
            
            TRAVEL(0.08128,90),
            ALGAE_TRAVEL(0.08128, 65),

            CORAL_TRANSFER(0.08128,-23.5), //same as CORAL_DIRECT_INTAKE

            L1(-1,-1), //Placeholder
            L2(0.71628,-35),
            L3(1.11252,-35),
            L4(1.8161,-45),

            GROUND_INTAKE(-1, -1),
            LOW_INTAKE(0.9144,-35),
            HIGH_INTAKE(1.30556,-35),

            PROCESSOR(0.184277,0),
            NET(1.8288,65);

            public final double elevatorHeightMeters;
            public final Rotation2d wristAngle;
            
            private GrabberPosition(double elevatorHeightMeters, double wristAngleDegrees) {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.wristAngle = Rotation2d.fromDegrees(wristAngleDegrees);
            }
        }
    }

    /** Contains data about the field */
    public static class FieldConstants {
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {

            //Blue Reef
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, 3.930, 3.400, 4.210, 3.200),
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, 3.658, 4.180, 3.658, 3.860),
            BLU_REEF_LK(19, 4.073906, 4.745482, 120.0, 4.190, 4.850, 3.920, 4.670),
            BLU_REEF_JI(20, 4.904740, 4.745482, 60.0, 5.060, 4.690, 4.780, 4.840),
            BLU_REEF_HG(21, 5.321046, 4.025900, 0.0, 5.321, 3.850, 5.321, 4.170),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, 4.780, 3.220, 5.060, 3.340),

            //Red Reef
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, 15.350, 3.210, 13.640, 3.340),
            RED_REEF_BA(7, 13.890498, 4.025900, 0.0, 13.910, 3.860, 13.910, 4.190),
            RED_REEF_DC(8, 13.474446, 4.745482, 60.0, 13.640, 4.690, 13.360, 4.830),
            RED_REEF_FE(9, 12.643358, 4.745482, 120.0, 12.780, 4.830, 12.510, 4.680),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, 12.220, 4.190, 12.220, 3.850),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, 12.490, 3.360, 12.790, 3.210);


            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d AprilTag;
            public final int AprilTagID;

            private ReefFace(int AprilTagID, double AT_x, double AT_y, double AT_theta, double L_x, double L_y, double R_x, double R_y) {
                this.AprilTagID = AprilTagID;
                this.AprilTag = new Pose2d(AT_x, AT_y, Rotation2d.fromDegrees(AT_theta));
                this.leftBranch = new Pose2d(L_x, L_y,Rotation2d.fromDegrees(AT_theta));
                this.rightBranch = new Pose2d(R_x, R_y,Rotation2d.fromDegrees(AT_theta));
            }
        }

        //Generic rotation-agnostic points of interest
        public enum PointOfInterest {
            BLU_REEF(4.487,4.010),
            RED_REEF(13.062,4.010);

            public final Translation2d position;
            private PointOfInterest(double xMeters, double yMeters) {
                this.position = new Translation2d(xMeters,yMeters);
            }
        }

        //Poses of interest
        public enum PoseOfInterest {
            BLU_PROCESSOR(0,0,0), //Placeholder
            RED_PROCESSOR(0,0,0), //Placeholder
            BLU_RIGHT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),55),
            BLU_LEFT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),305),
            RED_RIGHT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),-125),
            RED_LEFT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),125);

            public final Pose2d pose;
            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters,yMeters,Rotation2d.fromDegrees(omegaDeg));
            }
        }
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = -1;
        public static final int kMotorPort1 = -1;
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(0.5);
        public static final double maxOperatorControlVolts = 6;

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        //Heights measured in meters
        public static final double lowHeight = 0;
        public static final double autonHeight = 0;
        public static final double midHeight = 0;
        public static final double highHeight = 0;

        //public static final double minHeightMeters = 0;
        //public static final double maxHeightMeters = 0.85; //Temporary

        //https://wcproducts.com/collections/gearboxes/products/wcp-single-stage-gearbox  Inches.of(0.25).in(Meters)
        // still set to WAPUR elevator units, need to be changed
        public static final double kVelocityConversionFactor = (11/60.0) * 22 * 0.00635 / 60.0; //Gear ratio & chain pitch & rpm -> m/s
        public static final double kPositionConversionFactor = (11/60.0) * 22 * 0.00635; //Gear ratio & chain pitch
        public static class Electrical {
            public static final double kVoltageCompensation = 12;
            public static final double kCurrentLimit = 40;
        }


        public static class Control {
            //PID Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kP = 8;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            //Feedforward Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0; //Static gain (voltage)
            public static final double kG = 0.6; //Gravity gain (voltage)
            public static final double kV = 12; // velocity game
            public static final double kA = 0; //Acceleration Gain
        }
    }

    public static class ElevatorSimConstants {
        //RANDOM ports
        public static final int kSimMotorPort0 = 20;
        public static final int kSimMotorPort1 = 21;
        
        public static final double kElevatorGearing = 60.0/11.0;
        public static final double kCarriageMass = Units.lbsToKilograms(30); //kg
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0); //taken from example, do not know what this is
        public static final double kMinElevatorHeightMeters = 0;
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72.0);

        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;

        public static class Control {
            //PID Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kP = 6;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            //Feedforward Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0; // Static gain (voltage)
            public static final double kG = 2.8605; // Gravity gain (voltage)
            public static final double kV = 0; // velocity gain
            public static final double kA = 0; // Acceleration Gain
        }
    }

    public static final class GrabberConstants {
        public static final int kLeftMotorPort = -1;
        public static final int kRightMotorPort = -1;
        
        public static final double kCurrentLimit = 40; 
    }

    public static class WristConstants {
        public static final int kLeadMotorPort = -2;
        public static final int kFollowMotorPort = -3;

        public static final double wristAngleToleranceRadians = Units.degreesToRadians(1);
        public static final double maxOperatorControlVolts = 6;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;

        // source: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2*Math.PI; // rotations to radians
        public static final double kVelocityConversionFactor = (2*Math.PI) / 60.0; // rpm to radians/second

        public static final class Control {
            // PID constants
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            // feed forward constants
            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared
        }
    }

    public static class WristSimConstants {
        // values are NOT CORRECT
        public static final double kWristGearingReductions = 125;
        public static final double kWristLength = Units.feetToMeters(1); // excludes a 5 inch fixed piece
        public static final double kWristMass = 2;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class Control {
            public static final double kP = 18;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0553; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared
        }
    }

    public static class IndexConstants {
        public static final int kLeadMotorPort = -10;
        public static final int kFollowMotorPort = -10;

        public static final double kCurrentLimit = 40;
        public static final double kIndexVoltage = 6;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;
    }

    public static class BeamBreakConstants{
        public static final int indexBeamBreakPort = 0;
        public static final int transferBeamBreakPort = 1;
        public static final int grabberBeamBreakPort = 2;
    }

    private Constants() {
        throw new NotImplementedException();
    }
}
