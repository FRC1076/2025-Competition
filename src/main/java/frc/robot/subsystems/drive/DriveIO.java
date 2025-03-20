// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotSuperState;
import edu.wpi.first.math.geometry.Pose2d;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;



/**
 * A common interface that allows the subsystem code to interact with a drivetrain,
 * while abstracting away implementation, to allow for polymorphism (sim or hardware implementations)
 */
public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs extends SwerveDriveState {

        public double[] odometryTimestamps = new double[] {};
        public ChassisSpeeds[] odometrySpeeds = new ChassisSpeeds[] {};
        public Pose2d[] odometryPoses = new Pose2d[] {};
        public Rotation2d[] odometryHeadings = new Rotation2d[] {};

        public Rotation2d operatorForwardDirection = new Rotation2d();

        // Check to see if these are actually logged
        public void fromSwerveDriveState(SwerveDriveState stateIn) {
            this.Pose = stateIn.Pose;
            this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
            this.FailedDaqs = stateIn.FailedDaqs;
            this.ModulePositions = stateIn.ModulePositions;
            this.Timestamp = stateIn.Timestamp;
            this.ModuleStates = stateIn.ModuleStates;
            this.ModuleTargets = stateIn.ModuleTargets;
            this.Speeds = stateIn.Speeds;
            this.RawHeading = stateIn.RawHeading;
            this.OdometryPeriod = stateIn.OdometryPeriod;
        }

    }

    /**
     * A POD class for transfering data between the subsystem, IO layer, and 
     */
    @AutoLog
    public static class ModuleIOInputs {

        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveStatorCurrentAmps = 0.0;

        public Rotation2d turnPosition = new Rotation2d();
        public double turnAppliedVolts = 0.0;
        public double turnStatorCurrentAmps = 0.0;

        public SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
        public SwerveModuleState[] odometryStates = new SwerveModuleState[] {};
        public double[] odometryTimestamps = new double[] {};
    }

    /** UpdateInputs MUST be called before updateModuleInputs */
    public abstract void updateInputs(DriveIOInputs inputs);
    public abstract void acceptRequest(SwerveRequest request);
    public abstract void updateModuleInputs(ModuleIOInputs inputs, int moduleIndex);
    public abstract Translation2d[] getModuleLocations();
    public abstract void addVisionMeasurement(Pose2d poseEstimate, double timestampSeconds, Matrix<N3, N1> StdDevs);
    public abstract void resetPose(Pose2d pose);
    public abstract void resetHeading();
    public abstract Pose2d getPose();
    public abstract void setAllianceRotation(Rotation2d allianceRotation);
    public abstract void periodic();
    public abstract void setDriveStatorCurrentLimit(double currentLimit);
    public abstract void updateState(RobotSuperState state);
}
