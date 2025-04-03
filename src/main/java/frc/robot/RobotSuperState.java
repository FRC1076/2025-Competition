// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;

/** A singleton class that allows robot state data to be global across the code */
public class RobotSuperState {

    private static RobotSuperState inst;

    private SwerveDriveState driveState;
    private WristevatorState wristevatorState;
    private WristevatorState wristevatorGoal;
    private double elevatorHeightMeters;
    private Rotation2d wristAngle;
    private IndexState indexState;
    private GrabberState grabberState;
    private GrabberPossession possession;
    private Optional<Pose2d> targetPose; // Autoalign target pose
    private boolean safeToMoveElevator = false;
    private boolean safeToFeedCoral = false;
    private boolean isAutoaligned = false;

    public static RobotSuperState getInstance() {
        if (inst == null) {
            inst = new RobotSuperState();
        }
        return inst;
    }

    private RobotSuperState() {
        driveState = new SwerveDriveState();
        wristevatorGoal = WristevatorState.TRAVEL;
        wristevatorState = WristevatorState.TRAVEL;
        elevatorHeightMeters = 0;
        wristAngle = Rotation2d.kZero;
        indexState = IndexState.BACKWARDS;
        grabberState = GrabberState.IDLE;
        possession = GrabberPossession.EMPTY;
    }

    public void updateTargetPose(Pose2d pose){
        targetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTargetPose(){
        return targetPose;
    }

    public void updateDriveState(SwerveDriveState driveState){
        this.driveState.FailedDaqs = driveState.FailedDaqs;
        this.driveState.SuccessfulDaqs = driveState.SuccessfulDaqs;
        this.driveState.ModulePositions = driveState.ModulePositions;
        this.driveState.Speeds = driveState.Speeds;
        this.driveState.ModuleTargets = driveState.ModuleTargets;
        this.driveState.ModuleStates = driveState.ModuleStates;
        this.driveState.Timestamp = driveState.Timestamp;
        this.driveState.RawHeading = driveState.RawHeading;
        this.driveState.Pose = driveState.Pose;
        this.driveState.OdometryPeriod = driveState.OdometryPeriod;
    }

    public boolean isSafeToMoveElevator() {
        return safeToMoveElevator;
    }

    public void setSafeToMoveElevator(boolean safeToMoveElevator) {
        this.safeToMoveElevator = safeToMoveElevator;
    }

    // Getter and Setter for safeToFeedCoral
    public boolean isSafeToFeedCoral() {
        return safeToFeedCoral;
    }

    public void setSafeToFeedCoral(boolean safeToFeedCoral) {
        this.safeToFeedCoral = safeToFeedCoral;
    }

    // Getter and Setter for isAutoaligned
    public boolean isAutoaligned() {
        return isAutoaligned;
    }

    public void setAutoaligned(boolean isAutoaligned) {
        this.isAutoaligned = isAutoaligned;
    }

    public void updateWristevatorState(WristevatorState wristevatorState){
        this.wristevatorState = wristevatorState;
    }

    public void updateWristevatorGoal(WristevatorState goal){
        this.wristevatorGoal = goal;
    }

    public void updateIndexState(IndexState indexState){
        this.indexState = indexState;
    }

    public void updateGrabberState(GrabberState grabberState){
        this.grabberState = grabberState;
    }

    public void updateWristAngle(Rotation2d wristAngle){
        this.wristAngle = wristAngle;
    }

    public void updateElevatorHeight(double elevatorHeightMeters){
        this.elevatorHeightMeters = elevatorHeightMeters;
    }

    public void updatePossession(GrabberPossession possession){
        this.possession = possession;
    }

    public SwerveDriveState getDriveState() {
        return driveState.clone();
    }

    public Pose2d getPose() {
        return driveState.Pose;
    }

    public int getFailedDaqs() {
        return driveState.FailedDaqs;
    }

    public int getSuccessfulDaqs() {
        return driveState.SuccessfulDaqs;
    }

    public Rotation2d getRawHeading() {
        return driveState.RawHeading;
    }

    public WristevatorState getWristevatorState() {
        return wristevatorState;
    }

    public WristevatorState getWristevatorGoal() {
        return wristevatorGoal;
    }

    public IndexState getIndexState() {
        return indexState;
    }

    public GrabberState getGrabberState() {
        return grabberState;
    }

    public Rotation2d getWristAngle() {
        return wristAngle;
    }

    public double getElevatorHeight() {
        return elevatorHeightMeters;
    }

    public GrabberPossession getPossession() {
        return possession;
    }

    //NOTE: Only logs superstructure states, since subsystems are already logged
    public void logSuperstructureToAkit() {
        Logger.recordOutput("Superstructure/WristevatorState",wristevatorState.name());
        Logger.recordOutput("Superstructure/WristevatorGoal",wristevatorGoal.name());
        Logger.recordOutput("Superstructure/GrabberState",grabberState.name());
        Logger.recordOutput("Superstructure/IndexState",indexState.name());
    }
}
