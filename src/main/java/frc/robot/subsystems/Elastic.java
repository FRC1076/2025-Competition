// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GameConfig;
import frc.robot.SystemConfig;
import frc.robot.Constants.FieldConstants.CoralLevel;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.AutonSides;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Elastic {

    private static Elastic inst;
    // private SendableChooser<TeamColors> teamChooser;
    private Field2d field;
    private SendableChooser<AutonSides> autonSideChooser;
    private SendableChooser<Command> autoChooser;
    private HashMap<Alliance, String> AllianceNames; 
    private Alliance currentAllianceName;

    private Elastic() {
        /* This is a dropdown menu on the SmartDashboard that allows the user to select whether 
        the auton is on the left (default) or the right side of the field.
        */
        field = new Field2d();
        SmartDashboard.putData(field);

        autonSideChooser = new SendableChooser<>();
        autonSideChooser.setDefaultOption(GameConstants.autonSide.name(), GameConstants.autonSide);
        autonSideChooser.addOption(AutonSides.Left.name(), AutonSides.Left);
        autonSideChooser.addOption(AutonSides.Right.name(), AutonSides.Right);
        SmartDashboard.putData(autonSideChooser);
        
        // Initialize fields, because otherwise they're only updated when teleop is enabled
        this.putBoolean("safeToFeedCoral", false);
        this.putBoolean("safeToMoveElevator", false);
        this.putBoolean("isAutoAligned", false);
    }

    public static Elastic getInstance(){
        if (inst == null) {
            inst = new Elastic();
        }
        return inst;
    }

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public void putString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    public void putGrabberPossession(GrabberPossession grabberPossession) {
        // System.out.println("grabberPossession: " + grabberPossession.name());
        SmartDashboard.putString("grabberPossession", grabberPossession.name());
    }

    public void updateTransferBeamBreak(boolean beamBroken) {
        this.putBoolean("transferBB", beamBroken);
    }

    public void updateSafeToFeedCoral(BooleanSupplier safeToFeedCoral) {
        this.putBoolean("safeToFeedCoral", safeToFeedCoral.getAsBoolean());
    }

    public void updateSafeToMoveElevator(BooleanSupplier safeToMoveElevator) {
        this.putBoolean("safeToMoveElevator", safeToMoveElevator.getAsBoolean());
    }

    public void putAutopilotTargetLevel(CoralLevel level) {
        this.putString("stagedCoralLevel", level.name());
    }

    public void updateIsAutoAligned(BooleanSupplier isAutoAligned) {
        this.putBoolean("isAutoAligned", isAutoAligned.getAsBoolean());
    }

    // This should only be called AFTER the drivetrain is constructed
    public void buildAutoChooser(DriveSubsystem drive){
        autoChooser = AutoBuilder.buildAutoChooser(GameConfig.defaultAuton);
        autoChooser.addOption(
            "SeedPoseBlue", 
            Commands.runOnce(() -> drive.resetPose(new Pose2d(7.177, 5.147, Rotation2d.fromDegrees(180))))
        );
        autoChooser.addOption(
            "SeedPoseRed", 
            Commands.runOnce(() -> drive.resetPose(new Pose2d(10.380, 3.043, Rotation2d.fromDegrees(0))))
        );
        SmartDashboard.putData(autoChooser);
    }

    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }

    /** Returns true to mirror the auton from the left side to the right side
     * when in autonomous mode and the auton is selected as mirrored to the right side */
    public boolean getPathPlannerMirrored() {
        return DriverStation.isAutonomous() && autonSideChooser.getSelected().isRightSide;
    }
}
