// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.AutonSides;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.RobotContainer;

public class Elastic {
    // private SendableChooser<TeamColors> teamChooser;
    private Field2d field;
    private SendableChooser<AutonSides> autonSideChooser;
    private HashMap<Alliance, String> AllianceNames; 
    private Alliance currentAllianceName;

    private static Elastic inst;

    public static Elastic getInstance() {
        if (inst == null) {
            inst = new Elastic();
        }
        return inst;
    }

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

        // Maps the Alliance enum that the Driver Station returns to string names
        AllianceNames = new HashMap<>();
        AllianceNames.put(Alliance.Blue, "Blue");
        AllianceNames.put(Alliance.Red, "Red");
        this.putSelectedTeamColor();
        
        // Initialize fields, because otherwise they're only updated when teleop is enabled
        this.putBoolean("safeToFeedCoral", false);
        this.putBoolean("safeToMoveElevator", false);
        this.putBoolean("isAutoAligned", false);
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

    /** Gets the selected team color from the driver station */
    public Alliance getSelectedTeamColor() {
        return GameConstants.teamColor;
    }

    public void putSelectedTeamColor() {
        this.putSelectedTeamColor(this.getSelectedTeamColor());
    }

    public void putSelectedTeamColor(Alliance alliance) {
        SmartDashboard.putString(
            "teamColor",
            AllianceNames.get(alliance));
        this.currentAllianceName = alliance;
    }

    /** Sends the selected team color to the dashboard if it has changed */
    public void updateTeamColor() {
        if (this.getSelectedTeamColor() != this.currentAllianceName) {
            this.putSelectedTeamColor();
        }
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

    public void updateIsAutoAligned(BooleanSupplier isAutoAligned) {
        this.putBoolean("isAutoAligned", isAutoAligned.getAsBoolean());
    }

    /** Returns true to mirror the auton from the left side to the right side
     * when in autonomous mode and the auton is selected as mirrored to the right side */
    public boolean getPathPlannerMirrored() {
        return DriverStation.isAutonomous() && (GameConstants.autonSide == AutonSides.Right);// autonSideChooser.getSelected().isRightSide;
    }
}
