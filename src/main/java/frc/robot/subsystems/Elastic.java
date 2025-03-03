// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.AutonSides;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;

public class Elastic {
    // private SendableChooser<TeamColors> teamChooser;
    private SendableChooser<AutonSides> autonSideChooser;
    private HashMap<Alliance, String> AllianceNames; 
    private Alliance currentAllianceName;

    public Elastic() {
        /* This is a dropdown menu on the SmartDashboard that allows the user to select whether 
        the auton is on the left (default) or the right side of the field.
        */
        autonSideChooser = new SendableChooser<>();
        autonSideChooser.setDefaultOption(GameConstants.autonSide.name(), GameConstants.autonSide);
        autonSideChooser.addOption(AutonSides.kLeft.name(), AutonSides.kLeft);
        autonSideChooser.addOption(AutonSides.kRight.name(), AutonSides.kRight);
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
        return DriverStation.getAlliance().orElse(Alliance.Blue);
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
        return DriverStation.isAutonomous() && autonSideChooser.getSelected().isRightSide;
    }
}
