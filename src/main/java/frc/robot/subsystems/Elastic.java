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
import frc.robot.Constants.GameConstants.FlippedAuton;
import frc.robot.Constants.GameConstants.TeamColors;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;

//TODO: Either make this a singleton class or make putX methods static
public class Elastic {
    // private SendableChooser<TeamColors> teamChooser;
    private SendableChooser<FlippedAuton> flippedAutonChooser;
    private HashMap<Alliance, String> AllianceNames; 
    private Alliance currentAllianceName;

    public Elastic() {
        /*
        teamChooser = new SendableChooser<>();
        teamChooser.setDefaultOption(GameConstants.kTeamColor.color, GameConstants.kTeamColor);
        teamChooser.addOption(TeamColors.kTeamColorRed.color, TeamColors.kTeamColorRed);
        teamChooser.addOption(TeamColors.kTeamColorBlue.color, TeamColors.kTeamColorBlue);
        SmartDashboard.putData(teamChooser);
        */
        
        flippedAutonChooser = new SendableChooser<>();
        flippedAutonChooser.setDefaultOption(GameConstants.flippedAuton.name, GameConstants.flippedAuton);
        flippedAutonChooser.addOption(FlippedAuton.kNotFlipped.name, FlippedAuton.kNotFlipped);
        flippedAutonChooser.addOption(FlippedAuton.kFlipped.name, FlippedAuton.kFlipped);
        SmartDashboard.putData(flippedAutonChooser);

        AllianceNames = new HashMap<>();
        AllianceNames.put(Alliance.Blue, "Blue");
        AllianceNames.put(Alliance.Red, "Red");
        this.putSelectedTeamColor();
        
        // Initialize fields
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
        // System.out.println("grabberPossession: " + grabberPossession.name);
        SmartDashboard.putString("grabberPossession", grabberPossession.name);
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

    /** Returns true to flip the auton when in autonomous mode and the auton is selected as flipped */
    public boolean getPathPlannerFlipped() {
        return DriverStation.isAutonomous() && flippedAutonChooser.getSelected().isFlipped;
    }
}
