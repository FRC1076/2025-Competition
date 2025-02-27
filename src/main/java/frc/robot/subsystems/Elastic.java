// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;

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

    public void updateInterface() {
        
    }

    public Alliance getSelectedTeamColor() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public void putSelectedTeamColor() {
        SmartDashboard.putString(
            "teamColor",
            AllianceNames.get(this.getSelectedTeamColor()));
    }

    public void putSelectedTeamColor(Alliance alliance) {
        SmartDashboard.putString(
            "teamColor",
            AllianceNames.get(alliance));
    }

    public boolean getPathPlannerFlipped() {
        return DriverStation.isAutonomous() && flippedAutonChooser.getSelected().isFlipped;
    }
}
