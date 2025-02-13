package frc.robot.utils;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.StartPositions;
import frc.robot.Constants.GameConstants.TeamColors;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;

//TODO: Either make this a singleton class or make putX methods static
public class Elastic {

    private static Elastic instance;
    private SendableChooser<TeamColors> teamChooser;
    private SendableChooser<StartPositions> startPositionChooser;

    private Elastic() {
        teamChooser = new SendableChooser<>();
        teamChooser.setDefaultOption(GameConstants.kTeamColor.color, GameConstants.kTeamColor);
        teamChooser.addOption(TeamColors.kTeamColorRed.color, TeamColors.kTeamColorRed);
        teamChooser.addOption(TeamColors.kTeamColorBlue.color, TeamColors.kTeamColorBlue);

        startPositionChooser = new SendableChooser<>();
        startPositionChooser.setDefaultOption(GameConstants.kStartPosition.name, GameConstants.kStartPosition);
        // startPositionChooser.addOption(StartPositions.kStartA.name, StartPositions.kStartA);
        for(StartPositions position : StartPositions.values()) {
            startPositionChooser.addOption(position.name, position);
        }
    }

    public static Elastic getInstance() {
        if (instance == null) {
            instance = new Elastic();
        }
        return instance;
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

    public void putIndexPossession(IndexPossession indexPossession) {
        // System.out.println("indexPossession: " + indexPossession.name);
        SmartDashboard.putString("indexPossession", indexPossession.name);
    }

    public void putGrabberPossession(GrabberPossession grabberPossession) {
        // System.out.println("grabberPossession: " + grabberPossession.name);
        SmartDashboard.putString("grabberPossession", grabberPossession.name);
    }

    public void updateInterface() {
        
    }

    public TeamColors getSelectedTeamColor() {
        return teamChooser.getSelected();
    }

    public StartPositions getSelectedStartPosition() {
        return startPositionChooser.getSelected();
    }
}