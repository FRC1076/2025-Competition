package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.StartPositions;
import frc.robot.Constants.GameConstants.TeamColors;

//TODO: Either make this a singleton class or make putX methods static
public class Elastic {
    private SendableChooser<TeamColors> teamChooser;
    private SendableChooser<StartPositions> startPositionChooser;

    public Elastic() {
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

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public void putString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    // TODO: public void putIndexState(string or boolean?)

    // TODO: public void putWristevatorState(string?)

    // TODO: public void putGrabberState(string?)

    public void updateInterface() {
        
    }

    public TeamColors getSelectedTeamColor() {
        return teamChooser.getSelected();
    }

    public StartPositions getSelectedStartPosition() {
        return startPositionChooser.getSelected();
    }
}
