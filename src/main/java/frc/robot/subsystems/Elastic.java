// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.StartPositions;
import frc.robot.Constants.GameConstants.TeamColors;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.subsystems.superstructure.state.SuperState;
import frc.robot.subsystems.superstructure.state.Possession.PossessionState;

//TODO: Either make this a singleton class or make putX methods static
public class Elastic {

    private static Elastic inst = new Elastic();

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

    public void putPossessionState(PossessionState possessionState) {
        SmartDashboard.putString("Possession", possessionState.name());
    }

    public void putSuperState(SuperState superState) {
        SmartDashboard.putString("Superstate", superState.name());
    }

    public void putGoalState(SuperState goalState) {
        SmartDashboard.putString("Goalstate", goalState.name());
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
