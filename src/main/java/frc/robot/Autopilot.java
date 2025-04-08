package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
<<<<<<< HEAD:src/main/java/frc/robot/commands/auto/Autopilot.java
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.Constants.FieldConstants.CoralLevel;
import frc.robot.RobotSuperState;
=======
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants.ReefLevel;
>>>>>>> c6488a3e31c460353762419c93a31cff96c03d0f:src/main/java/frc/robot/Autopilot.java
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;

import java.util.Map;
import java.util.Set;
import java.util.HashMap;

/**
 * A subsystem that executes autonomous teleop routines requiring multiple subsystems
 */
public final class Autopilot {

    private static Autopilot inst;
    
    private final SuperstructureCommandFactory m_superstructureCommands;
    private final DriveCommandFactory m_driveCommands;
    private final DriveSubsystem m_drive;
    private final Superstructure m_superstructure;
    private final Map<CoralLevel,Command> coralCommandMap = new HashMap<>();
    private final Map<CoralLevel,Command> coralAutoCommandMap = new HashMap<>();
    private CoralLevel targetLevel = CoralLevel.L1;

    
    private Autopilot(){
        // Constructor is private to enforce singleton pattern
    }

    public void registerDrive(DriveSubsystem drive){
        m_driveCommands = drive.CommandBuilder;
        m_drive = drive;
    }

    public void registerSuperstructure(Superstructure superstructure){
        m_superstructureCommands = superstructure.CommandBuilder;
        m_superstructure = superstructure;
        coralCommandMap.put(CoralLevel.L1,m_superstructureCommands.preL1());
        coralCommandMap.put(CoralLevel.L2,m_superstructureCommands.preL2());
        coralCommandMap.put(CoralLevel.L3,m_superstructureCommands.preL3());
        coralCommandMap.put(CoralLevel.L4,m_superstructureCommands.preL4());
        coralCommandMap.put(CoralLevel.NONE, Commands.none());
        coralAutoCommandMap.put(CoralLevel.L2, m_superstructureCommands.preL2());
        coralAutoCommandMap.put(CoralLevel.L3, m_superstructureCommands.preL3());
        coralAutoCommandMap.put(CoralLevel.L4, m_superstructureCommands.preL4());
    }


    // A net scoring routine that is based on sensor readings
    public Command netScoringRoutine(){
        return Commands.parallel(
            //Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs), TODO: Integrate LED state into RobotSuperState
            Commands.sequence(
                Commands.parallel(
                    m_superstructureCommands.preAutomaticNet().asProxy(),
                    m_driveCommands.directDriveToNearestPreNetLocation()
                ),
                Commands.parallel(
                    m_driveCommands.directDriveToNearestScoreNetLocation(),
                    m_superstructureCommands.preNet(),
                    Commands.sequence(
                        Commands.waitUntil(() -> {return RobotSuperState.getInstance().getElevatorHeight() > 1.9158291;}), //1.7 //1.9158291
                        m_superstructureCommands.doGrabberAction()
                    )
                )
            )
        );
    }

    /*
    // A net scoring routine that is based on time
    public Command netScoringRoutineTimed(){
        return Commands.sequence(
            m_driveCommands.directDriveToNearestPreNetLocation(),
            Commands.parallel(
                m_superstructureCommands.preNet(),
                Commands.parallel(
                    Commands.sequence(
                        Commands.waitSeconds(0.4),
                        m_driveCommands.directDriveToNearestScoreNetLocation()
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        m_superstructureCommands.doGrabberAction()
                    )
                )
            )
        );
    }*/

    public Command setTargetLevel(CoralLevel level){
        return Commands.runOnce(() -> {
            if(targetLevel == level){
                coralCommandMap.get(targetLevel).schedule();
                targetLevel = CoralLevel.NONE;
            }
            else{
                targetLevel = level;
            }
            Elastic.getInstance().putAutopilotTargetLevel(targetLevel);
        });
    }

    public Command executeAutoCoralCycleLeft(){
        return alignForCoralCycle(true);
    }

    public Command executeAutoCoralCycleRight(){
        return alignForCoralCycle(false);
    }

    private Command followCoralLevelTarget(boolean isL1, boolean leftSide) {
        if(isL1){
            if(leftSide){
                return m_superstructureCommands.preL1DirectLeft();
            }
            else {
                return m_superstructureCommands.preL1DirectRight();
            }
        }
        else{
            return coralAutoCommandMap.get(targetLevel);
        }
    }

    private Command alignForCoralCycle(boolean leftSide){
        return Commands.parallel(
            Commands.either(
                followCoralLevelTarget(true, leftSide), 
                followCoralLevelTarget(false, leftSide), 
                () -> targetLevel == CoralLevel.L1
            ),
            Commands.either(
                Commands.either(
                    m_driveCommands.directDriveToNearestLeftBranch(),
                    m_driveCommands.directDriveToNearestRightBranch(),
                    () -> leftSide
                ),
                m_driveCommands.directDriveToNearestReefFace(),
                () -> targetLevel == CoralLevel.L1
            )
        );
    }
}