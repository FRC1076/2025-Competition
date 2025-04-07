package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.RobotSuperState;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;

import java.util.Map;
import java.util.Set;
import java.util.HashMap;
import java.util.function.Supplier;

/**
 * A command factory that executes autonomous teleop routines requiring multiple subsystems
 */
public class Autopilot {
    
    private final SuperstructureCommandFactory m_superstructureCommands;
    private final DriveCommandFactory m_driveCommands;
    private final DriveSubsystem m_drive;
    private final Superstructure m_superstructure;
    private final Map<ReefLevel,Command> reefCommandMap = new HashMap<>();
    private ReefLevel targetLevel = ReefLevel.L1;
    private ReefLevel commandGoalLevel = ReefLevel.L1;
    private Command reefCommand = Commands.none();
    
    public Autopilot(DriveSubsystem drive, Superstructure superstructure){
        m_driveCommands = drive.CommandBuilder;
        m_superstructureCommands = superstructure.CommandBuilder;
        m_drive = drive;
        m_superstructure = superstructure;
        reefCommandMap.put(ReefLevel.L1,m_superstructureCommands.preL1());
        reefCommandMap.put(ReefLevel.L2,m_superstructureCommands.preL2());
        reefCommandMap.put(ReefLevel.L3,m_superstructureCommands.preL3());
        reefCommandMap.put(ReefLevel.L4,m_superstructureCommands.preL4());
    }


    // A net scoring routine that is based on sensor readings
    public Command netScoringRoutine(){
        return Commands.parallel(
            //Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs), TODO: Integrate LED state into RobotSuperState
            Commands.sequence(
                Commands.parallel(
                    m_superstructure.CommandBuilder.preAutomaticNet().asProxy(),
                    m_drive.CommandBuilder.directDriveToNearestPreNetLocation()
                ),
                Commands.parallel(
                    m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                    m_superstructure.CommandBuilder.preNet(),
                    Commands.sequence(
                        Commands.waitUntil(() -> {return m_superstructure.getElevator().getPositionMeters() > 1.9158291;}), //1.7 //1.9158291
                        m_superstructure.CommandBuilder.doGrabberAction()
                    )
                )
            )
        );
    }

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
    }

    public Command setTargetLevel(ReefLevel level){
        return Commands.runOnce(() -> {
            targetLevel = level;
            // TODO: Elastic.getInstance().putAutopilotTargetLevel(level);
        });
    }

    public Command executeAutoCoralCycleLeft(){
        return Commands.defer(() -> alignForCoralCycle(true, targetLevel), Set.of(m_drive, m_superstructure.getElevator(), m_superstructure.getWrist()));
    }

    public Command executeAutoCoralCycleRight(){
        return Commands.defer(() -> alignForCoralCycle(false, targetLevel), Set.of(m_drive, m_superstructure.getElevator(), m_superstructure.getWrist()));
    }

    private Command followReefLevelTarget() {
        return new FunctionalCommand(
            () -> {
                commandGoalLevel = targetLevel;
                reefCommand = reefCommandMap.get(commandGoalLevel);
                reefCommand.schedule();
            }, 
            () -> {
                if (commandGoalLevel != targetLevel){
                    commandGoalLevel = targetLevel;
                    reefCommand.cancel();
                    reefCommand = reefCommandMap.get(commandGoalLevel);
                    reefCommand.schedule();
                }
            }, 
            (interrupted) -> reefCommand.cancel(), 
            () -> RobotSuperState.getInstance().getWristevatorState() == RobotSuperState.getInstance().getWristevatorGoal()
        );
    }

    private Command alignForCoralCycle(boolean leftSide,ReefLevel level){
        return Commands.parallel(
            followReefLevelTarget(),
            leftSide ? m_driveCommands.directDriveToNearestLeftBranch() : m_driveCommands.directDriveToNearestRightBranch()
        );
    }
}