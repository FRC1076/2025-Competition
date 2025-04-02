package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.RobotSuperState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;

import java.util.Map;
import java.util.Set;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A command factory that executes autonomous teleop routines requiring multiple subsystems
 */
public class Autopilot {
    private final SuperstructureCommandFactory m_superstructureCommands;
    private final DriveCommandFactory m_driveCommands;
    private final DriveSubsystem m_drive;
    private final Superstructure m_superstructure;
    private final Map<ReefLevel,Supplier<Command>> reefCommandMap = new HashMap<>();
    private ReefLevel targetLevel = ReefLevel.L1;
    private ReefLevel commandGoalLevel = ReefLevel.L1;
    private Command reefCommand = Commands.none();
    
    
    public Autopilot(DriveSubsystem drive, Superstructure superstructure){
        m_driveCommands = drive.CommandBuilder;
        m_superstructureCommands = superstructure.CommandBuilder;
        m_drive = drive;
        m_superstructure = superstructure;
        reefCommandMap.put(ReefLevel.L1,m_superstructureCommands::preL1);
        reefCommandMap.put(ReefLevel.L2,m_superstructureCommands::preL2);
        reefCommandMap.put(ReefLevel.L3,m_superstructureCommands::preL3);
        reefCommandMap.put(ReefLevel.L4,m_superstructureCommands::preL4);
    }

    public Command coralIntakeRoutine(){
        return Commands.sequence(
            m_driveCommands.directDriveToNearestCoralStation(),
            m_superstructureCommands.intakeCoral()
        );
    }

    // A net scoring routine that is based on sensor readings
    public Command netScoringRoutine(){
        return Commands.sequence(
            m_driveCommands.directDriveToNearestPreNetLocation(),
            m_superstructureCommands.preNet(),
            Commands.parallel(
                m_superstructureCommands.scoreNet(),
                m_driveCommands.directDriveToNearestScoreNetLocation()
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
            System.out.println("LOADED TARGET REEF LEVEL: " + targetLevel.name());
        });
    }

    public Command executeAutoCoralCycleLeft(){
        return Commands.defer(() -> alignForCoralCycle(true, targetLevel), Set.of(m_drive, m_superstructure));
    }

    public Command executeAutoCoralCycleRight(){
        return Commands.defer(() -> alignForCoralCycle(false, targetLevel), Set.of(m_drive, m_superstructure));
    }

    private Command followReefLevelTarget() {
        return new FunctionalCommand(
            () -> {
                commandGoalLevel = targetLevel;
                reefCommand = reefCommandMap.get(commandGoalLevel).get();
                reefCommand.schedule();
            }, 
            () -> {
                if (commandGoalLevel != targetLevel){
                    commandGoalLevel = targetLevel;
                    reefCommand.cancel();
                    reefCommand = reefCommandMap.get(commandGoalLevel).get();
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
            leftSide ? m_driveCommands.directDriveToNearestLeftBranch() : m_driveCommands.directDriveToNearestRightBranch(),
            Commands.run(() -> {})
        );
    }
}
