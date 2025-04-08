package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants.ReefLevel;
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
    
    public static Autopilot getInstance() {
        if (inst == null) {
            inst = new Autopilot();
        }
        return inst;
    }
    
    private SuperstructureCommandFactory m_superstructureCommands;
    private DriveCommandFactory m_driveCommands;
    private final Map<ReefLevel,Command> reefCommandMap = new HashMap<>();
    private ReefLevel targetLevel = ReefLevel.L1;
    private Command reefCommand = Commands.none();
    
    private Autopilot(){
        // Constructor is private to enforce singleton pattern
    }

    public void registerDrive(DriveSubsystem drive){
        m_driveCommands = drive.CommandBuilder;
    }

    public void registerSuperstructure(Superstructure superstructure){
        m_superstructureCommands = superstructure.CommandBuilder;
        reefCommandMap.put(ReefLevel.L1,m_superstructureCommands.preL1());
        reefCommandMap.put(ReefLevel.L2,m_superstructureCommands.preL2());
        reefCommandMap.put(ReefLevel.L3,m_superstructureCommands.preL3());
        reefCommandMap.put(ReefLevel.L4,m_superstructureCommands.preL4());
        reefCommandMap.put(ReefLevel.NONE,Commands.none());
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
            Elastic.getInstance().putAutopilotTargetLevel(level);
        });
    }

    public Command executeAutoCoralCycleLeft(){
        return alignForCoralCycle(true);
    }

    public Command executeAutoCoralCycleRight(){
        return alignForCoralCycle(false);
    }

    public void clearStagedCommand() {
        this.targetLevel = ReefLevel.NONE;
    }

    public void cancelReefCommand() {
        reefCommand.cancel();
        targetLevel = ReefLevel.NONE;
    }

    private Command followReefLevelTarget() {
        return new Command() {

            private ReefLevel commandGoalLevel;

            @Override
            public void initialize() {
                commandGoalLevel = targetLevel;
                reefCommand = reefCommandMap.get(commandGoalLevel);
                reefCommand.schedule();
            }

            @Override
            public void execute() {
                if (commandGoalLevel != targetLevel){
                    commandGoalLevel = targetLevel;
                    reefCommand.cancel();
                    reefCommand = reefCommandMap.get(commandGoalLevel);
                    reefCommand.schedule();
                }
            }

            @Override
            public void end(boolean interrupted) {
                targetLevel = ReefLevel.NONE;
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    private Command alignForCoralCycle(boolean leftSide){
        return Commands.deadline(  
            leftSide 
                ? m_driveCommands.directDriveToNearestLeftBranch()
                : m_driveCommands.directDriveToNearestRightBranch(),
            followReefLevelTarget()
        );
    }
}