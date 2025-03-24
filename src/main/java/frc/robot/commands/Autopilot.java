package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;

/**
 * A command factory that executes autonomous teleop routines requiring multiple subsystems
 */
public class Autopilot {
    private final SuperstructureCommandFactory m_superstructureCommands;
    private final DriveCommandFactory m_driveCommands;
    
    public Autopilot(DriveSubsystem drive, Superstructure superstructure){
        m_driveCommands = drive.CommandBuilder;
        m_superstructureCommands = superstructure.CommandBuilder;
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
}
