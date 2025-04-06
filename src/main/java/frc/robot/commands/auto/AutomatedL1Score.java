// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands.auto;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Localization;
import frc.robot.subsystems.Superstructure;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.Constants.SuperstructureConstants.GrabberState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * If the robot has coral, drive to the L1 position and score. <p>
 * If the robot does not have coral, drive to the coral station, intake the coral, and then drive to the L1 position and score.
 */
public class AutomatedL1Score extends Command {

    private final DriveSubsystem m_drive;
    private final Superstructure superstructure;
    private final Trigger coralPossessionSupplier;
    private Command autoScoreCommand;

    public AutomatedL1Score(DriveSubsystem drive, Superstructure superstructure, Trigger coralPossessionSupplier) {
        this.m_drive = drive;
        this.superstructure = superstructure;
        this.coralPossessionSupplier = coralPossessionSupplier;
        this.autoScoreCommand = Commands.idle();
    }

    @Override
    public void initialize() {

        Pose2d currentPose = m_drive.getPose();
        Pose2d coralStationPose = Localization.getClosestCoralStation(currentPose);
        ReefFace closestReefFace = Localization.getClosestReefFace(currentPose);
        Pose2d scorePose = closestReefFace.getNextL1Position();

        // TODO: add waypoints dependent on the closest reef face so that we don't hit the reef

        autoScoreCommand =
            Commands.sequence(
                // Drive to the coral station while doing grabber intake //TODO: tune coral station poses
                Commands.parallel(
                    m_drive.CommandBuilder.directDriveToPose(coralStationPose),
                    superstructure.getCommandBuilder().autonGrabberIntakeCoral()
                ),
                // Drive to the L1 position while doing going to L1 preset and grabber adjust //TODO: see if grabber adjust can go out more for L1
                Commands.parallel(
                    m_drive.CommandBuilder.directDriveToPose(scorePose),
                    superstructure.getCommandBuilder().autonGrabberAdjustCoral(),
                    superstructure.getCommandBuilder().preL1()
                ),
                // Score the coral
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.L1_OUTTAKE), //TODO: tune voltages (8 and 5 seem low)
                    Commands.waitUntil(() -> !coralPossessionSupplier.getAsBoolean())
                    //Commands.waitSeconds(0.5) //TODO: tune time - important that CANRange doesn't see coral
                )
            ).repeatedly();

        // If the robot already has a coral, score before starting the main command
        if (coralPossessionSupplier.getAsBoolean()) {
            autoScoreCommand = 
                Commands.sequence(
                    Commands.parallel(
                        m_drive.CommandBuilder.directDriveToPose(scorePose),
                        superstructure.getCommandBuilder().preL1()
                    ),
                    Commands.parallel(
                        superstructure.applyGrabberState(GrabberState.L1_OUTTAKE), //TODO: tune voltages (8 and 5 seem low)
                        Commands.waitUntil(() -> !coralPossessionSupplier.getAsBoolean())
                        // Commands.waitSeconds(0.5) //TODO: tune time - important that CANRange doesn't see coral
                    )
                ).andThen(autoScoreCommand);
        }

        autoScoreCommand.schedule();
    }

    @Override
    public void execute(){}
    
    @Override
    public boolean isFinished() {
        return autoScoreCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        autoScoreCommand.cancel();
    }
}