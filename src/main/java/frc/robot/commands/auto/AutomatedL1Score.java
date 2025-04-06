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

    private Command intakeCommand;
    private Command scoreCommand;
    private Command autoL1Command;

    public AutomatedL1Score(DriveSubsystem drive, Superstructure superstructure, Trigger coralPossessionSupplier) {
        this.m_drive = drive;
        this.superstructure = superstructure;
        this.coralPossessionSupplier = coralPossessionSupplier;

        this.intakeCommand = Commands.idle();
        this.scoreCommand = Commands.idle(); 
        this.autoL1Command = Commands.idle();
    }

    @Override
    public void initialize() {

        Pose2d currentPose = m_drive.getPose();
        Pose2d coralStationPose = Localization.getClosestCoralStation(currentPose);
        ReefFace closestReefFace = Localization.getClosestReefFace(currentPose);
        Pose2d scorePose = closestReefFace.getNextL1Position();

        // TODO: add waypoints dependent on the closest reef face so that we don't hit the reef    

        // If the robot already has a coral, drive to the coral station until grabber intake is finished //TODO: tune coral station poses
        if (coralPossessionSupplier.getAsBoolean()) {
            autoL1Command = 
                Commands.sequence(
                    Commands.parallel(
                        m_drive.CommandBuilder.directDriveToPose(scorePose),
                        // superstructure.getCommandBuilder().autonGrabberAdjustCoral(), //maybe not necessary
                        superstructure.getCommandBuilder().preL1()
                    ),
                    // Score the coral
                    Commands.parallel(
                        superstructure.applyGrabberState(GrabberState.L1_OUTTAKE), //TODO: tune voltages (8 and 5 seem low)
                        Commands.waitUntil(() -> !coralPossessionSupplier.getAsBoolean()),
                        Commands.runOnce(() -> closestReefFace.increaseL1Index()) // The coral is scored, so next time score the next L1
                        //Commands.waitSeconds(0.5) //TODO: tune time - important that CANRange doesn't see coral
                    )
                );
        } else {
            // If the robot does not have a coral, drive to the coral station and intake the coral
            autoL1Command = 
                Commands.deadline(
                    superstructure.getCommandBuilder().autonGrabberIntakeCoral(),
                    m_drive.CommandBuilder.directDriveToPose(coralStationPose)
                );
        }

        autoL1Command.schedule();
    }

    @Override
    public void execute(){}
    
    @Override
    public boolean isFinished() {
        return autoL1Command.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        autoL1Command.cancel();
    }
}