// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands.auto;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.Localization;
import lib.utils.GeometryUtils;
import frc.robot.subsystems.Superstructure;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.commands.drive.PPDriveToPose;

import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotOffset;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    private final PathConstraints constraints;
    private final PPDriveToPose driveCommand;

    private Command autoL1Command;

    public AutomatedL1Score(DriveSubsystem drive, Superstructure superstructure, Trigger coralPossessionSupplier) {
        this.m_drive = drive;
        this.superstructure = superstructure;
        this.coralPossessionSupplier = coralPossessionSupplier;
        this.constraints = new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(540));
        this.driveCommand = new PPDriveToPose(m_drive, new Pose2d(), constraints,0.0);

        this.autoL1Command = Commands.idle();
    }

    @Override
    public void initialize() {

        Pose2d currentPose = m_drive.getPose();
        Pose2d coralStationPose = GeometryUtils.rotatePose(Localization.getClosestCoralStation(currentPose), Rotation2d.k180deg);
        ReefFace closestReefFace = Localization.getClosestReefFace(currentPose);
        Pose2d scorePose = GeometryUtils.rotatePose(closestReefFace.getNextL1Position().transformBy(robotOffset), Rotation2d.k180deg);
        WristevatorState scoreState = closestReefFace.getNextL1WristevatorState();

        // TODO: add waypoints dependent on the closest reef face so that we don't hit the reef    

        // If the robot already has a coral, drive to the coral station until grabber intake is finished // TODO: tune coral station poses
        if (coralPossessionSupplier.getAsBoolean()) {
            driveCommand.setTargetPose(scorePose);

            autoL1Command = 
                Commands.sequence(
                    // Drive to the reef
                    Commands.parallel(
                        driveCommand,
                        // superstructure.getCommandBuilder().autonGrabberAdjustCoral(), //maybe not necessary
                        superstructure.applyWristevatorState(scoreState)
                    ),
                    // Score the coral
                    //Commands.parallel(
                        superstructure.applyGrabberState(GrabberState.L1_OUTTAKE), // TODO: tune voltages (8 and 5 seem low)
                        Commands.waitUntil(coralPossessionSupplier.negate()),
                        Commands.runOnce(() -> closestReefFace.increaseL1Index()) // The coral is scored, so next time score the next L1
                        //superstructure.applyGrabberState(GrabberState.IDLE);
                );
        } /*else if (closestReefFace == ReefFace.BLU_REEF_GH || closestReefFace == ReefFace.RED_REEF_GH) {
            // If the robot is too far from the coral station and in danger of hitting the reef, do nothing
            autoL1Command = Commands.idle();
        } */else {
            // If the robot does not have a coral, drive to the coral station and intake the coral
            driveCommand.setTargetPose(coralStationPose);

            autoL1Command = 
                Commands.deadline(
                    Commands.sequence(
                        Commands.print("intake started"),
                        superstructure.getCommandBuilder().autonGrabberIntakeCoral(),
                        superstructure.applyGrabberState(GrabberState.IDLE)
                    ),
                    driveCommand
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