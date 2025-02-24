package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import lib.control.HolonomicController;

/**
 * A command that transforms the robot pose bhy the given transformation
 */
public class ApplyTransformCommand extends Command {

    private final Transform2d transform;
    private final DriveSubsystem m_drive;
    private final HolonomicController controller;
    private final double transTolerance;
    private final double rotTolerace;
    private Pose2d targetPose;
    
    public ApplyTransformCommand(Transform2d transform, HolonomicController controller, double transTolerance, double rotTolerace, DriveSubsystem drive) {
        this.transform = transform;
        this.transTolerance = transTolerance;
        this.rotTolerace = rotTolerace;
        this.controller = controller;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        targetPose = m_drive.getPose().transformBy(transform);
    }

    @Override
    public void execute() {
        m_drive.driveFO(controller.calculateFieldOriented(m_drive.getPose(),targetPose));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) <= transTolerance 
            && m_drive.getPose().getRotation().minus(targetPose.getRotation()).getRadians() <= rotTolerace;
    }

}