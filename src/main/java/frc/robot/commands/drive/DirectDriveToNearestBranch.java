// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Constants.Coordinates;
import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.fasterxml.jackson.core.TreeNode;
import com.pathplanner.lib.path.GoalEndState;


/** An example command that uses an example subsystem. */
public class DirectDriveToNearestBranch extends WrapperCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final Boolean isLeftBranch;
  private Pose2d nearestBranch;
  private Command pathfindCommand;
  private final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0));

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DirectDriveToNearestBranch(DriveSubsystem subsystem, Boolean isLeftBranch) {
    super(Commands.none());
    m_subsystem = subsystem;
    this.isLeftBranch = isLeftBranch;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isLeftBranch){
      nearestBranch = m_subsystem.getPose().nearest(Coordinates.leftBranchCoordinates).plus(robotOffset);
    }
    else{
      nearestBranch = m_subsystem.getPose().nearest(Coordinates.rightBranchCoordinates).plus(robotOffset);
    }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(m_subsystem.getPose().getTranslation(), nearestBranch.getTranslation().minus(m_subsystem.getPose().getTranslation()).getAngle()),
        new Pose2d(nearestBranch.getTranslation(), nearestBranch.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
    );
    PathPlannerPath path = new PathPlannerPath(waypoints, PathPlannerConstants.pathConstraints, null, new GoalEndState(0, nearestBranch.getRotation()));
    path.preventFlipping = true;
    pathfindCommand = m_subsystem.getFollowPathCommand(path);
    pathfindCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
