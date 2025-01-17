// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Constants.Coordinates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** An example command that uses an example subsystem. */
public class DriveToNearestBranch extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final Boolean isLeftBranch;
  private Command pathfindCommand;
  private Pose2d nearestApril;
  private Pose2d nearestBranch;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToNearestBranch(DriveSubsystem subsystem, Boolean isLeftBranch) {
    m_subsystem = subsystem;
    this.isLeftBranch = isLeftBranch;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nearestApril = m_subsystem.getPose().nearest(Coordinates.reefAprilCoordinates);
    if(isLeftBranch){
      nearestBranch = nearestApril.plus(new Transform2d(0.4572, -0.161163, Rotation2d.fromDegrees(0)));
    }
    else{
      nearestBranch = nearestApril.plus(new Transform2d(0.4572, 0.161163, Rotation2d.fromDegrees(0)));
    }
    m_subsystem.getPathfindToPoseCommand(nearestBranch).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
