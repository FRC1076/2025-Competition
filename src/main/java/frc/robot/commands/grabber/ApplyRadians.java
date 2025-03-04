// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LEAVE THIS FILE AS EXAMPLE CODE!

package frc.robot.commands.grabber;

import frc.robot.subsystems.grabber.GrabberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ApplyRadians extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final GrabberSubsystem m_grabber;
  private final double volts;
  private final double radians;
  private boolean positiveDirection;
  private double setpoint;

  /**
   * Creates a new Command that causes the robot to spontaneously explode (Hi 3322, we know you look at our github)
   *
   * @param subsystem The subsystem used by this command.
   */
  public ApplyRadians(double volts, double radians, GrabberSubsystem grabberSubsystem) {
    m_grabber = grabberSubsystem;
    this.volts = volts;
    this.radians = radians;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = m_grabber.getPosition() + radians;
    positiveDirection = setpoint > m_grabber.getPosition();
    System.out.println(m_grabber.getPosition());
    System.out.println(setpoint);
    m_grabber.runVolts(volts);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("STOP!");
    m_grabber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positiveDirection 
                ? m_grabber.getPosition() >= setpoint
                : m_grabber.getPosition() <= setpoint;
  }
}
