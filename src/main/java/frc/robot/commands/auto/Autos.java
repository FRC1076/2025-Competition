package frc.robot.commands.auto;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {

    private final DriveSubsystem m_drive;
    private final Superstructure m_superstructure;

    public Autos(DriveSubsystem drive, Superstructure superstructure){
        m_drive = drive;
        m_superstructure = superstructure;
    }
}
