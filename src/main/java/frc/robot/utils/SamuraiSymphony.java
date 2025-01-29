package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;


public class SamuraiSymphony extends Orchestra {
    

    public void addDrivetrain(SwerveDrivetrain<? extends TalonFX, ? extends TalonFX, ?> drive) {
        for (SwerveModule<? extends TalonFX,? extends TalonFX,?> module : drive.getModules()) {
            super.addInstrument(module.getDriveMotor());
            super.addInstrument(module.getSteerMotor());
        }
    }
}
