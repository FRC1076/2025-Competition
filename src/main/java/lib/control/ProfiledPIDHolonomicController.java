package lib.control;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfiledPIDHolonomicController {
    private final ProfiledPIDController m_velocityController;
    private final ProfiledPIDController m_thetaController;

    public ProfiledPIDHolonomicController(
        ProfiledPIDController velocityController,
        ProfiledPIDController thetaController
    ) {
        m_velocityController = velocityController;
        m_thetaController = thetaController;
    }

    // Field-oriented
    public ChassisSpeeds calculate(Pose2d current, Pose2d target) {
        var tgtTrans = target.getTranslation();
        var curTrans = current.getTranslation();
        double transError = target.getTranslation().getDistance(current.getTranslation());
        double rotError = target.getRotation().minus(current.getRotation()).getRadians();
        double transOutput = m_velocityController.calculate(transError,0);
        double rotOutput = m_thetaController.calculate(rotError,0);
        Rotation2d directionOfTravel = tgtTrans.minus(curTrans).getAngle();
        ChassisSpeeds outSpeeds = new ChassisSpeeds(
            transOutput * directionOfTravel.getCos(),
            transOutput * directionOfTravel.getSin(),
            rotOutput
        );
        return outSpeeds;
    }
    
}