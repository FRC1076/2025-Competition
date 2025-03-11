package lib.control;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfiledPIDHolonomicController {
    private final ProfiledPIDController m_translationController;
    private final ProfiledPIDController m_thetaController;

    public ProfiledPIDHolonomicController(
        ProfiledPIDController translationController,
        ProfiledPIDController thetaController
    ) {
        m_translationController = translationController;
        m_thetaController = thetaController;
    }

    // Field-oriented
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        Translation2d targetTranslation = targetPose.getTranslation();
        Translation2d currentTranslation = currentPose.getTranslation();
        
        double translationError = targetPose.getTranslation().getDistance(currentPose.getTranslation());
        double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

        double translationOutput = m_translationController.calculate(translationError, 0);
        double rotationOutput = m_thetaController.calculate(rotationError, 0);
        Rotation2d directionOfTravel = targetTranslation.minus(currentTranslation).getAngle();

        ChassisSpeeds outSpeeds = new ChassisSpeeds(
            translationOutput * directionOfTravel.getCos(),
            translationOutput * directionOfTravel.getSin(),
            rotationOutput
        );
        return outSpeeds;
    }

    public boolean atReference() {
        return m_translationController.atGoal() && m_thetaController.atGoal();
    }
    
}