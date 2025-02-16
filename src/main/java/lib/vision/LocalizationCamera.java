package lib.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A generic interface for handling all camera objects */
public interface LocalizationCamera {

    public static record CommonPoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3,N1> stdDevs
    ) {}

    public abstract Optional<CommonPoseEstimate> getPoseEstimate();
}
