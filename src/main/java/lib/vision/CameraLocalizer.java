// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A generic interface for handling all camera objects */
public interface CameraLocalizer {

    public static record CommonPoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        List<Integer> fiducialIDs,
        Matrix<N3, N1> stdDevs
    ) {}

    public abstract Optional<CommonPoseEstimate> getPoseEstimate();

    public abstract String getName();

    public default boolean isConnected() {
        return true;
    }

    public default void addHeadingSupplier(Supplier<Rotation2d> heading) {}

    public default void setPoseStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}

    public default void setFallbackPoseStrategy(PhotonPoseEstimator.PoseStrategy strategy) {}
}
