// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

 package lib.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.vision.Limelight.LLPoseEstimate;

public class LimelightLocalizer implements CameraLocalizer {
    private static final Matrix<N3, N1> maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private final Limelight camera;
    private final Transform3d camToBotOffset;
    private final Matrix<N3, N1> defaultSingleStdDevs;
    private final Matrix<N3, N1> defaultMultiStdDevs;

    public LimelightLocalizer(
        Limelight camera,
        Transform3d offset,
        Matrix<N3, N1> defaultSingleStdDevs,
        Matrix<N3, N1> defaultMultiStdDevs
    ) {
        this.camera = camera;
        this.camToBotOffset = offset.inverse();
        this.defaultSingleStdDevs = defaultSingleStdDevs;
        this.defaultMultiStdDevs = defaultMultiStdDevs;
    }

     /**
      * Gets the pose estimate from the camera
      * @return The pose estimate, or Optional.empty() if no estimate is available
      */
    public Optional<CommonPoseEstimate> getPoseEstimate() {
        return camera.getPoseEstimateMT2().map(
            (poseEstimate) -> new CommonPoseEstimate(
                poseEstimate.pose().transformBy(camToBotOffset).toPose2d(),
                poseEstimate.timestampSeconds(),
                calculateStdDevs(poseEstimate)
            )
        );
    }

    public String getName() {
        return camera.getName();
    }

    private Matrix<N3, N1> calculateStdDevs(LLPoseEstimate estimate) {
        var stdDevs = defaultSingleStdDevs.copy();
        int numTargets = estimate.tagCount();
        double avgDist = estimate.avgTagDist();
        if (numTargets == 0) {
            return maxStdDevs; //No targets detected, resort to maximum std devs
        }

        // One or more tags visible, run the full heuristic.

        // Decrease std devs if multiple targets are visible
        avgDist /= numTargets;
        if (numTargets > 1) {
            stdDevs = defaultMultiStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTargets == 1 && avgDist > 4){
            //Distance greater than 4 meters, and only one tag detected, resort to maximum std devs
            stdDevs = maxStdDevs;
        } else {
            stdDevs = stdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return stdDevs;
    }
}
