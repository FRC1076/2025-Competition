package lib.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import lib.vision.CameraLocalizer.CommonPoseEstimate;

public class LoggedPhotonVisionLocalizer implements CameraLocalizer {
    
    public static class PhotonVisionInputs {
        public boolean cameraConnected = false;
        public boolean estimatePresent = false;
        public int tagsDetected = 0;
        public Integer[] fiducialIDs = new Integer[]{};
        public Pose3d pose = new Pose3d();
        public Matrix<N3,N1> stddevs = VecBuilder.fill(0, 0, 0);
        public String strategy = "NULL";

        public void log(String key) {
            Logger.recordOutput(key, cameraConnected);
            Logger.recordOutput(key, estimatePresent);
            Logger.recordOutput(key, tagsDetected);
            Logger.recordOutput(key, fiducialIDs.toString());
            Logger.recordOutput(key, pose);
            Logger.recordOutput(key, stddevs);
            Logger.recordOutput(key, strategy);
        }
    }
        

    private static final Matrix<N3, N1> maxStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Supplier<Rotation2d> headingSupplier;
    private final Matrix<N3, N1> defaultSingleStdDevs;
    private final Matrix<N3, N1> defaultMultiStdDevs;
    private final PhotonVisionInputs inputs = new PhotonVisionInputs();

    public LoggedPhotonVisionLocalizer(
        PhotonCamera camera, 
        Transform3d offset,
        PhotonPoseEstimator.PoseStrategy primaryStrategy,
        PhotonPoseEstimator.PoseStrategy multiTagFallbackStrategy,
        Supplier<Rotation2d> headingSupplier,
        AprilTagFieldLayout fieldLayout,
        Matrix<N3, N1> defaultSingleStdDevs,
        Matrix<N3, N1> defaultMultiStdDevs
    ) {
        this.camera = camera;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, primaryStrategy, offset);
        poseEstimator.setMultiTagFallbackStrategy(multiTagFallbackStrategy);
        this.headingSupplier = headingSupplier;
        this.defaultSingleStdDevs = defaultSingleStdDevs;
        this.defaultMultiStdDevs = defaultMultiStdDevs;
    }

    /**
     * Calculates the standard deviations for the pose estimate based on how many tags are visible and how far they are
     */
    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est) {
        var stdDevs = defaultSingleStdDevs;
        int numTargets = 0;
        double avgDist = 0;
        var targets = est.targetsUsed;
        for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {continue;}
            numTargets++;
            avgDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }
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

    /**
     * Gets the pose estimate from the camera
     * @return The pose estimate, or Optional.empty() if no estimate is available
     */
    public Optional<CommonPoseEstimate> getPoseEstimate() {

        inputs.cameraConnected = camera.isConnected();
        inputs.estimatePresent = false;
        inputs.tagsDetected = 0;

        poseEstimator.addHeadingData(Timer.getFPGATimestamp(), headingSupplier.get());
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        for (var res : results) {
            visionEst = poseEstimator.update(res);
        }

        Optional<CommonPoseEstimate> result = visionEst.map(
            (EstimatedRobotPose estimate) -> {
                var stddevs = calculateStdDevs(estimate);
                inputs.tagsDetected = estimate.targetsUsed.size();
                inputs.fiducialIDs = estimate.targetsUsed.stream().map((tgt) -> tgt.getFiducialId()).toArray((size) -> new Integer[size]);
                inputs.pose = estimate.estimatedPose;
                inputs.stddevs = stddevs;
                inputs.strategy = estimate.strategy.name();
                return new CommonPoseEstimate(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds,
                    stddevs
                );
            }
        );

        // inputs.log("PhotonVision/" + getName());

        return result;
    }

    public String getName() {
        return camera.getName();
    }
}
