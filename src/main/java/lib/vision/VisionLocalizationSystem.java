// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.vision;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.functional.TriConsumer;
import lib.vision.CameraSource.CommonPoseEstimate;


/**
 * A class that manages robot position estimates from multiple cameras and 
 * sends them to all externally registered consumers using the {@link VisionLocalizationSystem#update()} method
 * <p>Consumers are registered using the {@link VisionLocalizationSystem#addMeasurementConsumer()} method
 */
public class VisionLocalizationSystem {

    // A callback to a pose estimate supplier from a CameraSource
    private class SourceCallback {
        public final Supplier<Optional<CommonPoseEstimate>> supplier;
        public boolean enabled;

        public SourceCallback(Supplier<Optional<CommonPoseEstimate>> supplier) {
            this.supplier = supplier;
            enabled = true;
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private TriConsumer<Pose2d,Double,Matrix<N3,N1>> measurementConsumer = (pose,timestamp,stddevs) -> {}; //A default no-op consumer is instantiated to prevent null pointer dereferences

    private final Map<String,SourceCallback> sourceCallbacks = new HashMap<>();
    
    /**
     * Adds a consumer
     * <p> This method should be called externally on initialization
     * 
     * @param consumer A consumer that accepts a Pose2d, Double, and 3x1 Matrix
     * <p> The Pose2d is the robot pose estimate in meters
     * <p> The Double is the timestamp of the measurement in seconds
     * <p> The 3x1 Matrix is the standard deviations of the measurement
     */
    public void addMeasurementConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        measurementConsumer = measurementConsumer.andThen(consumer);
    }

    /**
     * Adds a source to the vision system
     */
    public void addSource(CameraSource source) {
        sourceCallbacks.put(source.getName(),new SourceCallback(source::getPoseEstimate));
    }

    /**
     * Enables or disables selected sources
     * @param enabled Whether or not the sources should be enabled
     * @param sources The IDs of the sources to enable or disable
     */
    public void enableSources(boolean enabled, String... sources) {
        for (String sourceID : sources) {
            sourceCallbacks.get(sourceID).enabled = enabled;
        }
    }

    /**
     * Enables or disables all cameras
     * @param enabled Whether or not the cameras should be enabled
     */
    public void enableAllSources(boolean enabled) {
        for (var source : sourceCallbacks.values()) {
            source.enabled = true;
        }
    }

    /**
     * Fetches pose estimates from sources and sends them to all consumers. This function should be called exactly once every main function loop
     */
    public void update() {
        for (var source : sourceCallbacks.values()) {
            if (source.enabled) {
                source.supplier.get().ifPresent(
                    (estimate) -> {
                        measurementConsumer.accept(
                            estimate.pose(),
                            estimate.timestampSeconds(),
                            estimate.stdDevs()
                        );
                    }
                );
            }
        }
    }
}