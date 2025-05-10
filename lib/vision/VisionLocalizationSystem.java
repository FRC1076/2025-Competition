// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.function.BooleanConsumer;
import lib.functional.TriConsumer;
import lib.vision.CameraLocalizer.CommonPoseEstimate;



public class VisionLocalizationSystem {

    private class CamStruct {
        public final CameraLocalizer camera;
        public boolean cameraActive; //Signals whether or not the camera reading should be added to vision measurements

        // sets whether or not the camera is active
        public void setActive(boolean active) {
            cameraActive = active;
        }

        public CamStruct(CameraLocalizer camera) {
            this.camera = camera;
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private TriConsumer<Pose2d,Double,Matrix<N3,N1>> measurementConsumer;
    private final List<CamStruct> cameras = new ArrayList<>();
    
    public void registerMeasurementConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        if (measurementConsumer == null) {
            //Initialize measurementConsumer if none exists
            measurementConsumer = consumer;
        } else {
            //If measurementConsumer already exists, then compose it with the new measurementConsumer
            measurementConsumer = measurementConsumer.andThen(consumer);
        }
    }

    /**
     * Adds a camera to the ApriltagLocalizer
     * 
     * @return
     * A BooleanConsumer that can set whether or not the ApriltagLocalizer will consider the measurements from this camera in its pose estimates
     */
    public BooleanConsumer addCamera(CameraLocalizer camera) {
        CamStruct camStruct = new CamStruct(camera);
        cameras.add(camStruct);
        return camStruct::setActive;
    }

    /**
     * Fetches pose estimate from cameras and sends them to all consumers. This function should be called exactly once every main function loop
     */
    public void update() {
        for (var camStruct : cameras) {
            if (camStruct.cameraActive) {
                camStruct.camera.getPoseEstimate().ifPresent(
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
