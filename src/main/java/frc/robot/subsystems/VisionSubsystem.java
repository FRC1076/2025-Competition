// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.fieldLayout;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import frc.robot.utils.VirtualSubsystem;
import frc.robot.utils.VisionHelpers;
import lib.functional.TriConsumer;
import lib.vision.PhotonVisionSource;
import lib.vision.VisionLocalizationSystem;

//TODO: Add sim
public class VisionSubsystem extends VirtualSubsystem {
    private final VisionLocalizationSystem m_localizationSystem;

    private final PhotonVisionSource m_frontLeftCamera;
    private final PhotonVisionSource m_frontRightCamera;
    private final PhotonVisionSource m_rearLeftCamera;
    private final PhotonVisionSource m_rearRightCamera;

    //private final VisionSystemSim sim;

    private final Supplier<Pose2d> poseSupplier;

    public VisionSubsystem(Supplier<Pose2d> poseSupplier) {

        m_localizationSystem = new VisionLocalizationSystem();
        this.poseSupplier = poseSupplier;

        m_frontLeftCamera = VisionHelpers.buildPVSourceFromConfig(PhotonConfig.FRONT_LEFT_CAM, () -> poseSupplier.get().getRotation());
        m_frontRightCamera = VisionHelpers.buildPVSourceFromConfig(PhotonConfig.FRONT_RIGHT_CAM, () -> poseSupplier.get().getRotation());
        m_rearLeftCamera = VisionHelpers.buildPVSourceFromConfig(PhotonConfig.REAR_LEFT_CAM, () -> poseSupplier.get().getRotation());
        m_rearRightCamera = VisionHelpers.buildPVSourceFromConfig(PhotonConfig.REAR_RIGHT_CAM, () -> poseSupplier.get().getRotation());

        m_localizationSystem.addSource(m_frontLeftCamera);
        m_localizationSystem.addSource(m_frontRightCamera);
        m_localizationSystem.addSource(m_rearLeftCamera);
        m_localizationSystem.addSource(m_rearRightCamera);

    }

    public VisionSubsystem withMeasurementConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        m_localizationSystem.addMeasurementConsumer(consumer);
        return this;
    }

    public void enableRearCameras(boolean enabled) {
        m_localizationSystem.enableSources(enabled,"REAR_LEFT_CAM","REAR_RIGHT_CAM");
    }

    @Override
    public void periodic() {
        m_localizationSystem.update();
    }
}