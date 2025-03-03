// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import static frc.robot.Constants.VisionConstants.fieldLayout;
import static frc.robot.Constants.VisionConstants.Photonvision.kCpnpParams;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import lib.vision.PhotonVisionSource;

public final class VisionHelpers {

    private VisionHelpers() {}

    public static final PhotonVisionSource buildPVSourceFromConfig(PhotonConfig config,Supplier<Rotation2d> headingSupplier) {
        var cam = new PhotonCamera(config.name);
        return new PhotonVisionSource(
            cam,
            config.offset,
            config.multiTagPoseStrategy, 
            config.singleTagPoseStrategy, 
            fieldLayout,
            config.defaultSingleTagStdDevs,
            config.defaultMultiTagStdDevs, 
            headingSupplier,
            config.cameraMatrix,
            config.distCoeffs,
            kCpnpParams
        );
    }
}
