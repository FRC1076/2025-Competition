// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.utils;

import org.apache.commons.lang3.NotImplementedException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class GeometryUtils {
    private GeometryUtils() {
        throw new NotImplementedException("This is a utility class!");
    }

    /** rotates a Pose2d */
    public static Pose2d rotatePose(Pose2d pose, Rotation2d rot) {
        return new Pose2d(pose.getTranslation(),pose.getRotation().rotateBy(rot));
    }

    /* finds angle from one pose to another pose */
    public static Rotation2d angleToPose(Pose2d startPose, Pose2d endPose){
        return endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    }
}