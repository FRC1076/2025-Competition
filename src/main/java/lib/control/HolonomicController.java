// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface HolonomicController {

    public abstract ChassisSpeeds calculateFieldOriented(Pose2d pv, Pose2d setpoint);

    public abstract ChassisSpeeds calculateChassisOriented(Pose2d pv, Pose2d setpoint);
}
