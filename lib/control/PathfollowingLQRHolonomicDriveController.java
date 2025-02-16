// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.control;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Provides an LQR holonomic controller in the interface shape of a PathFollowingController, for compatibility with pathplanner */
public class PathfollowingLQRHolonomicDriveController extends LQRHolonomicDriveController implements PathFollowingController {

    public PathfollowingLQRHolonomicDriveController(LQRHolonomicDriveControllerTolerances tolerances, double dt) {
        super(tolerances,dt);
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        super.resetController();
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        double xFF = targetState.fieldSpeeds.vxMetersPerSecond;
        double yFF = targetState.fieldSpeeds.vyMetersPerSecond;
        var LQRFeedbacks = super.calculateRaw(currentPose,targetState.pose);
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            LQRFeedbacks.get(0,0) + xFF, 
            LQRFeedbacks.get(1,0) + yFF,
            LQRFeedbacks.get(2,0),
            currentPose.getRotation()
        );

    }

}