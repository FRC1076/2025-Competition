// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands.drive;

import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import frc.robot.RobotSuperState;
import lib.utils.GeometryUtils;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

/** This automatically drives to a pose without using A* to generate a trajectory, 
  * useful for when we know there are no obstructions on the field between the robot 
  * and the desired pose */
public class AutopilotDriveToPose extends Command {

    private Command followPathCommand = Commands.none();
    private Pose2d targetPose;
    private double endVelocity;

    public AutopilotDriveToPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.endVelocity = 0;
    }

    public AutopilotDriveToPose(Pose2d targetPose, double endVelocity) {
        this.targetPose = targetPose;
        this.endVelocity = endVelocity;
    }

    @Override
    public void initialize() {
        generatePath();
        followPathCommand.schedule();
    }

    public BooleanSupplier atGoal() {
        return () -> targetPose.getTranslation().getDistance(RobotSuperState.getInstance().getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;
    }

    @Override
    public void execute(){
        //System.out.println(followPathCommand.isScheduled());
    }
    
    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        followPathCommand.cancel();
    }

    private void generatePath() {
        Pose2d currentPose = RobotSuperState.getInstance().getPose();
        Pose2d startingWaypoint = new Pose2d(currentPose.getTranslation(), GeometryUtils.angleToPose(currentPose, targetPose));
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startingWaypoint,
            targetPose
        );

        // Prevent PathPlanner from treating the start pose as identical to the end pose when they are too close to each other
        if (targetPose.getTranslation().getDistance(startingWaypoint.getTranslation()) > PathPlannerConstants.pathGenerationToleranceMeters){
            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                PathPlannerConstants.pathConstraints, 
                //new IdealStartingState(m_drive.getVelocityMPS(), m_drive.getHeading()), 
                null,
                new GoalEndState(endVelocity, targetPose.getRotation())
            );
            path.preventFlipping = true;
            followPathCommand = AutoBuilder.followPath(path);
        } else {
            followPathCommand = Commands.none();
        }
    }

    public void setTargetPose(Pose2d targetPose){
        followPathCommand.cancel();
        this.targetPose = targetPose;
        if (this.isScheduled()){
            generatePath();
            followPathCommand.schedule();
        }
    }

    public void setEndVelocity(double endVelocity){
        followPathCommand.cancel();
        this.endVelocity = endVelocity;
        if (this.isScheduled()){
            generatePath();
            followPathCommand.schedule();
        }
    }
}