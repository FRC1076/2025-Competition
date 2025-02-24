// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants.PoseOfInterest;
import frc.robot.Constants.FieldConstants.ReefFace;

/**
 * A class containing utility functions for localization
 * 
 * @author Tejas Gupta
 * @author David Loh
 * @author Jesse Kane
 */
public final class Localization {

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest Reef Face
     */
    
    public static ReefFace getClosestReefFace(Pose2d robotPose){
        double closestDistance = Double.MAX_VALUE; // Distance away from april tag
        ReefFace closestFace = null;

        for (ReefFace face: ReefFace.values()){
            double distance = robotPose.getTranslation().getDistance(face.AprilTag.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = face;
            }
        }

        return closestFace;
    }

    /** 
     * Returns the pose of the nearest coral station 
     * 
     * @author Jesse Kane
     */
    public static Pose2d getClosestCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getCoralStationPoses());
    }

    public static Pose2d getClosestBlueCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getBlueCoralStationPoses());
    }

    public static Pose2d getClosestRedCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getRedCoralStationPoses());
    }

    /** Returns a list of all coral stations */
    public static List<Pose2d> getCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_LEFT_STATION.pose);
        poseList.add(PoseOfInterest.BLU_RIGHT_STATION.pose);
        poseList.add(PoseOfInterest.RED_RIGHT_STATION.pose);
        poseList.add(PoseOfInterest.RED_LEFT_STATION.pose);
        return poseList;
    }
    /** Returns a list of red coral stations */
    public static List<Pose2d> getRedCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.RED_RIGHT_STATION.pose);
        poseList.add(PoseOfInterest.RED_LEFT_STATION.pose);
        return poseList;
    }

    /** Returns a list of blue coral stations */
    public static List<Pose2d> getBlueCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_LEFT_STATION.pose);
        poseList.add(PoseOfInterest.BLU_RIGHT_STATION.pose);
        return poseList;
    }

    private static ReefFace getReefFromAprilTagID(int aprilTagID){
        for (ReefFace face: ReefFace.values()) {
            if (face.aprilTagID == aprilTagID) {
                return face;
            }
        }
        return null;
    }
}
