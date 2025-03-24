// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants;
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

    private static PoseOfInterest getClosestPoseOfInterest(Set<PoseOfInterest> poses,Pose2d robotPose){
        double closestDistance = Double.MAX_VALUE;
        PoseOfInterest closestPose = null;

        for (PoseOfInterest pose : poses) {
            double distance = robotPose.getTranslation().getDistance(pose.pose.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = pose;
            }
        }

        return closestPose;
    }

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest coral station
     */
    public static PoseOfInterest getClosestCoralStation(Pose2d robotPose) {
       return getClosestPoseOfInterest(FieldConstants.coralStations,robotPose);
    }

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest blue coral station
     */
    public static PoseOfInterest getClosestBluCoralStation(Pose2d robotPose) {
       return getClosestPoseOfInterest(FieldConstants.bluCoralStations,robotPose);
    }

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest coral station
     */
    public static PoseOfInterest getClosestRedCoralStation(Pose2d robotPose) {
       return getClosestPoseOfInterest(FieldConstants.redCoralStations,robotPose);
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
