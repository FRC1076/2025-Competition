package lib.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

public class Limelight {

    public static record LLFiducial(
        double id,
        double txnc,
        double tync,
        double ta,
        double distToCamera,
        double distToRobot,
        double ambiguity
    ) {}

    public static record LLPoseEstimate(
        Pose3d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        List<LLFiducial> fiducials,
        boolean isMegaTag2
    ) {}

    private final String NTName;
    private final NetworkTable LLNetworkTable;
    
    public Limelight(String name) {
        this(NetworkTableInstance.getDefault(),name);
    }

    public Limelight(NetworkTableInstance server, String name) {
        NTName = name;
        LLNetworkTable = server.getTable(name);
    }

    public boolean hasValidTarget() {
        return LLNetworkTable.getEntry("tv").getInteger(0) == 1;
    }

    public List<LLFiducial> getFiducials() {
        List<LLFiducial> fiducials = new ArrayList<>();
        double[] rawFiducialsEntry = LLNetworkTable.getEntry("rawfiducials").getDoubleArray(new double[]{});
        if (rawFiducialsEntry.length == 0) {
            return fiducials; //Returns empty array if no valid fiducials are detected
        }
        double id = 0;
        double txnc = 0;
        double tync = 0;
        double ta = 0;
        double distToCamera = 0;
        double distToRobot = 0;
        double ambiguity = 0;
        for (int i = 0; i < rawFiducialsEntry.length; i += 7) {
            id = rawFiducialsEntry[i];
            txnc = rawFiducialsEntry[i+1];
            tync = rawFiducialsEntry[i+2];
            ta = rawFiducialsEntry[i+3];
            distToCamera = rawFiducialsEntry[i+4];
            distToRobot = rawFiducialsEntry[i+5];
            ambiguity = rawFiducialsEntry[i+6];
            fiducials.add(new LLFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity));
        }
        return fiducials;
    }

    /** Gets a Megatag 2 pose estimate */
    public Optional<LLPoseEstimate> getPoseEstimateMT2() {
        TimestampedDoubleArray poseEntry = getLLDoubleArrayEntry("botpose_orb_wpiblue").getAtomic();
        var poseArray = poseEntry.value;
        long timestamp = poseEntry.timestamp;
        var fiducials = getFiducials();
        if (fiducials.size() == 0) {
            return Optional.empty(); //Returns empty optional if no fiducials are detected
        }
        Pose3d pose = new Pose3d(poseArray[0],poseArray[1],poseArray[2],new Rotation3d(poseArray[3],poseArray[4],poseArray[5]));
        double latency = poseArray[6];
        int tagCount = (int)poseArray[7];
        double tagSpan = poseArray[8];
        double avgTagDist = poseArray[9];
        double avgTagArea = poseArray[10];
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
        return Optional.of(new LLPoseEstimate(pose,adjustedTimestamp,latency,tagCount,tagSpan,avgTagDist,avgTagArea,fiducials,true));
    }

    public String getName() {
        return NTName;
    }

    private DoubleArrayEntry getLLDoubleArrayEntry(String key) {
        return LLNetworkTable.getDoubleArrayTopic(key).getEntry(new double[]{});
    }

    
}
