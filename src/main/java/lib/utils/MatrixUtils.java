package lib.utils;

import org.apache.commons.lang3.NotImplementedException;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public final class MatrixUtils {
    private MatrixUtils() {
        throw new NotImplementedException("This is a utility class!");
    }
    
    public static Vector<N3> poseToVector(Pose2d pose) {
        return VecBuilder.fill(pose.getX(),pose.getY(),pose.getRotation().getRadians());
    }

}
