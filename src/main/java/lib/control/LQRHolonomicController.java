// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.control;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import lib.math.MatrixUtils;


/**
 * Combines a state-space model and a Linear-Quadratic Regulator to provide feedback control to a holonomic drivetrain
 * <p>
 * State-space model description:
 * <p>
 * Inputs: x velocity, y velocity, angular velocity (field-oriented)
 * <p>
 * States: x position, y position, rotation
 * <p>
 * A linear state-space model can be represented by the following equation:
 * <p>
 * xdot = Ax + Bu
 * <p>
 * y = Cx + Du 
 * <p>
 * where xdot is the state's rate of change, y is the system's output, x is the system's current state, and u is the input.
 * <p>
 * An LQR Controller works by minimizing a cost function, which is governed by the state-space model of the plant, as well an effort penalty matrix and an error penalty matrix.
 * The effort matrix contains the weights that governs how much the controller penalizes control efforts, and the error matrix governs how much the controller penalizes error. For simplicity's sake,
 * this class automatically generates the weight vectors based on an LQRHolonomicDriveControllerWeights object, which contains tolerances that are inversely proportional to the penalties
 * <p>
 * In field-oriented mode, the controller outputs field-oriented ChassisSpeeds objects,
 * while in chassis-oriented mode, the controller outputs chassis-oriented ChassisSpeeds objects
 */

public class LQRHolonomicController implements HolonomicController {

    /**
     * A POD class for holding the weights of an LQR Holonomic Drive Controller
     * 
     * @param transErrorTolerance Translational error tolerance, a lower value will cause the controller to more aggressively compensate for translational error
     * @param transEffortTolerance Translational effort tolerance, a lower value will cause the controller to dampen its translational feedbacks
     * @param rotErrorTolerance Rotational error tolerance, a lower value will cause the controller to more aggressively compensate for rotational error
     * @param rotEffortTolerance Rotational effort tolerance, a lower value will cause the controller to dampen its rotational feedbacks 
     */
    public static record LQRHolonomicDriveControllerTolerances(
        double transErrorTolerance,
        double transEffortTolerance,
        double rotErrorTolerance, 
        double rotEffortTolerance 
    ) {}

    private final LinearSystem<N3,N3,N3> drivePlant; // A state-space system representing the robot's drivetrain
    private final LinearQuadraticRegulator<N3,N3,N3> LQRController;

    /**
     * Constructor for an LQR controlled Holonomic Drive Train. All non-dimensionless values should be given in SI base units (seconds, meters, etc.)
     * 
     * @param q a vector representing the tolerances for error (A lower value will cause the controller to more strongly correct for error)
     * @param r a vector representing the tolerances for effort (A lower value will cause the controller to more strongly dampen inputs)
     * @param dt discretization time step, in seconds
     */
    public LQRHolonomicController(Vector<N3> q, Vector<N3> r, double dt) {

        Matrix<N3,N3> A = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {0, 0, 0},
                    {0, 0, 0},
                    {0, 0, 0}
                }
            )
        );

        Matrix<N3,N3> B = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
                }
            )
        );

        Matrix<N3,N3> C = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}
                }
            )
        );

        Matrix<N3,N3> D = new Matrix<>(
            new SimpleMatrix(
                new double[][] {
                    {dt, 0, 0},
                    {0, dt, 0},
                    {0, 0, dt}
                }
            )
        );

        drivePlant = new LinearSystem<>(A,B,C,D); // Models field-oriented state with field-oriented speeds as inputs, as the dynamical system is only nonlinear relative to the chassis
        LQRController = new LinearQuadraticRegulator<>(drivePlant, q, r, dt);
    }

    /**
     * Constructor for an LQR controlled Holonomic Drive Train. All non-dimensionless values should be given in SI base units (seconds, meters, etc.)
     * 
     * @param errorTolerance the error tolerance (A lower value will cause the feedback controller to behave more aggressively)
     * @param effortTolerance the effort tolerance (A lower value will cause the feedback controller to behave less aggressively)
     * @param dt discretization time step, in seconds
     * @param fieldOriented whether or not the controller should output field-oriented outputs
     */
    public LQRHolonomicController(double errorTolerance, double effortTolerance, double dt) {
        this(
            VecBuilder.fill(errorTolerance,errorTolerance,errorTolerance),
            VecBuilder.fill(effortTolerance,effortTolerance,effortTolerance),
            dt
        );
    }

    /**
     * Constructor for an LQR controlled Holonomic Drive Train. All non-dimensionless values should be given in SI base units (seconds, meters, etc.)
     * 
     * @param tolerances contains the tolerances that govern the LQR controller
     * @param dt discretization time step, in seconds
     */
    public LQRHolonomicController(LQRHolonomicDriveControllerTolerances tolerances, double dt) {
        this(
            VecBuilder.fill(tolerances.transErrorTolerance(),tolerances.transErrorTolerance(),tolerances.rotErrorTolerance()),
            VecBuilder.fill(tolerances.transEffortTolerance(),tolerances.transEffortTolerance(),tolerances.rotEffortTolerance()),
            dt
        );
    }



    /* Returns a state-space model of the plant */
    public LinearSystem<N3,N3,N3> getDrivePlant(){
        return drivePlant;
    }

    /* Returns the internal LQR controller */
    public LinearQuadraticRegulator<N3,N3,N3> getController(){
        return LQRController;
    }

    /**
     * Gets the next calculation of the internal LQR controller, without any additional processing
     * 
     * @param pv
     * @param setpoint
     * @return
     * the calculated field-oriented controller feedbacks, in the form of a vector
     */
    public Matrix<N3,N1> calculateRaw(Pose2d pv, Pose2d setpoint) {
        var spVec = MatrixUtils.poseToVector(setpoint);
        var pvVec = MatrixUtils.poseToVector(pv);
        return LQRController.calculate(pvVec,spVec);
    }

    /**
     *  Gets the next calculation of the LQR controller
     * 
     * @param pv the robot's current pose
     * @param setpoint the setpoint pose
     * @return
     * a field-oriented ChassisSpeeds object
    */
    public ChassisSpeeds calculateFieldOriented(Pose2d pv, Pose2d setpoint){
        var spVec = MatrixUtils.poseToVector(setpoint);
        var pvVec = MatrixUtils.poseToVector(pv);
        var outputFieldRelative = LQRController.calculate(pvVec, spVec);
        return new ChassisSpeeds(outputFieldRelative.get(0,0),outputFieldRelative.get(1,0),outputFieldRelative.get(2,0));
    }

    /**
     *  Gets the next calculation of the LQR controller
     * 
     * @param pv the robot's current pose
     * @param setpoint the setpoint pose
     * @return
     * a chassis-oriented ChassisSpeeds object
    */
    public ChassisSpeeds calculateChassisOriented(Pose2d pv, Pose2d setpoint){
        var spVec = MatrixUtils.poseToVector(setpoint);
        var pvVec = MatrixUtils.poseToVector(pv);
        var outputFieldRelative = LQRController.calculate(pvVec, spVec);
        return ChassisSpeeds.fromFieldRelativeSpeeds(outputFieldRelative.get(0,0),outputFieldRelative.get(1,0),outputFieldRelative.get(2,0),pv.getRotation());
    }

    /**resets the LQR controller*/
    public void resetController() {
        LQRController.reset();
    }

    /**
     * Adjusts LQR controller to correct for latency
     * 
     * @param dt discretization time-step (seconds)
     * @param ls latency (seconds)
     */
    public void latencyCompensate(double dt, double ls) {
        LQRController.latencyCompensate(drivePlant, dt, ls);
    }

}