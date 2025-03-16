// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.commands.drive.PPDriveToPose;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.utils.Localization;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotOffset;

import lib.utils.GeometryUtils;
import lib.vision.VisionLocalizationSystem;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();
    private Boolean hasSetAlliance = false; // Wait until the driverstation had an alliance before setting it
    private boolean isAutoAligned = false;
    public final DriveCommandFactory CommandBuilder;
    private final VisionLocalizationSystem vision;
    private final Elastic elastic;
    
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> acceptRequest(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> acceptRequest(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                acceptRequest(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public DriveSubsystem(DriveIO io, VisionLocalizationSystem vision, Elastic elastic) {
        this.io = io;
        this.vision = vision;
        this.elastic = elastic;
        vision.registerMeasurementConsumer(this.io::addVisionMeasurement); // In DriveIOHardware, addVisionMeasurement is built into the SwerveDrivetrain class

        if (GameConstants.teamColor == Alliance.Red) {
            io.setAllianceRotation(Rotation2d.fromDegrees(180));
        } else {
            io.setAllianceRotation(Rotation2d.fromDegrees(0));
        }
        
        try {
            AutoBuilder.configure(
                () -> elastic.getPathPlannerMirrored() 
                    ? this.getMirroredPose() 
                    : this.getPose(),
                (pose) -> {
                    if (elastic.getPathPlannerMirrored()) {
                        this.resetPoseMirrored(pose);
                    } else {
                        this.resetPose(pose);
                    }
                },
                () -> driveInputs.Speeds,
                (speeds, feedforwards) -> {
                    if (elastic.getPathPlannerMirrored()) {
                        driveCoMirrored(speeds);
                    } else {
                        driveCO(speeds);
                    }
                },
                new PPHolonomicDriveController(
                    // PID constants for translation
                    PathPlannerConstants.Control.transPID,
                    // PID constants for rotation
                    PathPlannerConstants.Control.rotPID
                ),
                RobotConfig.fromGUISettings(),
                () -> GameConstants.teamColor == Alliance.Red,//DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
        CommandBuilder = new DriveCommandFactory(this);

        
    }

    @Override
    public void periodic(){
        // updateModuleInputs and processInputs are only used for logging
        io.periodic(); //currently just for calling sim
        vision.update();
        io.updateInputs(driveInputs);
        // io.updateModuleInputs(frontLeftInputs, 0);
        // io.updateModuleInputs(frontRightInputs, 1);
        // io.updateModuleInputs(rearLeftInputs, 2);
        // io.updateModuleInputs(rearRightInputs, 3);
        Logger.processInputs("Drive", driveInputs);
        // Logger.processInputs("Drive/FrontLeft", frontLeftInputs);
        // Logger.processInputs("Drive/FrontRight", frontRightInputs);
        // Logger.processInputs("Drive/RearLeft", rearLeftInputs);
        // Logger.processInputs("Drive/RearRight", rearRightInputs);

         /*
            if(DriverStation.getAlliance().isPresent()){
                hasSetAlliance = true;
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    io.setAllianceRotation(Rotation2d.fromDegrees(180));
                } else {
                    io.setAllianceRotation(Rotation2d.fromDegrees(0));
                }
            } 
        }*/
    }

    /** This method is not used in any command logic. It is only used for LEDs and Elastic */
    public boolean isAutoAligned() {
        return isAutoAligned;
    }

    public void clearAutoAlignedStatus() {
        isAutoAligned = false;
    }

    /** Swerve drive request with chassis-oriented chassisSpeeds */
    public void driveCO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    /** Swerve drive request with chassis-orriented chassisSpeeds, but mirrors the speeds (used for mirroring paths in auton) */
    public void driveCoMirrored(ChassisSpeeds speeds) {
        speeds.vyMetersPerSecond *= (-1);  
        speeds.omegaRadiansPerSecond *= (-1);
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public double getVelocityMPS() {
        double velMPS = Math.sqrt(Math.pow(driveInputs.Speeds.vxMetersPerSecond,2.0) + Math.pow(driveInputs.Speeds.vyMetersPerSecond,2.0));
        return velMPS;
    }

    public double getAngularVelocityRadPerSec() {
        return driveInputs.Speeds.omegaRadiansPerSecond;
    }

    /** Swerve drive request with field-oriented chassisSpeeds */
    public void driveFO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyFieldSpeeds().withSpeeds(speeds).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
    }
    
    //TODO: ADD DRIVE METHOD TO DRIVE WITH PATHPLANNER WHEELFORCE FEEDFORWARDS
    
    /** Swerve drive request with heading lock */
    public void driveFOHeadingLocked(double xMetersPerSecond, double yMetersPerSecond, Rotation2d targetDirection) {
        FieldCentricFacingAngle request = new FieldCentricFacingAngle()
        .withVelocityX(xMetersPerSecond)
        .withVelocityY(yMetersPerSecond)
        .withTargetDirection(targetDirection)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        request.HeadingController.setPID(3.5, 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        io.acceptRequest(request);
    }

    /** Set swerveRequest */
    public void acceptRequest(SwerveRequest request) {
        io.acceptRequest(request);
    }

    /** Resets the pose of the robot */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Resets the pose of the robot, but mirrors the input Pose2d, while remaining on the same side of the field (used for mirroring paths in auton) */
    public void resetPoseMirrored(Pose2d pose) {
        Pose2d newPose = new Pose2d(
            pose.getX(),
            FieldConstants.fieldWidthMeters - pose.getY(),
            pose.getRotation().unaryMinus());
        io.resetPose(newPose);
    }

    /** Makes the current heading of the robot the default zero degree heading
     * (Used if forward is the wrong direction)
     */
    public void resetHeading() {
        io.resetHeading();
    }

    public Rotation2d getHeading() {
        return io.getPose().getRotation();
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return io.getPose();
    }

    /** Returns the pose of the robot, but mirrored */
    public Pose2d getMirroredPose() {
        Pose2d pose = new Pose2d(
            io.getPose().getX(),
            FieldConstants.fieldWidthMeters - io.getPose().getY(),
            io.getPose().getRotation().unaryMinus());
        return pose;
    }

    public class DriveCommandFactory {
        private final DriveSubsystem drive;
        private final Map<ReefFace, Command> leftBranchAlignmentCommands = new HashMap<>();
        private final Map<ReefFace, Command> reefCenterAlignmentCommands = new HashMap<>();
        private final Map<ReefFace, Command> rightBranchAlignmentCommands = new HashMap<>();
        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
            for (ReefFace face : ReefFace.values()) {
                leftBranchAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.leftBranch.transformBy(robotOffset), Rotation2d.k180deg)));
                reefCenterAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotOffset), Rotation2d.k180deg)));
                rightBranchAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.rightBranch.transformBy(robotOffset), Rotation2d.k180deg)));
            }
        }

        public Command pathfindToPose(Pose2d targetPose) {
            return AutoBuilder.pathfindToPose(
                targetPose,
                PathPlannerConstants.pathConstraints,
                0.0
            );
        }
    
        public Command followPath(PathPlannerPath path){
            return AutoBuilder.followPath(path);
        }

        public TeleopDriveCommand teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
            return new TeleopDriveCommand(drive, xSupplier, ySupplier, omegaSupplier);
        }

        public Command directDriveToPose(Pose2d targetPose) {
            return new PPDriveToPose(drive, targetPose);
        }

        public Command directDriveToNearestLeftBranch() {
            return new SelectCommand<>(leftBranchAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestReefFace() {
            return new SelectCommand<>(reefCenterAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestRightBranch() {
            return new SelectCommand<>(rightBranchAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command dynamicSysID(){
            return null; // WIP
        }

        public Command quasistaticSysID(){
            return null; // WIP
        }
        
        public Command applySwerveRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> acceptRequest(requestSupplier.get()));
        }

    }

}
