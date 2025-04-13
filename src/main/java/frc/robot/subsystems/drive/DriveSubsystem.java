// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.commands.drive.DirectDriveToPose;
import frc.robot.commands.drive.PPDriveToPose;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.utils.Localization;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotOffset;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotLeftL1Offset;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotCenterL1Offset;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotRightL1Offset;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
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
        private final Map<ReefFace, Command> leftL1AlignmentCommands = new HashMap<>();
        // private final Map<ReefFace, Command> centerL1AlignmentCommands = new HashMap<>();
        private final Map<ReefFace, Command> rightL1AlignmentCommands = new HashMap<>();
        private final PPDriveToPose driveToPreNetCommand;
        private final PPDriveToPose driveToScoreNetCommand;
        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
            for (ReefFace face : ReefFace.values()) {
                leftBranchAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.leftBranch.transformBy(robotOffset), Rotation2d.k180deg)));
                reefCenterAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotOffset), Rotation2d.k180deg)));
                rightBranchAlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.rightBranch.transformBy(robotOffset), Rotation2d.k180deg)));
                leftL1AlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotLeftL1Offset), Rotation2d.k180deg)));
                //centerL1AlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotCenterL1Offset), Rotation2d.k180deg)));
                rightL1AlignmentCommands.put(face, directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotRightL1Offset), Rotation2d.k180deg)));
            }
            driveToPreNetCommand = new PPDriveToPose(drive, Pose2d.kZero);
            driveToScoreNetCommand = new PPDriveToPose(drive, Pose2d.kZero);
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
            return Commands.parallel(
                new PPDriveToPose(drive, targetPose),
                Commands.sequence(
                    Commands.runOnce(() -> {isAutoAligned = false;}),
                    Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                    Commands.runOnce(() -> {isAutoAligned = true;})
                ));
            /*
            DirectDriveToPoseCommand directDriveToPoseCommand = new DirectDriveToPoseCommand(drive, targetPose);

            return Commands.parallel(
                    directDriveToPoseCommand,
                    Commands.sequence(
                        Commands.runOnce(() -> {isAutoAligned = false;}),
                        Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                        Commands.run(() -> {isAutoAligned = true;})
                    )

            );*/
        }

        public Command directDriveToPose(Pose2d targetPose, PathConstraints constraints) {
            return Commands.parallel(
                new PPDriveToPose(drive, targetPose, constraints, 0),
                Commands.sequence(
                    Commands.runOnce(() -> {isAutoAligned = false;}),
                    Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                    Commands.runOnce(() -> {isAutoAligned = true;})
                ));
        }

        public Command directDriveToNearestLeftBranch() {
            return new SelectCommand<>(leftBranchAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestLeftL1() {
            return new SelectCommand<>(leftL1AlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        /*
        public Command directDriveToNearestCenterL1() {
            return new SelectCommand<>(centerL1AlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }*/

        public Command directDriveToNearestRightL1() {
            return new SelectCommand<>(rightL1AlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestReefFace() {
            return new SelectCommand<>(reefCenterAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestRightBranch() {
            return new SelectCommand<>(rightBranchAlignmentCommands, () -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestPreNetLocation() {
            return new FunctionalCommand(
                () -> {
                    driveToPreNetCommand.setTargetPose(
                        new Pose2d(
                            getPose().getX() < 8.785
                            ? 7.618 - 5.5 * 0.3048 //- 0.3556
                            : 9.922 + 5.5 * 0.3048, //+ 0.3556,
                            getPose().getY(),
                            getPose().getX() < 8.785
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg
                        )
                    );
                    driveToPreNetCommand.setEndVelocity(0.5);
                    driveToPreNetCommand.schedule();

                },
                () -> {},
                (interrupted) -> driveToPreNetCommand.cancel(),
                () -> driveToPreNetCommand.isFinished()
            );
        }

        public Command directDriveToNearestScoreNetLocation() {
            return new FunctionalCommand(
                () -> {
                    driveToScoreNetCommand.setTargetPose(
                        new Pose2d(
                            getPose().getX() < 8.785
                            ? 7.618 - 0.1 //- 0.3556
                            : 9.922 + 0.1, //+ 0.3556,
                            getPose().getY(),
                            getPose().getX() < 8.785
                            ? Rotation2d.kZero
                            : Rotation2d.k180deg
                        )
                    );
                    driveToScoreNetCommand.setEndVelocity(0);
                    driveToScoreNetCommand.schedule();
                },
                () -> {},
                (interrupted) -> driveToScoreNetCommand.cancel(),
                () -> driveToScoreNetCommand.isFinished()
            );
        }

        public Command autonDriveToScoreNetLocation() {
            return new FunctionalCommand(
                () -> {
                    driveToScoreNetCommand.setTargetPose(
                        new Pose2d(          
                            7.518,
                            5.037,
                            Rotation2d.kZero
                        )
                    );
                    driveToScoreNetCommand.setEndVelocity(0);
                    driveToScoreNetCommand.schedule();
                },
                () -> {},
                (interrupted) -> driveToScoreNetCommand.cancel(),
                () -> driveToScoreNetCommand.isFinished()
            );
        }
        
        public Command applySwerveRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> acceptRequest(requestSupplier.get()));
        }

    }

}
