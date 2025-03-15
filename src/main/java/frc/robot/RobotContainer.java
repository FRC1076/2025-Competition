// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexIOHardware;
import frc.robot.subsystems.index.IndexIOSim;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.led.LEDIODigitalPins;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.led.LEDOnRIO;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.control.DynamicSlewRateLimiter2d;
import lib.extendedcommands.CommandUtils;
import lib.hardware.hid.SamuraiXboxController;
import lib.vision.LoggedPhotonVisionLocalizer;
import lib.vision.PhotonVisionLocalizer;
import lib.vision.VisionLocalizationSystem;
import lib.vision.Limelight.LEDState;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.subsystems.Superstructure;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultSingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.Photonvision.driverCamName;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultMultiTagStdDevs;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.ElevatorClutchRotFactor;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.ElevatorClutchTransFactor;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.elevatorAccelerationTable;
import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.VisionConstants.fieldLayout;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_drive;
    private final ElevatorSubsystem m_elevator;
    private final WristSubsystem m_wrist;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final Trigger m_transferBeamBreak;
    private final Trigger m_interruptElevator;
    private final Trigger m_interruptWrist;
    // private final Trigger m_isDisabled;
    private final Trigger m_safeToFeedCoral;
    private final Trigger m_safeToMoveElevator;
    private final Trigger m_isAutoAligned;
    private final Trigger m_elevatorZeroed;
    private final Superstructure m_superstructure;
    private final SuperstructureVisualizer superVis;
    private final Elastic m_elastic;
    private final LEDSubsystem m_LEDs;

    private boolean m_autonState = false;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);

    private final CommandXboxController m_operatorController =
        new SamuraiXboxController(OIConstants.kOperatorControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);
    /* 
    private final CommandXboxController m_beamBreakController = 
        new CommandXboxController(2);
    */
    
    private final SendableChooser<Command> m_autoChooser;

    private final VisionLocalizationSystem m_vision = new VisionLocalizationSystem();

    private final VisionSystemSim m_visionSim;

    private final DynamicSlewRateLimiter2d slewRateLimiter;
    private boolean slewRateLimiterEnabled = true;

   // private final PhotonCamera m_driveCamera;

    private final TeleopDriveCommand teleopDriveCommand;

    private double lastLoopTime = 0;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

    /* 
    DO NOT REFACTOR INTO A SWITCH STATEMENT!!! 
    Because the expressions being evaluated are known at compile time, the compiler will 
    discard the unused branches, functioning as a shitty bootleg version of
    if constexpr
    Also, ignore the "comparing identical expressions" and "dead code" warnings
    */
    
        DigitalInput transferDIO = new DigitalInput(BeamBreakConstants.transferBeamBreakPort);
        m_transferBeamBreak = new Trigger(() -> {return ! transferDIO.get();});//.or(m_beamBreakController.x());
        m_interruptElevator = new Trigger(() -> m_operatorController.getLeftY() != 0);
        m_interruptWrist = new Trigger(() -> m_operatorController.getRightY() != 0);
    
        // m_driveCamera = new PhotonCamera(driverCamName);
        // m_driveCamera.setDriverMode(true);
        
        if (SystemConstants.currentMode == 0) {
            m_visionSim = null;
            m_elastic = new Elastic(this);
            m_drive = new DriveSubsystem(new DriveIOHardware(TunerConstants.createDrivetrain()), m_vision, m_elastic);

            // BLUE
            if(GameConstants.teamColor == Alliance.Blue){
                m_drive.resetPose(new Pose2d(7.177, 5.147, Rotation2d.fromDegrees(180)));
            }
            // RED
            else{ 
                m_drive.resetPose(new Pose2d(10.380, 3.043, Rotation2d.fromDegrees(0)));
            }
            
            m_elevator = new ElevatorSubsystem(new ElevatorIOHardware(), this::getLoopTime);
            m_wrist = new WristSubsystem(new WristIOHardware(), this::getLoopTime);
            m_grabber = new GrabberSubsystem(new GrabberIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
            m_LEDs = new LEDSubsystem(new LEDOnRIO());
            for (PhotonConfig config : PhotonConfig.values()){
                var cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizer(
                    cam, 
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    fieldLayout,
                    config.defaultSingleTagStdDevs, 
                    config.defaultMultiTagStdDevs)
                );
            }
        } else if (SystemConstants.currentMode == 1) {
            m_elastic = new Elastic(this);
            m_drive = new DriveSubsystem(new DriveIOSim(TunerConstants.createDrivetrain()), m_vision, m_elastic);
            m_elevator = new ElevatorSubsystem(new ElevatorIOSim(), this::getLoopTime);
            m_wrist = new WristSubsystem(new WristIOSim(), this::getLoopTime);
            m_grabber = new GrabberSubsystem(new GrabberIOSim());
            m_index = new IndexSubsystem(new IndexIOSim());
            m_LEDs = new LEDSubsystem(new LEDIOSim());
            m_visionSim = new VisionSystemSim("main");
            /*
            for (PhotonConfig config : PhotonConfig.values()){
                var cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizer(
                    cam, 
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    fieldLayout,
                    kDefaultSingleTagStdDevs, 
                    kDefaultMultiTagStdDevs)
                );
                m_visionSim.addCamera(new PhotonCameraSim(cam), config.offset);
            }
            CommandUtils.makePeriodic(() -> m_visionSim.update(m_drive.getPose()));
            */
        }

        m_superstructure = new Superstructure(
            m_elevator,
            m_grabber,
            m_index, 
            m_wrist, 
            m_elastic,
            m_transferBeamBreak
        );

        superVis = new SuperstructureVisualizer(m_superstructure);

        slewRateLimiter = new DynamicSlewRateLimiter2d(
            () -> elevatorAccelerationTable.get(m_elevator.getPositionMeters()),
            0
        );

        teleopDriveCommand = m_drive.CommandBuilder.teleopDrive(
            () -> slewRateLimiterEnabled
                ? slewRateLimiter.calculateY(-m_driverController.getLeftX(), -m_driverController.getLeftY())
                : -m_driverController.getLeftY(),
            () -> slewRateLimiterEnabled
                ? slewRateLimiter.calculateX(-m_driverController.getLeftX(), -m_driverController.getLeftY())
                : -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        );

        // Drive team status triggers
        m_safeToFeedCoral = new Trigger(() -> m_superstructure.getSafeToFeedCoral());
        m_safeToMoveElevator = new Trigger(() -> m_superstructure.getSafeToMoveElevator());
        // m_isAutoAligned = new Trigger(() -> m_drive.isAutoAligned());
        m_isAutoAligned = new Trigger(() -> teleopDriveCommand.isAutoAligned());
        m_elevatorZeroed = new Trigger(() -> m_elevator.isZeroed());

        m_drive.setDefaultCommand(
            teleopDriveCommand
        );

        //m_grabber.setDefaultCommand(m_superstructure.getCommandBuilder().stopGrabber());

        m_wrist.setDefaultCommand(m_wrist.applyManualControl(
            () -> -m_operatorController.getRightY()
        ));

        m_elevator.setDefaultCommand(m_elevator.applyManualControl(
            () -> -m_operatorController.getLeftY(),
            () -> false
        ));

        m_index.setDefaultCommand(Commands.sequence(
            m_superstructure.holdIndexState(IndexState.BACKWARDS),
            Commands.idle(m_index)
        ));

        m_LEDs.setDefaultCommand(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Configure named commands for auton in PathPlanner
        configureNamedCommands();

        // Configure miscellaneous bindings
        configureBindings();

        // Configure shared bindings
        configureSharedBindings();

        // Configure the driver bindings
        configureDriverBindings();

        // Configure the operator bindings
        configureOperatorBindings();
    
        //configure beam break triggers
        configureBeamBreakTriggers();

        //Build the auto chooser with PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption(
            "DoNothingBlue180", 
            Commands.runOnce(() -> m_drive.resetPose(new Pose2d(7.177, 5.147, Rotation2d.fromDegrees(180))))
        );
        m_autoChooser.addOption(
            "DoNothingRed0", 
            Commands.runOnce(() -> m_drive.resetPose(new Pose2d(10.380, 3.043, Rotation2d.fromDegrees(0))))
        );
        SmartDashboard.putData(m_autoChooser);

        CommandUtils.makePeriodic(() -> m_elastic.updateTeamColor(), true);


    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureBindings() {
        m_isAutoAligned
            .onTrue(
                m_LEDs.setStateTimed(LEDStates.AUTO_ALIGNED))
            .onChange(
                Commands.runOnce(
                    () -> m_elastic.updateIsAutoAligned(m_drive::isAutoAligned)));
        
        m_safeToMoveElevator
            .onTrue(
                m_LEDs.setStateTimed(LEDStates.CORAL_INDEXED))
            .onChange(
                Commands.runOnce(
                    () -> m_elastic.updateSafeToMoveElevator(m_superstructure::getSafeToMoveElevator)));

        m_elevatorZeroed
            .onTrue(
                m_LEDs.setStateTimed(LEDStates.ELEVATOR_ZEROED));

        m_safeToFeedCoral.onChange(
            Commands.runOnce(
                () -> m_elastic.updateSafeToFeedCoral(m_superstructure::getSafeToFeedCoral)));
    }

    private void configureNamedCommands(){
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        
        NamedCommands.registerCommand("preL1", superstructureCommands.preL1());
        NamedCommands.registerCommand("preL2", superstructureCommands.preL2());
        NamedCommands.registerCommand("preL3", superstructureCommands.preL3());
        NamedCommands.registerCommand("preL4", superstructureCommands.preL4());
        NamedCommands.registerCommand("preL4Direct", superstructureCommands.preL4Direct());
        NamedCommands.registerCommand("preProcessor", superstructureCommands.preProcessor());
        NamedCommands.registerCommand("lowAlgae", superstructureCommands.lowAlgaeIntake());
        NamedCommands.registerCommand("highAlgae", superstructureCommands.highAlgaeIntake());
        NamedCommands.registerCommand("preNet", superstructureCommands.preNet());
        NamedCommands.registerCommand("preIntakeCoral", superstructureCommands.preIntakeCoral());
        NamedCommands.registerCommand("autonIntakeCoral", superstructureCommands.autonIntakeCoral());
        NamedCommands.registerCommand("autonGrabberIntakeCoral", superstructureCommands.autonGrabberIntakeCoral());
        NamedCommands.registerCommand("autonGrabberAdjustCoral", superstructureCommands.autonGrabberAdjustCoral());
        NamedCommands.registerCommand("autonShoot", superstructureCommands.autonShoot());
        NamedCommands.registerCommand("autonAlgaeIntakeAndHold", superstructureCommands.autonAlgaeIntakeAndHold());
        NamedCommands.registerCommand("stopAndRetract", superstructureCommands.stopAndRetract());
        // NamedCommands.registerCommand("wristFlickUp", superstructureCommands.wristFlickUp()); didn't work before
    }

    private void configureSharedBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();

        m_driverController.rightTrigger()
            .or(m_operatorController.rightTrigger())
                .onTrue(superstructureCommands.doGrabberAction())
                    .whileFalse(superstructureCommands.stopAndRetract());
    }

    private void configureDriverBindings() {
        m_driverController.a().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestLeftBranch()
        );

        m_driverController.b().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestRightBranch()
        );
        
        // Point to reef
        // m_driverController.y().whileTrue(teleopDriveCommand.applyReefHeadingLock());

        // Apply single clutch
        m_driverController.rightBumper().and(m_driverController.leftBumper().negate())
            .whileTrue(teleopDriveCommand.applySingleClutch());

        // Apply double clutch
        m_driverController.leftBumper().and(m_driverController.rightBumper().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Apply FPV Driving TODO: Finalize bindings and FPV clutch with drive team
        m_driverController.leftBumper().and(m_driverController.rightBumper()).and(m_driverController.x().negate()).or(m_driverController.leftTrigger())
            .whileTrue(
                Commands.parallel(
                    teleopDriveCommand.applyDoubleClutch(),
                    Commands.startEnd(
                        () -> slewRateLimiterEnabled = false,
                        () -> slewRateLimiterEnabled = true
                    )
                )
            );

        m_driverController.x().and(m_driverController.leftBumper().negate()).and(m_driverController.rightBumper().negate())
            .onTrue(m_LEDs.setStateTimed(LEDStates.HUMAN_PLAYER_SIGNAL, 5));

        m_driverController.povUp().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = true));

        m_driverController.povDown().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = false));

        m_driverController.leftBumper().and(
            m_driverController.rightBumper().and(
                m_driverController.x()
            )
        ).onTrue(new InstantCommand(
            () -> m_drive.resetHeading()
        )); 
    }

    private void configureOperatorBindings() {

        //if (SystemConstants.operatorSysID) {
            /* 
            // Quasistsic and Dynamic control scheme for Elevator Sysid
            m_driverController.rightBumper().and(   
                m_driverController.a()
            ).whileTrue(m_elevator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));

            m_driverController.rightBumper().and(
                m_driverController.b()
            ).whileTrue(m_elevator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            m_driverController.rightBumper().and(
                m_driverController.x()
            ).whileTrue(m_elevator.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
            
            m_driverController.rightBumper().and(
                m_driverController.y()
            ).whileTrue(m_elevator.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));
            
        */
        /*
            //Quasistsic and Dynamic control scheme for Wrist Sysid
            
            m_driverController.rightBumper().and(
                m_driverController.a()
            ).whileTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kForward));

            m_driverController.rightBumper().and(
                m_driverController.b()
            ).whileTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            m_driverController.rightBumper().and(
                m_driverController.x()
            ).whileTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kForward));
        
            m_driverController.rightBumper().and(
                m_driverController.y()
            ).whileTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kReverse));
        */
        //}
         
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        
        // L1
        m_operatorController.x()
        .and(m_operatorController.leftBumper().negate())
            .onTrue(superstructureCommands.preL1());

        // L2
        m_operatorController.a()
        .and(m_operatorController.leftBumper().negate())
            .onTrue(superstructureCommands.preL2());

        // L3
        m_operatorController.b()
            .and(m_operatorController.leftBumper().negate())
            .onTrue(superstructureCommands.preL3());

        // L4
        m_operatorController.y()
        .and(m_operatorController.leftBumper().negate())
            .onTrue(superstructureCommands.preL4());

        // Processor
        m_operatorController.x()
            .and(m_operatorController.leftBumper())
            .onTrue(superstructureCommands.preProcessor());

        // Low Algae Intake
        m_operatorController.a()
            .and(m_operatorController.leftBumper())
            .onTrue(superstructureCommands.lowAlgaeIntake());

        // High Algae Intake
        m_operatorController.b()
            .and(m_operatorController.leftBumper())
            .onTrue(superstructureCommands.highAlgaeIntake());

        // Net
        m_operatorController.y()
            .and(m_operatorController.leftBumper())
            .onTrue(superstructureCommands.preNet());

        // Coral Intake and transfer into Grabber
        m_operatorController.leftTrigger()
            .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.intakeCoral())
            .whileFalse(superstructureCommands.stopIntake());

        m_operatorController.povLeft()
            .whileTrue(superstructureCommands.grabberIntakeCoral())
            .onFalse(superstructureCommands.stopGrabber());

        // Manual coral intake and transfer
        m_operatorController.povUp()
            .onTrue(
                m_superstructure.holdIndexState(IndexState.TRANSFER).alongWith(
                m_superstructure.applyGrabberState(GrabberState.CORAL_INTAKE)))
            .onFalse(
                m_superstructure.holdIndexState(IndexState.BACKWARDS).alongWith(
                m_superstructure.applyGrabberState(GrabberState.IDLE)));

        // Manual coral intake reverse and transfer
        m_operatorController.povDown()
            .onTrue(
                m_superstructure.holdIndexState(IndexState.BACKWARDS).alongWith(
                m_superstructure.applyGrabberState(GrabberState.REVERSE_CORAL_INTAKE)))
            .onFalse(
                m_superstructure.applyGrabberState(GrabberState.IDLE));

        // Ground Algae Intake
        m_operatorController.leftTrigger().and(m_operatorController.leftBumper()).onTrue(superstructureCommands.groundAlgaeIntake());

        m_operatorController.povRight().onTrue(
            superstructureCommands.holdAlgae()
        ).onFalse(
            superstructureCommands.stopGrabber()
        );

        // Interrupts any elevator command when the the left joystick is moved
        m_interruptElevator.onTrue(superstructureCommands.interruptElevator());

        // Interrupts any wrist command when the right joystick is moved
        m_interruptWrist.onTrue(superstructureCommands.interruptWrist());
        
        m_operatorController.rightBumper()
            .onTrue(superstructureCommands.removeAlgae())
            .onFalse(superstructureCommands.stopGrabber());

        m_operatorController.start().whileTrue(
            Commands.parallel(
                m_elevator.autoHome(),
                m_wrist.applyAngle(algaeTravelAngle)
            )
        );
    }

    private void configureBeamBreakTriggers() {
        /*
        m_transferBeamBreak.onChange(
            Commands.run(
                () -> m_elastic.updateTransferBeamBreak(m_transferBeamBreak.getAsBoolean())
            ).ignoringDisable(true)
        );*/
    }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        // return Commands.runOnce(() -> m_drive.resetPose(new Pose2d(7.177, 5.147, Rotation2d.fromDegrees(180))));
        return AutoBuilder.buildAuto("Grabber J4_K4_L4 - E4_D4_C4");
        // return AutoBuilder.buildAuto("J4_K4 - E4_D4");
        // return m_autoChooser.getSelected();
    }

    /**
     * Used to prevent the robot from moving quickly when teleop first starts, 
     * before voltages are updated by the normal comands
     */
    public void zeroVoltages() {
        m_drive.driveCO(new ChassisSpeeds());
        m_elevator.setVoltage(0);
        m_wrist.setVoltage(0);
    }

    public void updateLoopTime(double loopTime) {
        lastLoopTime = loopTime;
    }

    public double getLoopTime() {
        return lastLoopTime;
    }

    public void setAutonState(boolean inAuton) {
        m_autonState = inAuton;
    }

    public boolean getAutonState() {
        return m_autonState;
    }

    public static Command threadCommand() {
        return Commands.sequence(
            Commands.waitSeconds(20),
            Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 10))
        ).ignoringDisable(true);
    }
}
