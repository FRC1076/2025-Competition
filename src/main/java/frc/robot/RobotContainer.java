// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.auto.AutomatedL1Score;
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
import lib.hardware.hid.SamuraiPS5Controller;
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
import frc.robot.Constants.CANRangeConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.subsystems.Superstructure;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultSingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.Photonvision.driverCamName;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultMultiTagStdDevs;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.ElevatorClutchRotFactor;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.ElevatorClutchTransFactor;
import static frc.robot.Constants.DriveConstants.DriverControlConstants.elevatorAccelerationTable;
import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.VisionConstants.fieldLayout;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.hardware.CANrange;
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
    private final Trigger m_grabberCANRange;
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

    private final CommandPS5Controller m_droperatorController = 
        new SamuraiPS5Controller(OIConstants.kDroperatorControllerPort)
            .withDeadband(OIConstants.kControllerDeadband);
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
    private final TeleopDriveCommand slowToStopDrivetrain;

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
        CANrange grabberCANRange = new CANrange(CANRangeConstants.grabberCANRangeId);
        m_grabberCANRange = new Trigger(() -> (grabberCANRange.getDistance().getValue().in(Meters) < CANRangeConstants.grabberCANRangeTriggerDistanceMeters));
        m_transferBeamBreak = new Trigger(() -> {return ! transferDIO.get();});//.or(m_beamBreakController.x());
       
        if (OIConstants.kUseDroperatorController) {
            // Use Droperator controller
            m_interruptElevator = m_droperatorController.L3();
            m_interruptWrist = m_droperatorController.R3();
        } else {
            // Regular operator controller
            m_interruptElevator = new Trigger(() -> m_operatorController.getLeftY() != 0);
            m_interruptWrist = new Trigger(() -> m_operatorController.getRightY() != 0);
        }
    
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
            m_drive,
            m_elastic,
            m_transferBeamBreak,
            m_grabberCANRange
        );

        superVis = new SuperstructureVisualizer(m_superstructure);

        slewRateLimiter = new DynamicSlewRateLimiter2d(
            () -> elevatorAccelerationTable.get(m_elevator.getPositionMeters()),
            0
        );

        if (OIConstants.kUseDroperatorController) {
            teleopDriveCommand = m_drive.CommandBuilder.teleopDrive(
                () -> slewRateLimiterEnabled
                    ? slewRateLimiter.calculateY(-m_droperatorController.getLeftX(), -m_droperatorController.getLeftY())
                    : -m_droperatorController.getLeftY(),
                () -> slewRateLimiterEnabled
                    ? slewRateLimiter.calculateX(-m_droperatorController.getLeftX(), -m_droperatorController.getLeftY())
                    : -m_droperatorController.getLeftX(),
                () -> -m_droperatorController.getRightX()
            );
        } else {
            teleopDriveCommand = m_drive.CommandBuilder.teleopDrive(
                () -> slewRateLimiterEnabled
                    ? slewRateLimiter.calculateY(-m_driverController.getLeftX(), -m_driverController.getLeftY())
                    : -m_driverController.getLeftY(),
                () -> slewRateLimiterEnabled
                    ? slewRateLimiter.calculateX(-m_driverController.getLeftX(), -m_driverController.getLeftY())
                    : -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX()
            );
        }

        slowToStopDrivetrain = m_drive.CommandBuilder.teleopDrive(() -> slewRateLimiter.calculateY(0, 0), () -> slewRateLimiter.calculateX(0, 0), () -> 0.0);

        // Drive team status triggers
        m_safeToFeedCoral = new Trigger(() -> m_superstructure.getSafeToFeedCoral());
        m_safeToMoveElevator = new Trigger(() -> m_superstructure.getSafeToMoveElevator());
        m_isAutoAligned = new Trigger(() -> m_drive.isAutoAligned());
        // m_isAutoAligned = new Trigger(() -> teleopDriveCommand.isAutoAligned());
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
        
        if (OIConstants.kUseDroperatorController) {
            // Use combined driver and operator controller
            configureDroperatorBindings();

            // Enable color controller?
            if (OIConstants.kUseOperiverColorController) {
                configureColorBindings();
            }
        } else if (OIConstants.kUseAlternateDriverController) {
            // Use alternate shared and driver bindings
            configureAlternateSharedBindings();
            configureAlternateDriverBindings();

            // Configure the operator bindings
            configureOperatorBindings();
        } else {
            // Use default shared and driver bindings
            configureSharedBindings();
            configureDriverBindings();

            // Configure the operator bindings
            configureOperatorBindings();
        }
    
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {

        // Standard 3 coral auto
        return AutoBuilder.buildAuto("Grabber J4_K4_L4 - E4_D4_C4");

        // Compementary 2 coral auto
        // return AutoBuilder.buildAuto("Grabber A4-B4 - B4-A4");

        // Complementary 1 coral 2 net auto
        // return AutoBuilder.buildAuto("H4_GHnet_IJnet");

        // Untested for a while 2 coral (funnel)
        // return AutoBuilder.buildAuto("J4_K4 - E4_D4");

        // Select auto from Elastic (leave disabled)
        // return m_autoChooser.getSelected();

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

        final Command scoreNet = 
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                    Commands.sequence(
                        superstructureCommands.preNet(),
                        Commands.sequence(
                            Commands.waitUntil(() -> {return m_superstructure.getElevator().getPositionMeters() > 1.9158291;}), //1.7 //1.9158291
                            superstructureCommands.doGrabberAction()
                        )
                    )
                );
        
        NamedCommands.registerCommand("preL1", superstructureCommands.preL1());
        NamedCommands.registerCommand("preL2", superstructureCommands.preL2());
        NamedCommands.registerCommand("preL3", superstructureCommands.preL3());
        NamedCommands.registerCommand("preL4", superstructureCommands.preL4());
        NamedCommands.registerCommand("preL4Direct", superstructureCommands.preL4Direct());
        NamedCommands.registerCommand("preProcessor", superstructureCommands.preProcessor());
        NamedCommands.registerCommand("lowAlgae", superstructureCommands.lowAlgaeIntake());
        NamedCommands.registerCommand("lowAlgaeDirect", superstructureCommands.lowAlgaeDirectIntake());
        NamedCommands.registerCommand("highAlgae", superstructureCommands.highAlgaeIntake());
        NamedCommands.registerCommand("highAlgaeDirect", superstructureCommands.highAlgaeIntakeDirect());
        NamedCommands.registerCommand("autonAlgaeIntake", superstructureCommands.autonAlgaeIntake());
        NamedCommands.registerCommand("autonAlgaeHold", superstructureCommands.holdAlgae());
        NamedCommands.registerCommand("scoreNet", scoreNet);
        // NamedCommands.registerCommand("preNet", superstructureCommands.preNet());
        NamedCommands.registerCommand("preIntakeCoral", superstructureCommands.preIntakeCoral());
        NamedCommands.registerCommand("autonIntakeCoral", superstructureCommands.autonIntakeCoral());
        NamedCommands.registerCommand("autonGrabberIntakeCoral", superstructureCommands.autonGrabberIntakeCoral());
        NamedCommands.registerCommand("autonGrabberAdjustCoral", superstructureCommands.autonGrabberAdjustCoral());
        NamedCommands.registerCommand("autonShoot", superstructureCommands.autonShoot());
        NamedCommands.registerCommand("stopAndRetract", superstructureCommands.stopAndRetract());
        NamedCommands.registerCommand("preAutomaticNet", superstructureCommands.preAutomaticNet().asProxy());
        NamedCommands.registerCommand("stopGrabber", superstructureCommands.stopGrabber());
        // NamedCommands.registerCommand("wristFlickUp", superstructureCommands.wristFlickUp()); didn't work before
    }

    private void configureSharedBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();

        // Shoot
        m_driverController.rightTrigger()
            .or(m_operatorController.rightTrigger())
                .onTrue(superstructureCommands.doGrabberAction())
                    .whileFalse(superstructureCommands.stopAndRetract());
    }

    private void configureDriverBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();

        m_driverController.a().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestLeftBranch()
            )
        );

        m_driverController.b().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestRightBranch()
            )
        );

        m_driverController.povLeft().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestLeftL1()
            )
        );

        /*
        m_driverController.povUp().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestCenterL1()
            )
        );*/

        m_driverController.povRight().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestRightL1()
            )
        );
        
        // Point to reef
        // m_driverController.y().whileTrue(teleopDriveCommand.applyReefHeadingLock());

        // Shoot and go to algae intake from reef
        m_driverController.rightBumper().and(m_driverController.leftBumper().negate())
            .onTrue(superstructureCommands.doGrabberAction())
            .onFalse(superstructureCommands.stopAndAlgaeIntake());

        // Apply double clutch
        m_driverController.leftBumper().and(m_driverController.rightBumper().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Apply FPV Driving
        m_driverController.leftBumper().and(m_driverController.rightBumper()).and(m_driverController.x().negate()).or(m_driverController.leftTrigger())
            .whileTrue(
                teleopDriveCommand.applyDoubleClutch()
                /* Disabled as it is a feature for advanced drivers
                Commands.parallel(
                    teleopDriveCommand.applyDoubleClutch(),
                    Commands.startEnd(
                        () -> slewRateLimiterEnabled = false,
                        () -> slewRateLimiterEnabled = true
                    )
                )
                */
            );

        m_driverController.x().and(m_driverController.leftBumper().negate()).and(m_driverController.rightBumper().negate())
            .onTrue(m_LEDs.setStateTimed(LEDStates.HUMAN_PLAYER_SIGNAL, 5));

        //m_driverController.povUp().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = true));

        //m_driverController.povDown().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = false));

        m_driverController.leftBumper().and(
            m_driverController.rightBumper().and(
                m_driverController.x()
            )
        ).onTrue(new InstantCommand(
            () -> m_drive.resetHeading()
        )); 

        /*
        m_driverController.y().whileTrue(
            Commands.sequence(
                m_drive.CommandBuilder.directDriveToNearestPreNetLocation(),
                superstructureCommands.preNet(),
                Commands.parallel(
                    m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superstructureCommands.doGrabberAction()
                    )
                )
            )
        );*/
        
        m_driverController.y().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                Commands.sequence(
                    Commands.parallel(
                        superstructureCommands.preAutomaticNet().asProxy(),
                        m_drive.CommandBuilder.directDriveToNearestPreNetLocation()
                    ),
                    Commands.parallel(
                        m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                        superstructureCommands.preNet(),
                        Commands.sequence(
                            Commands.waitUntil(() -> {return m_superstructure.getElevator().getPositionMeters() > 1.9158291;}), //1.7 //1.9158291
                            superstructureCommands.doGrabberAction()
                        )
                    )
                )
            )
        );
        /*
        m_driverController.povLeft()
            .whileTrue(new RepeatCommand(new AutomatedL1Score(m_drive, m_superstructure, m_grabberCANRange)).andThen(Commands.print("RepeatL1Cancelled")))
            .onFalse(m_superstructure.applyGrabberState(GrabberState.IDLE));*/
    }

    private void configureAlternateSharedBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();

        // Shoot
        m_driverController.b()
            .or(m_operatorController.rightTrigger())
                .onTrue(superstructureCommands.doGrabberAction())
                    .whileFalse(superstructureCommands.stopAndRetract());
    }

    private void configureAlternateDriverBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();

        m_driverController.leftTrigger().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestLeftBranch()
            )
        );

        m_driverController.rightTrigger().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestRightBranch()
            )
        );

        m_driverController.povLeft().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestLeftL1()
            )
        );

        /*
        m_driverController.povUp().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestCenterL1()
            )
        );*/

        m_driverController.povRight().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestRightL1()
            )
        );
        
        // Point to reef
        // m_driverController.y().whileTrue(teleopDriveCommand.applyReefHeadingLock());

        // Shoot and go to algae intake from reef
        m_driverController.a().and(m_driverController.leftBumper().negate())
            .onTrue(superstructureCommands.doGrabberAction())
            .onFalse(superstructureCommands.stopAndAlgaeIntake());

        // Apply single clutch
        m_driverController.rightBumper().and(m_driverController.leftBumper().negate())
            .whileTrue(teleopDriveCommand.applySingleClutch());

        // Apply double clutch
        m_driverController.leftBumper().and(m_driverController.rightBumper().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Apply FPV Driving
        m_driverController.leftBumper().and(m_driverController.rightBumper()).and(m_driverController.x().negate())
            .whileTrue(
                teleopDriveCommand.applyDoubleClutch()
                /* Disabled as it is a feature for advanced drivers
                Commands.parallel(
                    teleopDriveCommand.applyDoubleClutch(),
                    Commands.startEnd(
                        () -> slewRateLimiterEnabled = false,
                        () -> slewRateLimiterEnabled = true
                    )
                )
                */
            );

        m_driverController.x().and(m_driverController.leftBumper().negate()).and(m_driverController.rightBumper().negate())
            .onTrue(m_LEDs.setStateTimed(LEDStates.HUMAN_PLAYER_SIGNAL, 5));

        //m_driverController.povUp().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = true));

        //m_driverController.povDown().onTrue(Commands.runOnce(() -> slewRateLimiterEnabled = false));

        m_driverController.leftBumper().and(
            m_driverController.rightBumper().and(
                m_driverController.x()
            )
        ).onTrue(new InstantCommand(
            () -> m_drive.resetHeading()
        )); 

        /*
        m_driverController.y().whileTrue(
            Commands.sequence(
                m_drive.CommandBuilder.directDriveToNearestPreNetLocation(),
                superstructureCommands.preNet(),
                Commands.parallel(
                    m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        superstructureCommands.doGrabberAction()
                    )
                )
            )
        );*/
        
        m_driverController.y().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                Commands.sequence(
                    Commands.parallel(
                        superstructureCommands.preAutomaticNet().asProxy(),
                        m_drive.CommandBuilder.directDriveToNearestPreNetLocation()
                    ),
                    Commands.parallel(
                        m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                        superstructureCommands.preNet(),
                        Commands.sequence(
                            Commands.waitUntil(() -> {return m_superstructure.getElevator().getPositionMeters() > 1.9158291;}), //1.7 //1.9158291
                            superstructureCommands.doGrabberAction()
                        )
                    )
                )
            )
        );
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
        
        // Retract mechanisms without shooting
        m_operatorController.back()
            .onTrue(superstructureCommands.retractMechanisms());

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

        if (OIConstants.kUseAlternateOperatorController) {
            // Swap controls for ground algae intake and processor

            // Ground Algae Intake
            m_operatorController.x()
                .and(m_operatorController.leftBumper())
                .onTrue(superstructureCommands.groundAlgaeIntake());

            // Processor
            m_operatorController.leftTrigger()
                .and(m_operatorController.leftBumper())
                .onTrue(superstructureCommands.preProcessor());
        } else {
            // Ground Algae Intake
            m_operatorController.leftTrigger()
                .and(m_operatorController.leftBumper())
                .onTrue(superstructureCommands.groundAlgaeIntake());

            // Processor
            m_operatorController.x()
                .and(m_operatorController.leftBumper())
                .onTrue(superstructureCommands.preProcessor());
        }

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

        m_operatorController.rightBumper()
            .whileTrue(superstructureCommands.grabberIntakeCoral())
            .onFalse(superstructureCommands.stopGrabber());

        m_operatorController.povRight()
            .onTrue(superstructureCommands.lollipopAlgaeIntake());

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

        /*
        m_operatorController.povRight().onTrue(
            superstructureCommands.holdAlgae()
        ).onFalse(
            superstructureCommands.stopGrabber()
        );*/

        // Interrupts any elevator command when the the left joystick is moved
        m_interruptElevator.onTrue(superstructureCommands.interruptElevator());

        // Interrupts any wrist command when the right joystick is moved
        m_interruptWrist.onTrue(superstructureCommands.interruptWrist());
        
        m_operatorController.povLeft()
            .onTrue(superstructureCommands.doGrabberAction())
            .onFalse(superstructureCommands.stopAndAlgaeIntake());

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

    // Stuff for droperator algae mode
    boolean algaeModeEnabled = false;
    private void setAlgaeMode(boolean val) {
        algaeModeEnabled = val;
    }
    @AutoLogOutput
    private boolean getAlgaeMode() {
        return algaeModeEnabled || (m_operatorController.leftBumper().getAsBoolean() && !m_operatorController.rightBumper().getAsBoolean());
    }

    // Stuff for droperator manual wrist/elevator control
    boolean manualMechanismControlEnabled = false;

    private void setManualMechanismControl(boolean val) {
        manualMechanismControlEnabled = val;
    }

    @AutoLogOutput
    private boolean getManualMechanismControl() {
        return manualMechanismControlEnabled;
    }

    private void configureDroperatorBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        Trigger algaeMode = new Trigger(() -> algaeModeEnabled);
        Trigger manualMechanismControl = new Trigger(() -> manualMechanismControlEnabled);

        // Auto-align left
        m_droperatorController.povLeft().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestLeftBranch()
            )
        );

        // Auto-align right
        m_droperatorController.povRight().whileTrue(
            Commands.parallel(
                Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                m_drive.CommandBuilder.directDriveToNearestRightBranch()
            )
        );

        // Shoot
        m_droperatorController.R2()
            .and(m_droperatorController.L2().negate())
            .or(m_droperatorController.R2().and(algaeMode)) // Manual extake is disabled when algae mode is on
                .onTrue(superstructureCommands.doGrabberAction())
                .whileFalse(superstructureCommands.stopAndRetract());

        // Shoot and go to algae intake from reef
        m_droperatorController.R1().and(m_droperatorController.L1().negate())
            .onTrue(superstructureCommands.doGrabberAction())
            .onFalse(
                Commands.sequence(
                    superstructureCommands.stopAndAlgaeIntake(),
                    Commands.runOnce(() -> setAlgaeMode(true))
                )
            );

        // Apply double clutch
        m_droperatorController.L1().and(m_droperatorController.R1().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Apply FPV Driving
        m_droperatorController.L1().and(m_droperatorController.R1())
            .whileTrue(
                teleopDriveCommand.applyDoubleClutch()
                /* Disabled as it is a feature for advanced drivers
                Commands.parallel(
                    teleopDriveCommand.applyDoubleClutch(),
                    Commands.startEnd(
                        () -> slewRateLimiterEnabled = false,
                        () -> slewRateLimiterEnabled = true
                    )
                )
                */
            );

        // Reset gyro
        m_droperatorController.create()
            .onTrue(new InstantCommand(
                () -> m_drive.resetHeading()
            )); 

        // Auto-net
        m_droperatorController.povUp()
            .whileTrue(
                Commands.parallel(
                    Commands.run(() -> m_LEDs.setState(LEDStates.AUTO_ALIGNING), m_LEDs),
                    Commands.sequence(
                        Commands.parallel(
                            superstructureCommands.preAutomaticNet().asProxy(),
                            m_drive.CommandBuilder.directDriveToNearestPreNetLocation()
                        ),
                        Commands.parallel(
                            m_drive.CommandBuilder.directDriveToNearestScoreNetLocation(),
                            superstructureCommands.preNet(),
                            Commands.sequence(
                                Commands.waitUntil(() -> {return m_superstructure.getElevator().getPositionMeters() > 1.9158291;}), //1.7 //1.9158291
                                superstructureCommands.doGrabberAction()
                            )
                        )
                    )
                )
            )
            .onFalse(
                Commands.runOnce(() -> setAlgaeMode(false))
            );

        // Retract mechanisms without shooting
        m_droperatorController.PS()
            .onTrue(superstructureCommands.retractMechanisms());

        // L1
        m_droperatorController.square()
            .and(algaeMode.negate())
                .onTrue(superstructureCommands.preL1());
 
        // L2
        m_droperatorController.cross()
            .and(algaeMode.negate())
                .onTrue(superstructureCommands.preL2());
 
        // L3
        m_droperatorController.circle()
            .and(algaeMode.negate())
                .onTrue(superstructureCommands.preL3());
 
        // L4
        m_droperatorController.triangle()
            .and(algaeMode.negate())
                .onTrue(superstructureCommands.preL4());
        
        // Toggle algae mode
        m_droperatorController.touchpad().onTrue(
            Commands.runOnce(() -> setAlgaeMode(!algaeModeEnabled))
        );

        // Processor
        m_droperatorController.square()
            .and(algaeMode)
                .onTrue(superstructureCommands.preProcessor());
 
        // Low Algae Intake
        m_droperatorController.cross()
            .and(algaeMode)
                .onTrue(superstructureCommands.lowAlgaeIntake());
 
        // High Algae Intake
        m_droperatorController.circle()
            .and(algaeMode)
                .onTrue(superstructureCommands.highAlgaeIntake());
 
        // Net
        m_droperatorController.triangle()
            .and(algaeMode)
                .onTrue(superstructureCommands.preNet());
 
        // Coral Intake and transfer into Grabber
        m_droperatorController.L2()
            .and(algaeMode.negate())
                .whileTrue(superstructureCommands.intakeCoral())
                .whileFalse(superstructureCommands.stopIntake());

        // Grabber intake coral        
        m_droperatorController.povDown()
            .whileTrue(superstructureCommands.grabberIntakeCoral())
            .onFalse(superstructureCommands.stopGrabber());
 
        /*
         m_droperatorController.L1()
            .and(m_droperatorController.R1())
            .and(m_droperatorController.L2())
            .and(m_droperatorController.R2())
            .and(m_droperatorController.povDown())
            .and(m_droperatorController.square())
            .and(() -> (m_droperatorController.getRightY() < -0.9))
                .onTrue(superstructureCommands.lollipopAlgaeIntake());
        */
 
        // Ground Algae Intake
        m_droperatorController.L2()
            .and(algaeMode)
                .onTrue(superstructureCommands.groundAlgaeIntake());
        
        // Manual extake
        m_droperatorController.L2()
            .and(m_droperatorController.R2())
            .and(algaeMode.negate())
                .onTrue(
                    m_superstructure.holdIndexState(IndexState.TRANSFER).alongWith(
                    m_superstructure.applyGrabberState(GrabberState.CORAL_INTAKE)))
                .onFalse(
                    m_superstructure.holdIndexState(IndexState.BACKWARDS).alongWith(
                    m_superstructure.applyGrabberState(GrabberState.IDLE)));

        // Stop drivetrain when using manual mechanism control
        manualMechanismControl 
            .whileTrue(
                Commands.parallel(
                    slowToStopDrivetrain,
                    m_wrist.applyManualControl(
                        () -> -m_droperatorController.getRightY()
                    ),
                    m_elevator.applyManualControl(
                        () -> -m_droperatorController.getLeftY(),
                        () -> false
                    )
                )
                );
        
        // Manual wrist control
        m_droperatorController.R3()
            .onTrue(
                Commands.runOnce(() -> setManualMechanismControl(!manualMechanismControlEnabled))
            );

        // Manual elevator control
        m_droperatorController.L3()
            .onTrue(
                Commands.runOnce(() -> setManualMechanismControl(!manualMechanismControlEnabled))
            );

        // Interrupts any elevator command when the the left joystick is moved
        m_interruptElevator.onTrue(superstructureCommands.interruptElevator());
 
        // Interrupts any wrist command when the right joystick is moved
        m_interruptWrist.onTrue(superstructureCommands.interruptWrist());
        
        // Re-zero elevator
        m_droperatorController.options().whileTrue(
            Commands.parallel(
                m_elevator.autoHome(),
                m_wrist.applyAngle(algaeTravelAngle)
            )
        ); 
    }

    private void configureColorBindings() {
        // Red
        m_operatorController.b()
            .whileTrue(
                Commands.run(() -> m_LEDs.setState(LEDStates.RED_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Orange
        m_operatorController.povUp()
            .whileTrue(
                Commands.run(() -> m_LEDs.setState(LEDStates.ORANGE_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Yellow
        m_operatorController.y()
            .whileTrue(
                Commands.run(() -> m_LEDs.setState(LEDStates.YELLOW_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Green
        m_operatorController.a()
            .whileTrue(
                Commands.run(() -> m_LEDs.setState(LEDStates.GREEN_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Blue
        m_operatorController.x()
            .whileTrue(
                Commands.run(() -> m_LEDs.setState(LEDStates.BLUE_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));

        // Purple
        m_operatorController.povDown()
            .whileTrue(
                Commands.run(() -> m_LEDs.setStateTimed(LEDStates.PURPLE_HP_SIGNAL), m_LEDs)
            ).onFalse(Commands.run(() -> m_LEDs.setState(LEDStates.IDLE), m_LEDs));
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
            Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
            Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp())
        ).ignoringDisable(true);
    }

    public void enableAllCameras() {
        m_vision.enableAllCameras(true);
    }

    public void disableRearRightCamera() {
        m_vision.enableCameras(false, "BACK_RIGHT_CAM");
    }
}
