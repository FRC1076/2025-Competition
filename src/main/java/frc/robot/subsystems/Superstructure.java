// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.coralTravelAngle;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.RobotSuperState;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.Localization;
import frc.robot.utils.VirtualSubsystem;
import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.SelectWithFallbackCommandFactory;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure class that contains all subsystems and commands for the robot's superstructure <p>
 * Allows all of the subsystems to talk to each other <p>
 * Allows sensors to interact with subystems <p>
 * Contains command factories for actions requiring multiple systems <p>
 */
public class Superstructure extends VirtualSubsystem {

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private final DriveSubsystem m_drive;
    private final Elastic m_elastic;
    private final Trigger elevatorClutchTrigger;

    public final SuperstructureCommandFactory CommandBuilder;

    private final BooleanSupplier possessAlgae = () -> RobotSuperState.getInstance().getPossession() == GrabberPossession.ALGAE;

    private Boolean safeToFeedCoral;
    private Boolean safeToMoveElevator;

    private boolean elevatorClutch = false;

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        DriveSubsystem drive,
        Elastic elastic,
        Trigger transferBeamBreak,
        Trigger grabberCANRange //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        m_drive = drive;
        m_elastic = elastic;
        
        m_elastic.updateTransferBeamBreak(transferBeamBreak.getAsBoolean());
        CommandBuilder = new SuperstructureCommandFactory(this, transferBeamBreak, grabberCANRange);
        elevatorClutchTrigger = new Trigger(this::elevatorClutchSignal);

        this.safeToFeedCoral = false;
        this.safeToMoveElevator = false;
    }

    @Override
    public void periodic() {
        RobotSuperState.getInstance().logSuperstructureToAkit();
    }

    public ElevatorSubsystem getElevator() {
        return m_elevator;
    }

    public GrabberSubsystem getGrabber() {
        return m_grabber;
    }

    public WristSubsystem getWrist() {
        return m_wrist;
    }
    
    public IndexSubsystem getIndex() {
        return m_index;
    }

    /** This method isn't used for any command logic. It's only used to display on LEDs and Elastic */
    public boolean getSafeToFeedCoral(){
        return safeToFeedCoral;
    }

    /** This method isn't used for any command logic. It's only used to display on LEDs and Elastic */
    public boolean getSafeToMoveElevator(){
        return safeToMoveElevator;
    }

    /** This method isn't used for any command logic. It's only used to display on LEDs and Elastic */
    public boolean getElevatorZeroed() {
        return m_elevator.isZeroed();
    }

    public SuperstructureCommandFactory getCommandBuilder(){
        return CommandBuilder;
    }

    private boolean elevatorClutchSignal() {
        return elevatorClutch;
    }

    public Trigger elevatorClutchTrigger() {
        return elevatorClutchTrigger;
    }   

    // Command factories that apply states are private because they are only accessed by the main SuperStructureCommandFactory

    /**
     * Folds back wrist, moves elevator, then deploys wrist
     * <p> If the robot is already in the chosen state, it will skip the premoves
     * <p> The wrist and elevator are held in place after they reach their setpoints with a DaemonCommand, which allows
     * the sequential command to move forward but the action doesn't end
     * @param position the WristevatorState, which consists of elevator height and wrist angle
     * @return generic transition command from one state to another 
     */
    private Command applyWristevatorState(WristevatorState position, double tolerance, boolean grabberDown) {

        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };

        Command wristPreMoveCommand = Commands.either(
            m_wrist.applyAnglePersistent(algaeTravelAngle),
            m_wrist.applyAnglePersistent(grabberDown ? Rotation2d.kCW_90deg : coralTravelAngle),
            possessAlgae
        );

        // Due to command composition semantics, the command composition itself cannot require the subsystems directly
        return Commands.sequence(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateWristevatorGoal(position)),
            CommandUtils.makeDaemon(wristPreMoveCommand),
            Commands.waitUntil(() -> m_wrist.withinTolerance(WristConstants.wristAngleToleranceRadians)),
            //Commands.print("AT PREMOVE"),
            CommandUtils.makeDaemon(m_elevator.applyPositionPersistent(position.elevatorHeightMeters)),
            Commands.waitUntil(() -> m_elevator.withinTolerance(tolerance)),
            //Commands.print("AT ELEVATOR HEIGHT"),
            CommandUtils.makeDaemon(m_wrist.applyAnglePersistent(position.wristAngle)),
            Commands.waitUntil(() -> m_wrist.withinTolerance(WristConstants.wristAngleToleranceRadians)),
            //Commands.print("AT WRIST FINAL"),
            Commands.runOnce(() -> {
                RobotSuperState.getInstance().updateWristevatorState(position);
                RobotSuperState.getInstance().setWristevatorOverride(false);
            }),
            Commands.runOnce(ledSignal)
        );
    }

    private Command applyWristevatorStateNoPremove(WristevatorState position, double tolerance, boolean grabberDown) {

        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };

        // Due to command composition semantics, the command composition itself cannot require the subsystems directly
        return Commands.sequence(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateWristevatorGoal(position)),
            //CommandUtils.makeDaemon(wristPreMoveCommand),
            //Commands.waitUntil(() -> m_wrist.withinTolerance(WristConstants.wristAngleToleranceRadians)),
            //Commands.print("AT PREMOVE"),
            CommandUtils.makeDaemon(m_elevator.applyPositionPersistent(position.elevatorHeightMeters)),
            Commands.waitUntil(() -> m_elevator.withinTolerance(tolerance)),
            //Commands.print("AT ELEVATOR HEIGHT"),
            CommandUtils.makeDaemon(m_wrist.applyAnglePersistent(position.wristAngle)),
            Commands.waitUntil(() -> m_wrist.withinTolerance(WristConstants.wristAngleToleranceRadians)),
            //Commands.print("AT WRIST FINAL"),
            Commands.runOnce(() -> {
                RobotSuperState.getInstance().updateWristevatorState(position);
                RobotSuperState.getInstance().setWristevatorOverride(false);
            }),
            Commands.runOnce(ledSignal)
        );
    }

    /**
     * Folds back wrist, moves elevator, then deploys wrist
     * <p> If the robot is already in the chosen state, it will skip the premoves
     * <p> The wrist and elevator are held in place after they reach their setpoints with a DaemonCommand, which allows
     * the sequential command to move forward but the action doesn't end
     * @param position the WristevatorState, which consists of elevator height and wrist angle
     * @return generic transition command from one state to another 
     */
    private Command applyWristevatorState(WristevatorState position) {
        return applyWristevatorState(position,Units.inchesToMeters(0.5),false);
    }

    private Command applyWristevatorState(WristevatorState position, double tolerance) {
        return applyWristevatorState(position, tolerance,false);
    }

    /**
     * Folds back wrist, moves elevator, then deploys wrist
     * <p> If the robot is already in the chosen state, it will skip the premoves
     * <p> The wrist and elevator are held in place after they reach their setpoints with a DaemonCommand, which allows
     * the sequential command to move forward but the action doesn't end
     * @param position the WristevatorState, which consists of elevator height and wrist angle
     * @return generic transition command from one state to another 
     */
    /*
    private Command applyWristevatorState(WristevatorState position, BooleanSupplier inTolerance) {
        return applyWristevatorState(position, inTolerance, false);
    }*/

    /**
     * Applies a wristevator state directly without any premoves. Potentially dangerous if used when right up against the
     * reef, because the grabber could hit the branches
     * @param position
     * @return
     */
    private Command applyWristevatorStateDirect(WristevatorState position, BooleanSupplier override) {
        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };
        
        return Commands.parallel(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateWristevatorGoal(position)),
            Commands.runOnce(ledSignal),
            Commands.sequence(
                m_elevator.applyPosition(position.elevatorHeightMeters).asProxy(),
                CommandUtils.makeDaemon(m_elevator.holdPosition(position.elevatorHeightMeters), override)
            ),
            Commands.sequence(
                m_wrist.applyAngle(position.wristAngle).asProxy(),
                CommandUtils.makeDaemon(m_wrist.holdAngle(position.wristAngle), override)
            ),
            Commands.runOnce(() -> {
                RobotSuperState.getInstance().updateWristevatorState(position);
                RobotSuperState.getInstance().setWristevatorOverride(false);
            })
        ).until(override);
    }

    /**
     * Applies a wristevator state directly without any premoves. Potentially dangerous if used when right up against the
     * reef, because the grabber could hit the branches
     * @param position
     * @return
     */
    private Command applyWristevatorStateDirect(WristevatorState position) {
        return applyWristevatorStateDirect(position, () -> false);
    }

    
    
    private Command applyWristevatorStateGrabberDown(WristevatorState position, double tolerance) {

        return applyWristevatorState(position,tolerance,true);
    }

    private Command applyWristevatorStateGrabberDown(WristevatorState position) {
        return applyWristevatorStateGrabberDown(position, Units.inchesToMeters(0.5));
    }

    /**
     * Sets grabber wheels to run at desired state
     * @param state the GrabberState
     * @return command to run grabber as certain state
     */
    public Command applyGrabberState(GrabberState state) {
        return Commands.parallel(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command grabberRadiansBangBang(double volts, double rotations) {
        return m_grabber.applyRadiansBangBang(volts, rotations);
    }

    public Command applyIndexState(IndexState state) {
        return Commands.parallel(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateIndexState(state)),
            m_index.applyVolts(state.volts)
        );
    }

    /**
     * Sets indexer to run at desired state
     * @param state the IndexState
     * @return command to run index as certain state
     */
    public Command holdIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> RobotSuperState.getInstance().updateIndexState(state)),
            m_index.applyVolts(state.volts),
            Commands.idle(m_index)//.onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command interruptWrist(){
        return Commands.runOnce(() -> {
            if(m_wrist.getCurrentCommand() != null) m_wrist.getCurrentCommand().cancel();
            RobotSuperState.getInstance().setWristevatorOverride(true);
        });
        
    }

    private Command interruptElevator(){
        return Commands.runOnce(() -> {
            if(m_elevator.getCurrentCommand() != null) m_elevator.getCurrentCommand().cancel();
            RobotSuperState.getInstance().setWristevatorOverride(true);
        });
    }

    /** Contains all the command factories for the superstructure */
    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final Trigger m_transferBeamBreak;
        private final Trigger m_grabberCANRange;
        private final Map<WristevatorState, Supplier<Command>> grabberActionCommands = new HashMap<WristevatorState, Supplier<Command>>(); // We use a map of grabber action commands so that we can use the SelectWithFallBackCommand factory
        private final SelectWithFallbackCommandFactory<WristevatorState> grabberActionCommandFactory;
        private final Set<WristevatorState> algaeIntakeWristevatorStates = Set.of(WristevatorState.GROUND_INTAKE, WristevatorState.LOLLIPOP_INTAKE, WristevatorState.LOW_INTAKE, WristevatorState.HIGH_INTAKE); // Wristevator states that lead to intaking algae

        private SuperstructureCommandFactory (
            Superstructure superstructure,
            Trigger transferBeamBreak,
            Trigger grabberCANRange
        ) {
            this.superstructure = superstructure;
            m_transferBeamBreak = transferBeamBreak;
            m_grabberCANRange = grabberCANRange;
            grabberActionCommands.put(WristevatorState.L1, () -> superstructure.applyGrabberState(GrabberState.L1_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L2, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L3, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); 
            grabberActionCommands.put(WristevatorState.L4, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.GROUND_INTAKE,
                                        () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.LOLLIPOP_INTAKE,
                                        () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.PROCESSOR, () -> superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            grabberActionCommands.put(WristevatorState.LOW_INTAKE,
                                        () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.HIGH_INTAKE,
                                        () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.NET, () -> superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));

            grabberActionCommandFactory = new SelectWithFallbackCommandFactory<WristevatorState>(
                    grabberActionCommands,
                    () -> superstructure.applyGrabberState(GrabberState.DEFAULT_OUTTAKE),
                    RobotSuperState.getInstance()::getWristevatorState
            );
        }

        /**
         * Returns a command that does a grabber action differently depending on robot state (scoring coral, scoring algae, intaking algae)
         * and sets the grabber possession depending on the wristevator state
         */
        public Command doGrabberAction() {
            return Commands.parallel(
                grabberActionCommandFactory.buildCommand(),
                Commands.runOnce(() -> {safeToMoveElevator = false;}),
                Commands.runOnce(() -> RobotSuperState.getInstance().updatePossession(
                    algaeIntakeWristevatorStates.contains(RobotSuperState.getInstance().getWristevatorState())
                            ? GrabberPossession.ALGAE
                            : GrabberPossession.EMPTY)
                )
            );
        }

        /**
         * Returns a command that stops grabber wheels from spinning
         */
        public Command stopGrabber(){
            return Commands.either(
                superstructure.applyGrabberState(GrabberState.ALGAE_HOLD), 
                superstructure.applyGrabberState(GrabberState.IDLE),
                () -> (RobotSuperState.getInstance().getPossession() == GrabberPossession.ALGAE)
            );
        }

        /**
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return Commands.either(
                superstructure.applyWristevatorState(WristevatorState.ALGAE_TRAVEL),
                superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                () -> (RobotSuperState.getInstance().getPossession() == GrabberPossession.ALGAE)
            );
        }

        /**
         * Call this on button releases after scoring game pieces, stops grabber movement and retracts mechanisms to travel state
         */
        public Command stopAndRetract(){
            return Commands.parallel(
                stopGrabber(),
                retractMechanisms()
            );
        }

        public Command stopAndAlgaeIntake(){
            return Commands.parallel(
                stopGrabber(),
                Commands.either(
                    Commands.either(
                        superstructure.applyWristevatorState(WristevatorState.HIGH_INTAKE, Units.inchesToMeters(6)),
                        superstructure.applyWristevatorState(WristevatorState.LOW_INTAKE, Units.inchesToMeters(6)),
                        () -> Localization.getClosestReefFace(m_drive.getPose()).algaeHigh == true
                    ),
                    Commands.either(
                        superstructure.applyWristevatorState(WristevatorState.HIGH_INTAKE),
                        superstructure.applyWristevatorState(WristevatorState.LOW_INTAKE),
                        () -> Localization.getClosestReefFace(m_drive.getPose()).algaeHigh == true
                    ),
                    () -> m_elevator.getPositionMeters() > 0.25 //HIGH TRAVEL is 0.3m
                )
            );
        }

        /**
         * Call this on button releases after scoring game pieces, stops grabber movement and retracts GRABBER ONLY
         */
        public Command stopAndRetractGrabber(){
            return Commands.parallel(
                stopGrabber(),
                wristFlickUp()
            );
        }

        public Command wristFlickUp() {
            return m_wrist.applyAngle(coralTravelAngle);
        }

        public Command removeAlgae(){
            return superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE);
        }

        /**
         * Set elevator and wrist to L1 preset
         */
        public Command preL1(){
            return  
                Commands.either(
                    superstructure.applyWristevatorStateDirect(WristevatorState.L1),
                    superstructure.applyWristevatorState(WristevatorState.L1),
                    () -> RobotSuperState.getInstance().getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L2, Units.inchesToMeters(6)), //2 * 0.181368595),
                    superstructure.applyWristevatorState(WristevatorState.L2, Units.inchesToMeters(6)), //2 * 0.181368595),
                    () -> RobotSuperState.getInstance().getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L3, Units.inchesToMeters(6)),//, 2 * 0.181368595),
                    superstructure.applyWristevatorState(WristevatorState.L3, Units.inchesToMeters(6)),//, 2 * 0.181368595),
                    () -> RobotSuperState.getInstance().getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L4, Units.inchesToMeters(12)),//, 2 * 0.181368595),
                    superstructure.applyWristevatorState(WristevatorState.L4, Units.inchesToMeters(12)), //2 * 0.181368595),
                    () -> RobotSuperState.getInstance().getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyWristevatorStateNoPremove(WristevatorState.NET, Units.inchesToMeters(0.5),false);
        }

        public Command preAutomaticNet(){
            return superstructure.m_wrist.applyAngle(Rotation2d.fromDegrees(70));
        }

        /**
         * Set elevator and wrist to processor preset
         */
        public Command preProcessor(){
            return superstructure.applyWristevatorState(WristevatorState.PROCESSOR);
        }

        /**
         * Set elevator and wrist to ground algae preset
         */
        public Command groundAlgaeIntake(){
            return superstructure.applyWristevatorState(WristevatorState.GROUND_INTAKE);
        }

        /**
         * Set elevator and wrist to ground algae preset
         */
        public Command lollipopAlgaeIntake(){
            return superstructure.applyWristevatorState(WristevatorState.LOLLIPOP_INTAKE);
        }

        /**
         * Set elevator and wrist to low algae preset
         */
        public Command lowAlgaeIntake(){
            return superstructure.applyWristevatorState(WristevatorState.LOW_INTAKE);  
        }

        /**
         * Set elevator and wrist to high algae preset
         */
        public Command highAlgaeIntake(){
            return superstructure.applyWristevatorState(WristevatorState.HIGH_INTAKE);
        }

        /** 
         * Transfers a coral from the indexer to the grabber, without checking for position 
         */
        private Command transferCoral() {
            return Commands.sequence(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    superstructure.holdIndexState(IndexState.TRANSFER)
                ).until(m_grabberCANRange), // Wait until the coral starts to exit the funnel
                // Commands.waitUntil(m_grabber::hasFunnelCurrentSpike),
                // Commands.waitUntil(m_transferBeamBreak.negate().debounce(0.06)), // W ait until the coral fully exits the funnel
                superstructure.m_grabber.applyRotationsBangBang(6, 1.6), // Adjust rotations
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.IDLE),
                    superstructure.applyIndexState(IndexState.BACKWARDS)
                )
            );
        }

        /**
         * Bring the wristevator down to its travel state (wrist up, elevator all the way down),
         * brings the coral from the funnel into the indexer,
         * waits for a signal that it is safe to transfer the coral to the grabber,
         * rotates the wrist down,
         * and the rotates the motors to bring the coral into the grabber.
         * @param safeSignal a supplier indicating whether or not is safe to transfer the coral from the indexer to the grabber
         * @return a command sequence
         */
        public Command intakeCoral() { // (BooleanSupplier safeSignal)
            return Commands.sequence(
                superstructure.applyWristevatorStateDirect(WristevatorState.CORAL_TRANSFER),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    transferCoral()
                ),
                Commands.parallel(
                    //superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                    superstructure.applyWristevatorStateDirect(WristevatorState.HIGH_TRAVEL),
                    Commands.run(() -> safeToMoveElevator = true)
                ).unless(elevatorClutchTrigger)
            );
        }
/* 
        private Command completeTransfer() {
            return Commands.sequence(
                superstructure.holdIndexState(IndexState.TRANSFER).until(m_transferBeamBreak.negate().debounce(0.06)),
                superstructure.m_grabber.applyRotationsBangBang(12, 1.4), // TODO: Adjust rotations
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.IDLE),
                    superstructure.applyIndexState(IndexState.BACKWARDS)
                )
            );
        }
*/
        /**
         * Moves the elevator and wrist to the intake position, intakes the coral,
         * moves it out 0.2 rotations to avoid hitting the carriage,
         * and the moves the elevator and wrist to the travel position
         * @return a command sequence
         */
        public Command grabberIntakeCoral() {
            return Commands.sequence(
                superstructure.applyWristevatorState(WristevatorState.GRABBER_CORAL_INTAKE, Units.inchesToMeters(8)),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    Commands.sequence(
                        applyGrabberState(GrabberState.GRABBER_CORAL_INTAKE),
                        Commands.waitUntil(m_grabberCANRange),
                        m_grabber.applyRotationsBangBang(8, -0.37),
                        Commands.runOnce(() -> RobotSuperState.getInstance().updatePossession(GrabberPossession.CORAL))
                    )
                ),
                Commands.parallel(
                    superstructure.applyWristevatorStateDirect(WristevatorState.TRAVEL),
                    Commands.run(() -> safeToMoveElevator = true)
                )
            );
            
            /* Current sensing
            return Commands.sequence(
                superstructure.applyWristevatorState(WristevatorState.GRABBER_CORAL_INTAKE),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    Commands.sequence(
                        applyGrabberState(GrabberState.GRABBER_CORAL_INTAKE),
                        Commands.waitSeconds(0.2),
                        Commands.waitUntil(m_grabber::hasCoral),
                        m_grabber.applyRotationsBangBang(8, 0.2)
                    )
                ),
                Commands.parallel(
                    superstructure.applyWristevatorStateDirect(WristevatorState.TRAVEL),
                    Commands.run(() -> safeToMoveElevator = true)
                )
            );
            */
        }

        /**
         * Moves the elevator and wrist to the intake position directly and intakes the coral
         * 
         * <p> This command is different because it applies the wristevator state directly,
         * doesn't adjust the coral,
         * and doesn't return to the travel position afterwards
         * @return a command sequence
         */
        public Command autonGrabberIntakeCoral() {
            return Commands.sequence(
                superstructure.applyWristevatorStateDirect(WristevatorState.GRABBER_CORAL_INTAKE),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    Commands.sequence(
                        applyGrabberState(GrabberState.GRABBER_CORAL_INTAKE),
                        Commands.waitUntil(m_grabberCANRange)
                        // Commands.waitSeconds(0.2),
                        // Commands.waitUntil(m_grabber::hasCoral)
                        // m_grabber.applyRotationsBangBang(8, 0.2) moved to autonGrabberAdjustCoral instead to save time
                    )
                )   
                // Commands.run(() -> safeToMoveElevator = true)
            );
        }

        /**
         * This is scheduled after autonGrabberIntakeCoral to avoid the grabber hitting the carriage
         * @return
         */
        public Command autonGrabberAdjustCoral() {
            return Commands.sequence(
                m_grabber.applyRotationsBangBang(8,  -0.37), // (8, 0.2)
                Commands.runOnce(() -> safeToMoveElevator = true)
            );
        }

        /**
         * Stops grabber, and holds the indexer in a backwards state
         * @return
         */
        public Command stopIntake() {
            return Commands.parallel(
                    holdIndexState(IndexState.BACKWARDS),
                    applyGrabberState(GrabberState.IDLE)
                );
        }
        
        public Command interruptElevator() {
            return superstructure.interruptElevator();
        }

        public Command interruptWrist() {
            return superstructure.interruptWrist();
        }
        

        public Command preIntakeCoral() {
            return superstructure.applyWristevatorStateDirect(WristevatorState.CORAL_TRANSFER);
        }

        public Command autonIntakeCoral() {
            return Commands.sequence(
                // superstructure.applyWristevatorState(WristevatorState.CORAL_TRANSFER),
                transferCoral(),
                Commands.runOnce(() -> safeToMoveElevator = true)
            );
        }

        public Command preL4Direct() {
            return superstructure.applyWristevatorStateDirect(WristevatorState.L4);
        }

        public Command holdAlgae() {
            return Commands.parallel(
                Commands.runOnce(() -> RobotSuperState.getInstance().updatePossession(GrabberPossession.ALGAE)),
                superstructure.applyGrabberState(GrabberState.ALGAE_HOLD));
        }

        public Command autonShoot() {
            return m_grabber.applyRotationsBangBang(12, 2);
        }

        public Command autonAlgaeIntake() {
            return Commands.parallel(
                Commands.runOnce(() -> RobotSuperState.getInstance().updatePossession(GrabberPossession.ALGAE)),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
            );
        }

        @AutoLogOutput()
        public boolean getTransferBeambreak(){
            return m_transferBeamBreak.getAsBoolean();
        }

        @AutoLogOutput()
        public boolean getGrabberCANRange() {
            return m_grabberCANRange.getAsBoolean();
        }
    }
}
