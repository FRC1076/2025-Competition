// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.SuperstructureConstants.algaeNetReleaseHeightMeters;
import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.coralBranchStateSet;
import static frc.robot.Constants.SuperstructureConstants.emptyTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.highTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.coralTravelAngle;

import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotSuperState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants.PoseOfInterest;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.utils.Localization;
import frc.robot.utils.VirtualSubsystem;
import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.SelectWithFallbackCommandFactory;
import lib.functional.FunctionalUtils;
import lib.functional.NegatableBooleanSupplier;
import lib.utils.Combinatorics;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.security.Policy;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure class that contains all subsystems and commands for the robot's superstructure <p>
 * Allows all of the subsystems to talk to each other <p>
 * Allows sensors to interact with subystems <p>
 * Contains command factories for actions requiring multiple systems <p>
 */
public class Superstructure extends VirtualSubsystem {

    private static record WristevatorEdge(WristevatorState begin, WristevatorState end) {}
    private static final double branchUpSafeDeployOffsetMeters = 0.2; // The height above a branch where it is safe to deploy wrist TODO: TUNE THIS
    private static final double branchDownSafeDeployOffsetMeters = 0.05; // The height below a branch where it is safe to deploy wrist, TODO: TUNE THIS
    private static final double branchToL1SafeDeployHeightMeters = 0.5; // The height where it is safe to deploy the wrist when travelling from a branch to L1, TODO: TUNE THIS
    
    private static Set<WristevatorEdge> coralBranchEdgeSet = new HashSet<>();
    private static Set<WristevatorEdge> branchToL1EdgeSet = new HashSet<>();
    static {
        for (List<WristevatorState> edgeList : Combinatorics.permuteTwo(coralBranchStateSet)){
            coralBranchEdgeSet.add(new WristevatorEdge(edgeList.get(0), edgeList.get(1)));
        }
        for (WristevatorState branchState : coralBranchStateSet) {
            branchToL1EdgeSet.add(new WristevatorEdge(branchState,WristevatorState.L1));
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private final Elastic m_elastic;
    private final RobotSuperState m_state = RobotSuperState.getInstance();
    private final Trigger elevatorClutchTrigger;

    public final SuperstructureCommandFactory CommandBuilder;
    private final NegatableBooleanSupplier possessAlgae = () -> m_state.getPossession() == GrabberPossession.ALGAE;
    private final Map<WristevatorEdge,Command> edgeCommandMap = new HashMap<>();
    private final Command stickyControlCommand; // A command to be scheduled when the wristevator is under sticky control
    private boolean stickyControl = false;


    private Boolean safeToFeedCoral;
    private Boolean safeToMoveElevator;

    private boolean elevatorClutch = false;

    private double stickyControlHeight = 0; // The goal height of the elevator under sticky control
    private Rotation2d stickyControlAngle = new Rotation2d(); // the goal height of the wrist under sticky control

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        Elastic elastic,
        BooleanSupplier transferBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        m_elastic = elastic;
        
        m_elastic.updateTransferBeamBreak(transferBeamBreak.getAsBoolean());

        CommandBuilder = new SuperstructureCommandFactory(this, transferBeamBreak);
        elevatorClutchTrigger = new Trigger(this::elevatorClutchSignal);

        this.safeToFeedCoral = false;
        this.safeToMoveElevator = false;
        this.stickyControlCommand = Commands.run(
            () -> {
                m_elevator.setPosition(stickyControlHeight);
                m_wrist.setAngle(stickyControlAngle);
            },
            m_wrist,m_elevator
        );
    }

    @Override
    public void periodic() {
        m_state.updateElevatorHeight(m_elevator.getPositionMeters());
        m_state.updateWristAngle(m_wrist.getAngle());
        m_state.logSuperstructureToAkit();
        if (!stickyControl){
            stickyControlHeight = m_elevator.getPositionMeters();
            stickyControlAngle = m_wrist.getAngle();
        }
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

    private void resetWristevatorControllers() {
        m_elevator.resetController();
        m_wrist.resetController();
    }

    public Trigger elevatorClutchTrigger() {
        return elevatorClutchTrigger;
    }

    // Returns a command whose sole purpose is to require the elevator and wrist
    private Command requireWristevator() {
        return Commands.runOnce(() -> {},m_wrist,m_elevator);
    }

    private Command enableStickyControl(){
        return Commands.runOnce(() -> {
            if (!stickyControlCommand.isScheduled()){
                stickyControlHeight = m_elevator.getPositionMeters();
                stickyControlAngle = m_wrist.getAngle(); 
                // The angle and height are set to their current measurements to ensure a smooth transition into sticky control
                stickyControlCommand.schedule();
            }
        });
    }

    // Holds a certain angle on the wrist in the background
    // WARNING: FOR INTERNAL USE ONLY
    private Command applyStickyAngle(Rotation2d angle){
        return Commands.runOnce(() -> {
            m_wrist.resetController();
            stickyControlAngle = angle;
        }).andThen(Commands.waitUntil(
            () -> Math.abs(angle.minus(m_wrist.getAngle()).getRadians()) < WristConstants.wristAngleToleranceRadians
        ));
    }

    // Holds a certain angle on the elevator in the background when sticky controls are enabled
    // WARNING: FOR INTERNAL USE ONLY
    private Command applyStickyHeight(double heightMeters){
        return Commands.runOnce(() -> {
            m_elevator.resetController();
            stickyControlHeight = heightMeters;
        }).andThen(Commands.waitUntil(
            () -> Math.abs(heightMeters - m_elevator.getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters
        ));
    }

    private Command updateWristevatorGoal(WristevatorState goal){
        return Commands.runOnce(() -> m_state.updateWristevatorGoal(goal));
    }

    private Command updateWristevatorState(WristevatorState state){
        return Commands.runOnce(() -> m_state.updateWristevatorState(state));
    }

    // Returns a daemon command that will hold the wristevator in its current state in the background
    private Command holdWristevatorState(WristevatorState state){
        return Commands.parallel(
            CommandUtils.makeDaemon(m_wrist.holdAngle(state.wristAngle)),
            CommandUtils.makeDaemon(m_elevator.holdPosition(state.elevatorHeightMeters))
        );
    }

    // Command factories that apply states are private because they are only accessed by the main SuperStructureCommandFactory

    // an edgeCommand that does nothing up update the state telemetry
    private Command nopEdgeCommand(WristevatorEdge edge){
        return Commands.sequence(
            updateWristevatorState(edge.end()),
            updateWristevatorGoal(edge.end())
        );
    }

    // Constructs a new wristevator edge command, and stores it in the edgeCommandMap
    private Command buildEdgeCommand(WristevatorEdge edge){
        /*
        Command ledSignal = Commands.runOnce(() -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        });
        */
        Command edgeCommand = Commands.none();
        if (edge.end() == WristevatorState.OVERRIDE) {
            edgeCommand = nopEdgeCommand(edge); // TODO: CHECK IF OVERRIDE EDGES ARE NEEDED
        } else if (edge.begin() == edge.end()) {
            edgeCommand = nopEdgeCommand(edge);
        } else if (coralBranchEdgeSet.contains(edge)){
            // If the wristevator is moving between two branch states, travel with the grabber down
            double safeDeployHeight;
            BooleanSupplier safeToDeployWrist;
            if (edge.begin().elevatorHeightMeters > edge.end().elevatorHeightMeters){
                // If the wristevator is moving down between two coral states, begin deploying wrist slightly above elevator setpoint
                safeDeployHeight = edge.end().elevatorHeightMeters + branchUpSafeDeployOffsetMeters;
                safeToDeployWrist = () -> m_elevator.getPositionMeters() <= safeDeployHeight;
            } else {
                safeDeployHeight = edge.end().elevatorHeightMeters - branchDownSafeDeployOffsetMeters;
                safeToDeployWrist = () -> m_elevator.getPositionMeters() >= safeDeployHeight;
            }
            edgeCommand = coralBranchEdgeCommand(edge, safeToDeployWrist);
        } else if (branchToL1EdgeSet.contains(edge)){
            // If the wristevator is moving from a branch to L1, travel with the grabber up
            return branchToL1EdgeCommand(edge);
        }
        edgeCommandMap.put(edge,edgeCommand);
        return edgeCommand;
    }

    private Command coralBranchEdgeCommand(WristevatorEdge edge, BooleanSupplier safeToDeployWrist){
        return Commands.sequence(
            enableStickyControl(),
            updateWristevatorGoal(edge.end()),
            Commands.either(
                applyStickyAngle(coralTravelAngle),
                applyStickyAngle(algaeTravelAngle),
                possessAlgae
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(safeToDeployWrist),
                    applyStickyAngle(edge.end().wristAngle)
                ),
                applyStickyHeight(edge.end().elevatorHeightMeters)
            ),
            updateWristevatorState(edge.end())
        );
    }

    // A generic wristevator edge command that makes no assumptions about the robot's state
    private Command genericEdgeCommand(WristevatorEdge edge){
        return Commands.sequence(
            enableStickyControl(),
            updateWristevatorGoal(edge.end()),
            applyStickyAngle(highTravelAngle),
            applyStickyHeight(edge.end().elevatorHeightMeters),
            applyStickyAngle(edge.end().wristAngle),
            updateWristevatorState(edge.end())
        );
    }

    // A direct wristevator edge command, for auton
    private Command directEdgeCommand(WristevatorEdge edge){
        return Commands.sequence(
            enableStickyControl(),
            updateWristevatorGoal(edge.end()),
            Commands.parallel(
                applyStickyAngle(edge.end().wristAngle),
                applyStickyHeight(edge.end().elevatorHeightMeters)
            ),
            updateWristevatorState(edge.end())
        );
    }

    private Command branchToL1EdgeCommand(WristevatorEdge edge){
        return Commands.sequence(
            enableStickyControl(),
            updateWristevatorGoal(edge.end()),
            Commands.either(
                applyStickyAngle(highTravelAngle),
                applyStickyAngle(algaeTravelAngle),
                possessAlgae
            ),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(() -> m_elevator.getPositionMeters() <= branchToL1SafeDeployHeightMeters),
                    applyStickyAngle(edge.end().wristAngle)
                ),
                applyStickyHeight(edge.end().elevatorHeightMeters)
            ),
            updateWristevatorState(edge.end())
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
    private Command applyWristevatorStateLegacy(WristevatorState position) {

        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };

        Command wristPreMoveCommand = Commands.either(
            m_wrist.applyAngle(algaeTravelAngle),
            m_wrist.applyAngle(coralTravelAngle),
            possessAlgae
        );

        Command wristHoldCommand = Commands.either(
            m_wrist.holdAngle(algaeTravelAngle), 
            m_wrist.holdAngle(coralTravelAngle),
            possessAlgae
        );

        // Due to command composition semantics, the command composition itself cannot require the subsystems directly
        
        return Commands.sequence(
            Commands.runOnce(() -> m_state.updateWristevatorGoal(position)),
            wristPreMoveCommand.asProxy(),
            Commands.deadline(
                m_elevator.applyPosition(position.elevatorHeightMeters),
                wristHoldCommand
            ).asProxy(),
            CommandUtils.makeDaemon(m_elevator.holdPosition(position.elevatorHeightMeters)),
            m_wrist.applyAngle(position.wristAngle).asProxy(),
            CommandUtils.makeDaemon(m_wrist.holdAngle(position.wristAngle)),
            Commands.runOnce(() -> m_state.updateWristevatorState(position))
        ).alongWith(
            Commands.runOnce(ledSignal)
        );
    }

    /**
     * Applies a wristevator state directly without any premoves. Potentially dangerous if used when right up against the
     * reef, because the grabber could hit the branches
     * @param position
     * @return
     */
    private Command applyWristevatorStateDirect(WristevatorState position) {

        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };
        
        return Commands.parallel(
            Commands.runOnce(() -> m_state.updateWristevatorGoal(position)),
            Commands.runOnce(ledSignal),
            Commands.sequence(
                m_elevator.applyPosition(position.elevatorHeightMeters).asProxy(),
                CommandUtils.makeDaemon(m_elevator.holdPosition(position.elevatorHeightMeters))
            ),
            Commands.sequence(
                m_wrist.applyAngle(position.wristAngle).asProxy(),
                CommandUtils.makeDaemon(m_wrist.holdAngle(position.wristAngle))
            )
        ).andThen(Commands.runOnce(() -> m_state.updateWristevatorState(position)));
    }

    private Command applyWristevatorStateGrabberDown(WristevatorState position) {

        Runnable ledSignal = () -> {
            safeToFeedCoral = false;
            safeToMoveElevator = false;
        };

        Command wristPreMoveCommand = m_wrist.applyAngle(Rotation2d.fromDegrees(-90));

        Command wristHoldCommand = m_wrist.holdAngle(Rotation2d.fromDegrees(-90));

        // Due to command composition semantics, the command composition itself cannot require the subsystems directly
        
        return Commands.sequence(
            Commands.runOnce(() -> m_state.updateWristevatorGoal(position)),
            wristPreMoveCommand.asProxy(),
            Commands.deadline(
                m_elevator.applyPosition(position.elevatorHeightMeters),
                wristHoldCommand
            ).asProxy(),
            CommandUtils.makeDaemon(m_elevator.holdPosition(position.elevatorHeightMeters)),
            m_wrist.applyAngle(position.wristAngle).asProxy(),
            CommandUtils.makeDaemon(m_wrist.holdAngle(position.wristAngle)),
            Commands.runOnce(() -> m_state.updateWristevatorState(position))
        ).alongWith(
            Commands.runOnce(ledSignal)
        );
    }

    /**
     * Sets grabber wheels to run at desired state
     * @param state the GrabberState
     * @return command to run grabber as certain state
     */
    public Command applyGrabberState(GrabberState state) {
        return Commands.parallel(
            Commands.runOnce(() -> m_state.updateGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command grabberRadiansBangBang(double volts, double rotations) {
        return m_grabber.applyRadiansBangBang(volts, rotations);
    }

    public Command applyIndexState(IndexState state) {
        return Commands.parallel(
            Commands.runOnce(() -> m_state.updateIndexState(state)),
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
            Commands.runOnce(() -> m_state.updateIndexState(state)),
            m_index.applyVolts(state.volts),
            Commands.idle(m_index)//.onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command interruptWrist(){
        return Commands.runOnce(() -> {
            resetWristevatorControllers();
            if(m_wrist.getCurrentCommand() != null) m_wrist.getCurrentCommand().cancel();
        });
        
    }

    private Command interruptElevator(){
        return Commands.runOnce(() -> {
            resetWristevatorControllers();
            if(m_elevator.getCurrentCommand() != null) m_elevator.getCurrentCommand().cancel();
        });
    }

    /** Contains all the command factories for the superstructure */
    public class SuperstructureCommandFactory { 
        
        private final static Set<WristevatorState> algaeIntakeWristevatorStates = Set.of(WristevatorState.GROUND_INTAKE, WristevatorState.LOW_INTAKE, WristevatorState.HIGH_INTAKE); // Wristevator states that lead to intaking algae

        private final Superstructure superstructure;
        private final BooleanSupplier m_transferBeamBreak;
        private final Map<WristevatorState, Supplier<Command>> grabberActionCommands = new HashMap<WristevatorState, Supplier<Command>>(); // We use a map of grabber action commands so that we can use the SelectWithFallBackCommand factory
        private final SelectWithFallbackCommandFactory<WristevatorState> grabberActionCommandFactory;

        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier transferBeamBreak
        ) {
            this.superstructure = superstructure;
            m_transferBeamBreak = transferBeamBreak;
            grabberActionCommands.put(WristevatorState.L1, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L2, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L3, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); 
            grabberActionCommands.put(WristevatorState.L4, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.GROUND_INTAKE, () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE));
            grabberActionCommands.put(WristevatorState.PROCESSOR, () -> superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            grabberActionCommands.put(WristevatorState.LOW_INTAKE, () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE));
            grabberActionCommands.put(WristevatorState.HIGH_INTAKE, () -> superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE));
            grabberActionCommands.put(WristevatorState.NET, () -> superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));

            grabberActionCommandFactory = new SelectWithFallbackCommandFactory<WristevatorState>(
                    grabberActionCommands,
                    () -> superstructure.applyGrabberState(GrabberState.DEFAULT_OUTTAKE),
                    m_state::getWristevatorState
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
                Commands.runOnce(() -> m_state.updatePossession(
                    algaeIntakeWristevatorStates.contains(m_state.getWristevatorState())
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
                superstructure.possessAlgae
            );
        }

        /**
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return Commands.either(
                superstructure.applyWristevatorStateLegacy(WristevatorState.ALGAE_TRAVEL),
                superstructure.applyWristevatorStateLegacy(WristevatorState.TRAVEL),
                superstructure.possessAlgae
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

        public Command wristFlickUp() {
            return m_wrist.applyAngle(highTravelAngle);
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
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L1),
                    superstructure.applyWristevatorStateLegacy(WristevatorState.L1),
                    () -> m_state.getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L2),
                    superstructure.applyWristevatorStateLegacy(WristevatorState.L2),
                    () -> m_state.getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L3),
                    superstructure.applyWristevatorStateLegacy(WristevatorState.L3),
                    () -> m_state.getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return 
                Commands.either(
                    superstructure.applyWristevatorStateGrabberDown(WristevatorState.L4),
                    superstructure.applyWristevatorStateLegacy(WristevatorState.L4),
                    () -> m_state.getWristevatorState() == WristevatorState.HIGH_TRAVEL
                );
        }

        /**
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyWristevatorStateLegacy(WristevatorState.PRE_NET);
        }

        /**
         * Set elevator and wrist to processor preset
         */
        public Command preProcessor(){
            return superstructure.applyWristevatorStateLegacy(WristevatorState.PROCESSOR);
        }

        /**
         * Set elevator and wrist to ground algae preset
         */
        public Command groundAlgaeIntake(){
            return superstructure.applyWristevatorStateLegacy(WristevatorState.GROUND_INTAKE);
        }

        /**
         * Set elevator and wrist to low algae preset
         */
        public Command lowAlgaeIntake(){
            return superstructure.applyWristevatorStateLegacy(WristevatorState.LOW_INTAKE);  
        }

        /**
         * Set elevator and wrist to high algae preset
         */
        public Command highAlgaeIntake(){
            return superstructure.applyWristevatorStateLegacy(WristevatorState.HIGH_INTAKE);
        }

        /** 
         * Transfers a coral from the indexer to the grabber, without checking for position 
         */
        private Command transferCoral() {
            
            return Commands.sequence(
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    superstructure.holdIndexState(IndexState.TRANSFER)
                ).until(FunctionalUtils.debounce(m_transferBeamBreak,0.25)),
                Commands.waitUntil(() -> !m_transferBeamBreak.getAsBoolean()),
                superstructure.m_grabber.applyRotationsBangBang(12, 1.4), // Adjust rotations
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
         * @return a command sequence
         */
        public Command intakeCoral() { // (BooleanSupplier safeSignal)
            return Commands.sequence(
                superstructure.applyWristevatorStateLegacy(WristevatorState.CORAL_TRANSFER),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    transferCoral()
                ),
                Commands.parallel(
                    //superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                    superstructure.applyWristevatorStateDirect(WristevatorState.HIGH_TRAVEL),
                    Commands.run(() -> safeToMoveElevator = true)
                ).onlyIf(() -> !m_transferBeamBreak.getAsBoolean())
            );
        }

        /**
         * Moves the elevator and wrist to the intake position, intakes the coral,
         * moves it out 0.2 rotations to avoid hitting the carriage,
         * and the moves the elevator and wrist to the travel position
         * @return a command sequence
         */
        public Command grabberIntakeCoral() {
            return Commands.sequence(
                superstructure.applyWristevatorStateLegacy(WristevatorState.GRABBER_CORAL_INTAKE),
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
                        Commands.waitSeconds(0.2),
                        Commands.waitUntil(m_grabber::hasCoral)
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
                m_grabber.applyRotationsBangBang(8, 0.2),
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
                Commands.runOnce(() -> m_state.updatePossession(GrabberPossession.ALGAE)),
                superstructure.applyGrabberState(GrabberState.ALGAE_HOLD));
        }

        public Command autonShoot() {
            return m_grabber.applyRotationsBangBang(12, 2);
        }

        public Command scoreNet() {
            return Commands.sequence(
                applyWristevatorStateDirect(WristevatorState.PRE_NET),
                Commands.parallel(
                    applyWristevatorStateDirect(WristevatorState.NET),
                    Commands.sequence(
                        Commands.waitUntil(() -> m_elevator.getPositionMeters() >= algaeNetReleaseHeightMeters),
                        applyGrabberState(GrabberState.ALGAE_OUTTAKE)
                    )
                )
            );
        }

        // Automatically goes to the correct algae intake height based on pose
        public Command goToAlgaeIntake() {
            return Commands.either(
                applyWristevatorStateLegacy(WristevatorState.HIGH_INTAKE),
                applyWristevatorStateLegacy(WristevatorState.LOW_INTAKE),
                Localization::isClosestReefAlgaeHigh
            );
        }

        // Scores net without first moving to pre net
        public Command scoreNetDirect() {
            return Commands.parallel(
                applyWristevatorStateDirect(WristevatorState.NET),
                Commands.sequence(
                    Commands.waitUntil(() -> m_elevator.getPositionMeters() >= algaeNetReleaseHeightMeters),
                    applyGrabberState(GrabberState.ALGAE_OUTTAKE)
                )
            );
        }

        public Command autonAlgaeIntakeAndHold() {
            return Commands.sequence(
                m_grabber.applyRotationsBangBang(-12, 3),
                holdAlgae()
            );
        }

    }
}
