// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.coralTravelAngle;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.DaemonCommand;
import lib.extendedcommands.SelectWithFallbackCommand;
import lib.extendedcommands.SelectWithFallbackCommandFactory;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

/**
 * Superstructure class that contains all subsystems and commands for the robot's superstructure <p>
 * Allows all of the subsystems to talk to each other <p>
 * Allows sensors to interact with subystems <p>
 * Contains command factories for actions requiring multiple systems <p>
 */
public class Superstructure {

    /** A mutable (you can change the state after instantiation) class representing the Superstructure's desired state */
    public static class MutableSuperState {

        protected GrabberState grabberState;
        protected WristevatorState wristevatorState;
        protected IndexState indexState;
        protected GrabberPossession grabberPossession;

        public MutableSuperState(GrabberState grabberState, WristevatorState wristevatorState){
            this.grabberState = grabberState;
            this.wristevatorState = wristevatorState;
            grabberPossession = GrabberPossession.EMPTY;
        }

        public MutableSuperState() {
            this.grabberState = GrabberState.IDLE;
            this.wristevatorState = WristevatorState.TRAVEL;
            grabberPossession = GrabberPossession.EMPTY;
        }
        
        /** Set state of the wristevator <p>
         * States include elevator heights and wrist angles corresponding to specific actions
         */
        public void setWristevatorState(WristevatorState position) {
            this.wristevatorState = position;
        }

        /** Set state of the grabber. <p>
         * States include voltage to run grabber wheels at for different actions (intake, outtake, idle)
         */
        public void setGrabberState(GrabberState state) {
            this.grabberState = state;
        }

        public void setIndexerState(IndexState state) {
            this.indexState = state;
        }

        public void setGrabberPossession(GrabberPossession possession) {
            this.grabberPossession = possession;
        }

        public GrabberState getGrabberState() {
            return grabberState;
        }

        public WristevatorState getWristevatorState() {
            return wristevatorState;
        }

        public IndexState getIndexerState() {
            return indexState;
        }

        public GrabberPossession getGrabberPossession(){
            return grabberPossession;
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private final Elastic m_elastic;
    private final Trigger elevatorClutchTrigger;

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final MutableSuperStateAutoLogged superState = new MutableSuperStateAutoLogged();

    private Boolean safeToFeedCoral;
    private Boolean safeToMoveElevator;

    private boolean elevatorClutch = false;

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        Elastic elastic,
        BooleanSupplier indexBeamBreak, // REMOVE
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier grabberBeamBreak // REMOVE
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        m_elastic = elastic;
        
        m_elastic.updateTransferBeamBreak(transferBeamBreak.getAsBoolean());

        CommandUtils.makePeriodic(() -> {
            Logger.processInputs("Superstructure", superState);
        });
        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, grabberBeamBreak);
        elevatorClutchTrigger = new Trigger(this::elevatorClutchSignal);

        this.safeToFeedCoral = false;
        this.safeToMoveElevator = false;
    }

    public MutableSuperState getSuperState() {
        return this.superState;
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
    private Command applyWristevatorState(WristevatorState position) {

        Command wristPreMoveCommand = Commands.either(
            m_wrist.applyAngle(algaeTravelAngle),
            m_wrist.applyAngle(coralTravelAngle),
            () -> superState.getGrabberPossession() == GrabberPossession.ALGAE
        );

        Command wristHoldCommand = Commands.either(
            m_wrist.holdAngle(algaeTravelAngle), 
            m_wrist.holdAngle(coralTravelAngle),
            () -> superState.getGrabberPossession() == GrabberPossession.ALGAE
        );
        
        return //Commands.either(
            //applyWristevatorStateDirect(position),
            Commands.sequence(
                new ProxyCommand(Commands.runOnce(() -> superState.setWristevatorState(position))
                    .alongWith(Commands.runOnce(() -> {
                        safeToFeedCoral = false;
                        safeToMoveElevator = false;
                    }))),
                new ProxyCommand(wristPreMoveCommand),
                new ProxyCommand(Commands.deadline(
                    m_elevator.applyPosition(position.elevatorHeightMeters),
                    wristHoldCommand
                )),
                new ProxyCommand(new DaemonCommand(
                    () -> Commands.run(() -> m_elevator.setPosition(position.elevatorHeightMeters), m_elevator),
                    () -> false
                )),
                new ProxyCommand(m_wrist.applyAngle(position.wristAngle)),
                new ProxyCommand(new DaemonCommand(
                    () -> Commands.run(() -> m_wrist.setAngle(position.wristAngle), m_wrist),
                    () -> false)));
            //() -> superState.getWristevatorState() == position);

        /* Old code, we're not sure why it doesn't work
        Command wristPreMoveCommand = Commands.either(
            m_wrist.applyAngle(algaeTravelAngle),
            m_wrist.applyAngle(coralTravelAngle),
            () -> superState.getGrabberPossession() == GrabberPossession.ALGAE
        );

        
        // We use parallel commands here to reduce the number of loops that instant commands use
        return Commands.sequence(
            // Sets the wristevatorState instantly for purpose of logging
            new ProxyCommand(Commands.runOnce(() -> superState.setWristevatorState(position))) 
            // Folds the wrist in to avoid hitting obstacles, at an angle depending on grabber possession
            .alongWith(new ProxyCommand(wristPreMoveCommand)),
            // Raises the elevator until the elevator at the correct height and holds the wrist in place in the background
            new ProxyCommand(m_elevator.applyPosition(position.elevatorHeightMeters))
            .alongWith(new ProxyCommand(new DaemonCommand(
                () -> Commands.run(() -> m_wrist.setAngle(
                    superState.getGrabberPossession() == GrabberPossession.ALGAE
                        ? algaeTravelAngle
                        : coralTravelAngle)), 
                () -> false))),
            // Moves to the next command instantly, but holds the elevator in place in the background
            new ProxyCommand(new DaemonCommand(
                () -> Commands.run(() -> m_elevator.setPosition(position.elevatorHeightMeters), m_elevator),
                () -> false
            ))
            // Moves the wrist to the final position
            .alongWith(new ProxyCommand(m_wrist.applyAngle(position.wristAngle))),
            // Moves to the next command instantly, but holds the wrist in place in the background
            new ProxyCommand(new DaemonCommand(
                () -> Commands.run(() -> m_wrist.setAngle(position.wristAngle), m_wrist),
                () -> false
            ))
            // apply clutch trigger state found in WristevatorState
            .alongWith(Commands.runOnce(() -> elevatorClutch = position.elevatorClutch)
        ));
        */

        // Even older code
        /*
        return Commands.sequence(
            //m_elevator.applyPosition(position.elevatorHeightMeters),
            
            new DaemonCommand(
                () -> Commands.run(() -> m_elevator.setPosition(position.elevatorHeightMeters), m_elevator),
                () -> false
            )
            m_wrist.applyAngle(Rotation2d.fromDegrees(80)),
            m_wrist.applyAngle(Rotation2d.fromDegrees(-30))
        );*/
    }

    /**
     * Applies a wristevator state directly without any premoves. Potentially dangerous if used when right up against the
     * reef, because the grabber could hit the branches
     * @param position
     * @return
     */
    private Command applyWristevatorStateDirect(WristevatorState position) {
        safeToFeedCoral = false;
        safeToMoveElevator = false;
        
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> superState.setWristevatorState(position)),
                m_elevator.applyPosition(position.elevatorHeightMeters),
                m_wrist.applyAngle(position.wristAngle)),
            Commands.parallel(
                new DaemonCommand(
                    () -> Commands.run(() -> m_elevator.setPosition(position.elevatorHeightMeters), m_elevator),
                    () -> false),
                new DaemonCommand(
                    () -> Commands.run(() -> m_wrist.setAngle(position.wristAngle), m_wrist),
                    () -> false)
            )
        );
    }

    /**
     * Sets grabber wheels to run at desired state
     * @param state the GrabberState
     * @return command to run grabber as certain state
     */
    public Command applyGrabberState(GrabberState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command grabberRadiansBangBang(double volts, double rotations) {
        return m_grabber.applyRadiansBangBang(volts, rotations);
    }

    public Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
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
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.applyVolts(state.volts),
            Commands.idle(m_index)//.onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    public Command interruptWrist(){
        return Commands.runOnce(() -> {
            if(m_wrist.getCurrentCommand() != null) m_wrist.getCurrentCommand().cancel();
        });
        
    }

    public Command interruptElevator(){
        return Commands.runOnce(() -> {
            if(m_elevator.getCurrentCommand() != null) m_elevator.getCurrentCommand().cancel();
        });
    }

    /**
     * Updates game piece possession based on beambreaks and updates kG accordingly
     * (gamepieces have weight that affects elevator and wrist)
     * @param indexBB whether the beambreak sensor in the indexer detects something
     * @param transferBB whether the beambreak sensor between the indexer and grabber detects something
     * @param greabberBB whether the beambreak sensor in the grabber detects something
     */
    public void updatePossessionAndKg(
        boolean indexBB,
        boolean transferBB,
        boolean grabberBB
    ) {
        GrabberPossession grabberPossession;
        
        if (transferBB && grabberBB) {
            grabberPossession = GrabberPossession.CORAL;
        } else if (transferBB) {
            grabberPossession = GrabberPossession.TRANSFERRING;
        } else if (grabberBB) {
            grabberPossession = GrabberPossession.ALGAE;
        } else {
            grabberPossession = GrabberPossession.EMPTY;
        }

        m_elevator.setKg(grabberPossession.elevator_kG);
        m_wrist.setKg(grabberPossession.wrist_kG);
        superState.setGrabberPossession(grabberPossession);
        
        m_elastic.putGrabberPossession(grabberPossession);
        m_elastic.putBoolean("transferBB", transferBB);

        // For debugging the beambreaks
        //System.out.println("updatePossessionAndKg() Called");
        //System.out.println("transferBB: " + transferBB);
    }

    /** Contains all the command factories for the superstructure */
    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final Map<WristevatorState, Supplier<Command>> grabberActionCommands = new HashMap<WristevatorState, Supplier<Command>>(); // We use a map of grabber action commands so that we can use the SelectWithFallBackCommand factory
        private final SelectWithFallbackCommandFactory<WristevatorState> grabberActionCommandFactory;
        private final Set<WristevatorState> algaeIntakeWristevatorStates = Set.of(WristevatorState.GROUND_INTAKE, WristevatorState.LOW_INTAKE, WristevatorState.HIGH_INTAKE); // Wristevator states that lead to intaking algae

        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier indexBeamBreak,
            BooleanSupplier transferBeamBreak,
            BooleanSupplier grabberBeamBreak
        ) {
            this.superstructure = superstructure;
            m_indexBeamBreak = indexBeamBreak;
            m_transferBeamBreak = transferBeamBreak;
            grabberActionCommands.put(WristevatorState.L1, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L2, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L3, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); 
            grabberActionCommands.put(WristevatorState.L4, () -> superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.GROUND_INTAKE,
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
                    this.superstructure.getSuperState()::getWristevatorState
                );

            this.updatePossessionAndKg();
        }

        /**
         * Returns a command that does a grabber action differently depending on robot state (scoring coral, scoring algae, intaking algae)
         * and sets the grabber possession depending on the wristevator state
         */
        public Command doGrabberAction() {
            return Commands.parallel(
                grabberActionCommandFactory.buildCommand(),
                Commands.runOnce(() -> {safeToMoveElevator = false;}),
                Commands.runOnce(() -> superstructure.getSuperState().setGrabberPossession(
                    algaeIntakeWristevatorStates.contains(superstructure.getSuperState().getWristevatorState())
                            ? GrabberPossession.ALGAE
                            : GrabberPossession.EMPTY)
                )
            );
        }

        /**
         * Returns a command that stops grabber wheels from spinning
         */
        public Command stopGrabber(){
            return superstructure.applyGrabberState(GrabberState.IDLE);
        }

        /**
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return Commands.either(
                superstructure.applyWristevatorState(WristevatorState.ALGAE_TRAVEL),
                superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                () -> superstructure.superState.getGrabberPossession() == GrabberPossession.ALGAE
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
            return m_wrist.applyAngle(coralTravelAngle);
        }

        public Command removeAlgae(){
            return superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE);
        }

        /**
         * Set elevator and wrist to L1 preset
         */
        public Command preL1(){
            return superstructure.applyWristevatorState(WristevatorState.L1);
        }

        /**
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return superstructure.applyWristevatorState(WristevatorState.L2);
        }

        /**
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return superstructure.applyWristevatorState(WristevatorState.L3);
        }

        /**
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return superstructure.applyWristevatorState(WristevatorState.L4);
        }

        /**
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyWristevatorState(WristevatorState.NET);
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
            /*return Commands.parallel(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    Commands.waitUntil(m_indexBeamBreak),
                    superstructure.grabberRadiansBangBang(4, kIndexVoltage),
                    superstructure.applyGrabberState(GrabberState.IDLE)
                ),
                Commands.sequence(
                    superstructure.holdIndexState(IndexState.CORAL_TRANSFER),
                    Commands.waitUntil(() -> !m_indexBeamBreak.getAsBoolean()),
                    superstructure.holdIndexState(IndexState.EMPTY_IDLE)
                )
            );*/
            return Commands.sequence(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    superstructure.holdIndexState(IndexState.TRANSFER)
                )
                .until(m_transferBeamBreak), // Wait until the coral starts to exit the funnel
                Commands.waitSeconds(0.3),
                Commands.waitUntil(m_transferBeamBreak),
                Commands.waitUntil(() -> !m_transferBeamBreak.getAsBoolean()), // Wait until the coral fully exits the funnel
                superstructure.m_grabber.applyRotationsBangBang(12, 1.7), // Adjust rotations
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.IDLE),
                    superstructure.applyIndexState(IndexState.BACKWARDS)
                )
            );
            
            //.onlyIf(() -> superstructure.getSuperState().getGrabberPossession() == GrabberPossession.EMPTY);
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
                superstructure.applyWristevatorState(WristevatorState.CORAL_TRANSFER),
                Commands.parallel(
                    Commands.runOnce(() -> safeToFeedCoral = true),
                    transferCoral()
                ),
                Commands.parallel(
                    superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                    Commands.run(() -> safeToMoveElevator = true)
                )
            );
        }

        public Command stopIntake() {
            return 
                Commands.parallel(
                    holdIndexState(IndexState.BACKWARDS),
                    applyGrabberState(GrabberState.IDLE)
                );
        }
        
        /*
        public Command interruptWristevator() {
            return Commands.parallel(
                superstructure.interruptElevator(),
                superstructure.interruptWrist()
            );
        }*/

        
        public Command interruptElevator() {
            return superstructure.interruptElevator();
        }

        public Command interruptWrist() {
            return superstructure.interruptWrist();
        }
        

        public Command preIntakeCoral() {
            return superstructure.applyWristevatorState(WristevatorState.TRAVEL);
        }

        public Command autonIntakeCoral() {
            return Commands.sequence(
                superstructure.applyWristevatorState(WristevatorState.CORAL_TRANSFER),
                transferCoral(),
                Commands.runOnce(() -> safeToMoveElevator = true)
            );
        }

        public Command holdAlgae() {
            return Commands.parallel(
                Commands.runOnce(() -> superstructure.superState.setGrabberPossession(GrabberPossession.ALGAE)),
                superstructure.applyGrabberState(GrabberState.ALGAE_HOLD));
        }

        public Command autonShoot() {
            return m_grabber.applyRotationsBangBang(12, 3);
        }

        public Command autonAlgaeIntakeAndHold() {
            return Commands.sequence(
                m_grabber.applyRotationsBangBang(-12, 3),
                holdAlgae()
            );
        }

        /**
         * Used to calculate what the robot is possessing based on breambreaks
         */
        public Command updatePossessionAndKg() {
            return Commands.runOnce(
                () -> superstructure.updatePossessionAndKg(
                    m_indexBeamBreak.getAsBoolean(),
                    m_transferBeamBreak.getAsBoolean(),
                    false
                )
            );
        }

    }
}
