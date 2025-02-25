// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.SuperstructureConstants.algaeTravelAngle;
import static frc.robot.Constants.SuperstructureConstants.coralTravelAngle;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.drive.DriveSubsystem.DriveCommandFactory;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import static frc.robot.Constants.IndexConstants.kIndexVoltage;

import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.DaemonCommand;
import lib.extendedcommands.SelectWithFallbackCommand;

import java.util.function.BooleanSupplier;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;

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
    @AutoLog
    public static class MutableSuperState {

        protected GrabberState grabberState;
        protected WristevatorState WristevatorState;
        protected IndexState indexState;
        protected GrabberPossession grabberPossession;
        protected IndexPossession indexPossession;

        public MutableSuperState(GrabberState grabberState, WristevatorState WristevatorState, IndexState indexState){
            this.grabberState = grabberState;
            this.WristevatorState = WristevatorState;
            this.indexState = indexState;
            grabberPossession = GrabberPossession.EMPTY;
            indexPossession = IndexPossession.EMPTY;
        }

        public MutableSuperState() {}
        
        /** Set state of the wristevator <p>
         * States include elevator heights and wrist angles corresponding to specific actions
         */
        public void setWristevatorState(WristevatorState position) {
            this.WristevatorState = position;
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

        public void setIndexPossession(IndexPossession possession) {
            this.indexPossession = possession;
        }

        public GrabberState getGrabberState() {
            return grabberState;
        }

        public WristevatorState getWristevatorState() {
            return WristevatorState;
        }

        public IndexState getIndexerState() {
            return indexState;
        }

        public GrabberPossession getGrabberPossession(){
            return grabberPossession;
        }

        public IndexPossession getIndexerPossession(){
            return indexPossession;
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;
    private final Elastic m_elastic;
    private final Trigger elevatorClutchTrigger;
    private final LEDSubsystem m_led;

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final MutableSuperStateAutoLogged superState = new MutableSuperStateAutoLogged();

    private boolean elevatorClutch = false;

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        Elastic elastic,
        LEDSubsystem led,
        BooleanSupplier indexBeamBreak, //returns true when beam broken
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier grabberBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        m_elastic = elastic;
        m_led = led;
        
        m_elastic.putBoolean("grabberBB", indexBeamBreak.getAsBoolean());
        m_elastic.putBoolean("transferBB", transferBeamBreak.getAsBoolean());
        m_elastic.putBoolean("indexBB", grabberBeamBreak.getAsBoolean());

        CommandUtils.makePeriodic(() -> {
            Logger.processInputs("Superstructure", superState);
        });
        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, grabberBeamBreak);
        elevatorClutchTrigger = new Trigger(this::elevatorClutchSignal);
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
     * Sets grabber wheels to run at desired state
     * @param state the GrabberState
     * @return command to run grabber as certain state
     */
    private Command applyGrabberState(GrabberState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    //TODO: Refactor this to be more in-line with the rest of the superstructure code
    private Command grabberRotationsBangBang(double volts, double rotations) {
        return m_grabber.applyRotationsBangBang(volts,rotations);
    }

    /**
     * Sets indexer to run at desired state
     * @param state the IndexState
     * @return command to run index as certain state
     */
    private Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.applyVolts(state.volts)//.onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command interruptWrist(){
        return Commands.runOnce(() -> m_wrist.getCurrentCommand().cancel(), m_wrist);
        
    }

    private Command interruptElevator(){
        return Commands.runOnce(() -> m_elevator.getCurrentCommand().cancel(), m_elevator);
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
        IndexPossession indexPossession = indexBB 
            ? IndexPossession.CORAL 
            : IndexPossession.EMPTY;
        GrabberPossession grabberPossession;
        // TODO: as coral is shot, the robot will think we have an algae, and kG will increase. Add timeout?
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
        superState.setIndexPossession(indexPossession);
        superState.setGrabberPossession(grabberPossession);
        
        m_elastic.putIndexPossession(indexPossession);
        m_elastic.putGrabberPossession(grabberPossession);
        m_elastic.putBoolean("grabberBB", grabberBB);
        m_elastic.putBoolean("transferBB", transferBB);
        m_elastic.putBoolean("indexBB", indexBB);

        // For debugging the beambreaks
        //System.out.println("updatePossessionAndKg() Called");
        //System.out.println("grabberBB: " + grabberBB);
        //System.out.println("transferBB: " + transferBB);
        //System.out.println("indexBB: " + indexBB);

        // TODO: call add LED code here

    }

    /** Contains all the command factories for the superstructure */
    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_grabberBeamBreak;
        private final Command grabberActionSelectCommand;
        private final Map<WristevatorState, Command> grabberActionCommands = new HashMap<>(); // We use a map of grabber action commands so that we can use the SelectWithFallBackCommand factory
        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier indexBeamBreak,
            BooleanSupplier transferBeamBreak,
            BooleanSupplier grabberBeamBreak
        ) {
            this.superstructure = superstructure;
            m_indexBeamBreak = indexBeamBreak;
            m_transferBeamBreak = transferBeamBreak;
            m_grabberBeamBreak = grabberBeamBreak;
            grabberActionCommands.put(WristevatorState.L1, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L2, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            grabberActionCommands.put(WristevatorState.L3, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); 
            grabberActionCommands.put(WristevatorState.L4, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); // TODO: Are there any risks if we have a coral, try to score it, and then hit the button again? The grabber will think it has an algae
            grabberActionCommands.put(WristevatorState.GROUND_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.PROCESSOR, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            grabberActionCommands.put(WristevatorState.LOW_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.HIGH_INTAKE,
                                        superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                                        /* .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE)*/);
            grabberActionCommands.put(WristevatorState.NET, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));

            grabberActionSelectCommand = new SelectWithFallbackCommand<WristevatorState>(
                grabberActionCommands,
                superstructure.applyGrabberState(GrabberState.DEFAULT_OUTTAKE), // Default command to do if command can't be chosen from grabberActionCommands
                this.superstructure.getSuperState()::getWristevatorState
            );

            this.updatePossessionAndKg();
        }

        /**
         * Returns a command that does a grabber action differently depending on robot state (scoring coral, scoring algae, intaking algae)
         */
        public Command doGrabberAction() {
            return grabberActionSelectCommand;
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
            return superstructure.applyWristevatorState(WristevatorState.TRAVEL);
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
         * Transfers a coral from the funnel to the indexer 
         */
        public Command indexCoral() {
            return Commands.sequence(
                superstructure.applyIndexState(IndexState.CORAL_INTAKE),
                Commands.waitUntil(m_indexBeamBreak),
                superstructure.applyIndexState(IndexState.CORAL_IDLE),
                Commands.run(() -> {})
            );
        }

        /** 
         * Transfers a coral from the indexer to the grabber, without checking for position 
         */
        private Command transferCoral() {
            /*return Commands.parallel(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    Commands.waitUntil(m_indexBeamBreak),
                    superstructure.grabberRotationsBangBang(4, kIndexVoltage),
                    superstructure.applyGrabberState(GrabberState.IDLE)
                ),
                Commands.sequence(
                    superstructure.applyIndexState(IndexState.CORAL_TRANSFER),
                    Commands.waitUntil(() -> !m_indexBeamBreak.getAsBoolean()),
                    superstructure.applyIndexState(IndexState.EMPTY_IDLE)
                )
            );*/
            return Commands.sequence(
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    superstructure.applyIndexState(IndexState.CORAL_TRANSFER)
                ),
                Commands.waitUntil(m_indexBeamBreak),
                superstructure.m_grabber.applyRotationsBangBang(4,4*Math.PI),
                Commands.parallel(
                    superstructure.applyGrabberState(GrabberState.IDLE),
                    superstructure.applyIndexState(IndexState.EMPTY_IDLE)
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
        public Command intakeCoral(){ // (BooleanSupplier safeSignal)
            return Commands.sequence(
                /* 
                superstructure.applyWristevatorState(WristevatorState.TRAVEL),
                indexCoral(),
                Commands.waitUntil(safeSignal),
                */
                superstructure.applyWristevatorState(WristevatorState.CORAL_TRANSFER),
                transferCoral()//,
                //indexCoral()
            );
        }

        public Command stopIntake(){
            return 
                Commands.parallel(
                    applyIndexState(IndexState.CORAL_IDLE),
                    applyGrabberState(GrabberState.IDLE)
                );
        }

        public Command interruptElevator(){
            return superstructure.interruptElevator();
        }

        public Command interruptWrist(){
            return superstructure.interruptWrist();
        }

        public Command holdAlgae(){
            return superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE);
        }

        //TODO: Write separate intake coral command for auton

        /**
         * Used to calculate what the robot is possessing based on breambreaks
         */
        public Command updatePossessionAndKg(){
            return Commands.runOnce(
                () -> superstructure.updatePossessionAndKg(
                    m_indexBeamBreak.getAsBoolean(),
                    m_transferBeamBreak.getAsBoolean(),
                    m_grabberBeamBreak.getAsBoolean()
                )
            );
        }
    }
}
