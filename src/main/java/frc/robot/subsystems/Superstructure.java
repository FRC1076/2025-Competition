package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants.SuperstructureConstants.GrabberPosition;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.extendedcommands.CommandUtils;
import lib.extendedcommands.SelectWithFallbackCommand;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.wrist.SetWristAngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.IndexConstants.kIndexVoltage;

public class Superstructure {

    // A mutable class representing the Superstructure's state
    @AutoLog
    public static class SuperState {

        protected GrabberState grabberState;
        protected GrabberPosition grabberPosition;
        protected IndexState indexState;
        protected GrabberPossession grabberPossession;
        protected IndexPossession indexPossession;

        public SuperState(GrabberState grabberState, GrabberPosition grabberPosition, IndexState indexState){
            this.grabberState = grabberState;
            this.grabberPosition = grabberPosition;
            this.indexState = indexState;
            grabberPossession = GrabberPossession.EMPTY;
            indexPossession = IndexPossession.EMPTY;
        }

        public SuperState() {}

        //Calculates game piece possession from beambreaks
        public void calculatePossession(boolean indexBeamBreak, boolean transferBeamBreak, boolean grabberBeamBreak){
            indexPossession = indexBeamBreak 
                ? IndexPossession.CORAL 
                : IndexPossession.EMPTY;
            grabberPossession = grabberBeamBreak 
                ? (transferBeamBreak 
                    ? GrabberPossession.CORAL 
                    : GrabberPossession.ALGAE) 
                : GrabberPossession.EMPTY;
        }

        public void setGrabberPosition(GrabberPosition position) {
            this.grabberPosition = position;
        }

        public void setGrabberState(GrabberState state) {
            this.grabberState = state;
        }

        public void setIndexerState(IndexState state) {
            this.indexState = state;
        }

        public GrabberState getGrabberState() {
            return grabberState;
        }

        public GrabberPosition getGrabberPosition() {
            return grabberPosition;
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

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final SuperStateAutoLogged superState = new SuperStateAutoLogged();

    public Superstructure (
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        BooleanSupplier indexBeamBreak, //returns true when beam broken
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier grabberBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;
        
        CommandUtils.makePeriodic(() -> {
            Logger.processInputs("Superstructure",superState);
        });
        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, grabberBeamBreak);
    }

    public SuperState getSuperState() {
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

    /**
     * Folds back wrist, moves elevator, then deploys wrist
     * @param position the GrabberPosition, which consists of elevator height and wrist angle, to transition to
     * @return generic transition command from one state to another 
     */
    private Command applyGrabberPosition(GrabberPosition position) {
        
        Command wristPreMoveCommand = Commands.either(
            m_wrist.applyAngle(Rotation2d.fromDegrees(65)),
            m_wrist.applyAngle(Rotation2d.kCCW_90deg),
            () -> superState.getGrabberPossession() == GrabberPossession.ALGAE
        );
        
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberPosition(position)),
            wristPreMoveCommand,
            m_elevator.applyPosition(position.elevatorHeightMeters),
            m_wrist.applyAngle(position.wristAngle)
        );
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

    /**
     * Sets indexer to run at desired state
     * @param state the IndexState
     * @return command to run index as certain state
     */
    private Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.applyVolts(kIndexVoltage).onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_grabberBeamBreak;
        private final Map<GrabberPosition, Command> scoringCommands = new HashMap<>();
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
            scoringCommands.put(GrabberPosition.L1, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L2, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L3, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L4, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.NET, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            scoringCommands.put(GrabberPosition.PROCESSOR, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));

        }

        /**
         * Score game piece differently depending on robot state
         */
        public Command scoreGamePiece() {
            return new SelectWithFallbackCommand<GrabberPosition>(
                scoringCommands,
                superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE), //Default command to do if command can't be chosen from scoringCommands
                this.superstructure.getSuperState()::getGrabberPosition
            );
        }

        /**
         * Stop grabber wheels from spinning
         */
        public Command stopGrabber(){
            return superstructure.applyGrabberState(GrabberState.IDLE);
        }

        /**
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return superstructure.applyGrabberPosition(GrabberPosition.TRAVEL);
        }

        /**
         * Call this on button releases, stops grabber movement and retracts mechanisms to travel state
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
            return superstructure.applyGrabberPosition(GrabberPosition.L1);
        }

        /**
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return superstructure.applyGrabberPosition(GrabberPosition.L2);
        }

        /**
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return superstructure.applyGrabberPosition(GrabberPosition.L3);
        }

        /**
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return superstructure.applyGrabberPosition(GrabberPosition.L4);
        }

        /**
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyGrabberPosition(GrabberPosition.NET);
        }

        /**
         * Set elevator and wrist to processor preset
         */
        public Command preProcessor(){
            return superstructure.applyGrabberPosition(GrabberPosition.PROCESSOR);
        }

        /**
         * Set elevator and wrist to ground algae preset while running grabber intake if open
         */
        public Command groundAlgaeIntake(){
            return 
            Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.GROUND_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );
        }

        /**
         * Set elevator and wrist to low algae preset while running grabber intake if open
         */
        public Command lowAlgaeIntake(){
            return 
            Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.LOW_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );   
        }

        /**
         * Set elevator and wrist to high algae preset while running grabber intake if open
         */
        public Command highAlgaeIntake(){
            return Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.HIGH_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );
            
        }

        /** 
         * Transfers a coral from the funnel to the indexer 
         */
        private Command indexCoral() {
            return Commands.sequence(
                superstructure.applyIndexState(IndexState.CORAL_INTAKE),
                Commands.waitUntil(m_indexBeamBreak),
                superstructure.applyIndexState(IndexState.CORAL_IDLE)
            );
        }

        /** 
         * Transfers a coral from the indexer to the grabber, without checking for position 
         */
        private Command transferCoral() {
            return Commands.parallel(
                Commands.sequence(
                    superstructure.applyGrabberState(GrabberState.CORAL_INTAKE),
                    Commands.waitUntil(m_grabberBeamBreak),
                    superstructure.applyGrabberState(GrabberState.IDLE)
                ),
                Commands.sequence(
                    superstructure.applyIndexState(IndexState.CORAL_TRANSFER),
                    Commands.waitUntil(() -> !m_indexBeamBreak.getAsBoolean()),
                    superstructure.applyIndexState(IndexState.EMPTY_IDLE)
                )
            ).onlyIf(() -> superstructure.getSuperState().getGrabberPossession() == GrabberPossession.EMPTY);
        }

        public Command intakeCoral(BooleanSupplier safeSignal){
            return Commands.sequence(
                superstructure.applyGrabberPosition(GrabberPosition.TRAVEL),
                indexCoral(),
                Commands.waitUntil(safeSignal),
                superstructure.applyGrabberPosition(GrabberPosition.CORAL_TRANSFER),
                transferCoral()
            );
        }

        /**
         * Used to calculate what the robot is possessing based on breambreaks
         */
        public Command calculatePossession(){
            return Commands.runOnce(
                () -> this.superstructure.superState.calculatePossession(
                    m_indexBeamBreak.getAsBoolean(),
                    m_transferBeamBreak.getAsBoolean(),
                    m_grabberBeamBreak.getAsBoolean()
                )
            );
        }
    }
}
