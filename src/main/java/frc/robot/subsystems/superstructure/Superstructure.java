// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GrabberConstants.kCoralEffectorVoltage;
import static frc.robot.Constants.GrabberConstants.kAlgaeHoldingVoltage;
import static frc.robot.Constants.ElevatorConstants.Control.kMaxVel;
import static frc.robot.subsystems.superstructure.state.Possession.*;

import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.grabber.Grabber;
import frc.robot.subsystems.superstructure.state.SuperState;
import frc.robot.subsystems.superstructure.wrist.Wrist;

import java.util.Map;
import lib.extendedcommands.SelectWithFallbackCommandFactory;

import java.util.HashMap;
import java.util.HashSet;
import lib.math.Combinatorics;

public class Superstructure extends SubsystemBase {
    
    private static final double algaeTravelRadians = Units.degreesToRadians(65);
    private static final double coralTravelRadians = Units.degreesToRadians(-90);
    private static final double emptyTravelRadians = Units.degreesToRadians(90);

    private static final double coralTravelHeight = 0.25;
    // State machine static loading

    private static record Edge (
        SuperState start,
        SuperState end
    ) {}

    private static class EdgeSet extends HashSet<Edge> {
        public EdgeSet(Edge... edges) {
            for (var edge : edges) {
                super.add(edge);
            }
        }
    }

    private static final Set<Edge> scoringEdgeSet = new EdgeSet(
        new Edge(SuperState.ALIGN_L1, SuperState.SCORE_L1),
        new Edge(SuperState.ALIGN_L2, SuperState.SCORE_L2),
        new Edge(SuperState.ALIGN_L3, SuperState.SCORE_L3),
        new Edge(SuperState.ALIGN_L4, SuperState.SCORE_L4),
        new Edge(SuperState.PRE_NET, SuperState.NET_SCORE),
        new Edge(SuperState.PRE_PROCESSOR, SuperState.PROCESSOR_SCORE)
    );

    private static final Set<SuperState> coralTraversalStates = new HashSet<>();
    private static final Set<Edge> coralTraversalEdges = new HashSet<>();

    static {
        coralTraversalStates.add(SuperState.CORAL_TRAVEL);
        coralTraversalStates.add(SuperState.ALIGN_L1);
        coralTraversalStates.add(SuperState.ALIGN_L2);
        coralTraversalStates.add(SuperState.ALIGN_L3);
        coralTraversalStates.add(SuperState.ALIGN_L4);
        for (var edgeTuple : Combinatorics.permuteTwo(coralTraversalStates)) {
            coralTraversalEdges.add(new Edge(edgeTuple.get(0),edgeTuple.get(1)));
        }
    }

    private static final EdgeSet algaeIntakeEdges = new EdgeSet(
        new Edge(SuperState.PRE_GROUND_INTAKE,SuperState.GROUND_INTAKE),
        new Edge(SuperState.PRE_LOW_INTAKE,SuperState.LOW_INTAKE),
        new Edge(SuperState.PRE_HIGH_INTAKE,SuperState.HIGH_INTAKE)
    );

    // State Machine Runtime Loading

    private final Map<Edge,Command> edgeCommandMap = new HashMap<>();
    private final Set<Edge> forbiddenEdges = new HashSet<>();

    private Command edgeCommand = Commands.idle();


    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Grabber m_grabber;
    private final Funnel m_funnel;

    private SuperState goalState;
    private SuperState currentState = SuperState.EMPTY_TRAVEL;
    private SuperState cachedState; // Stores the previous state for tempApplyState commands, or the end state for applyTempEnd commands. Also stores the previous state during overrides
    private PossessionState possession = PossessionState.EMPTY;
    private final BooleanSupplier m_transferBeamBreak;
    private final DoubleSupplier grabberTravelVoltSupplier = () -> possession == PossessionState.ALGAE
        ? kAlgaeHoldingVoltage
        : 0.0;
    
    private final Map<PossessionState,Supplier<Command>> premoveCommandSupplierMap = new HashMap<>();
    private final SelectWithFallbackCommandFactory<PossessionState> premoveCommandFactory;
    
    public final SuperstructureCommandFactory commandBuilder = new SuperstructureCommandFactory();

    public Superstructure(
        Elevator elevator,
        Wrist wrist,
        Grabber grabber,
        Funnel funnel,
        BooleanSupplier grabberBeamBreak
    ) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_grabber = grabber;
        m_funnel = funnel;
        m_transferBeamBreak = grabberBeamBreak;

        premoveCommandSupplierMap.put(PossessionState.EMPTY,() -> applyWristAngle(emptyTravelRadians));
        premoveCommandSupplierMap.put(PossessionState.CORAL,() -> Commands.sequence(
            applyElevatorHeight(coralTravelHeight).onlyIf(() -> m_elevator.getPositionMeters() < (coralTravelHeight-0.01)),
            applyWristAngle(coralTravelRadians)
        )); // A small buffer is added to coralTravelHeight for safety
        premoveCommandSupplierMap.put(PossessionState.ALGAE, () -> applyWristAngle(algaeTravelRadians));
        premoveCommandFactory = new SelectWithFallbackCommandFactory<>(premoveCommandSupplierMap, Commands::none, () -> possession);

        // Initialize scoring edge commands
        for (Edge edge : scoringEdgeSet) {
            buildEdgeCommand(edge);
        }
        // Initialize coral traversal edge commands
        for (Edge edge : coralTraversalEdges) {
            buildEdgeCommand(edge);
        }

        // Initialize algae intake edge commands and userland commands
        for (Edge edge : algaeIntakeEdges) {
            buildEdgeCommand(edge);
        }
    }

    @Override
    public void periodic() {
        m_elevator.periodic();
        m_wrist.periodic();
        m_grabber.periodic();
        m_funnel.periodic();
    }

    @Override
    public void simulationPeriodic() {
        m_elevator.simulationPeriodic();
        m_wrist.simulationPeriodic();
        // TODO: ADD WPILIB SIMMING TO GRABBER AND FUNNEL SIM IO
    }

    private Command buildEdgeCommand(Edge edge) {

        if (edge.end() == SuperState.OVERRIDE) {
            edgeCommandMap.put(edge,Commands.none());
            return Commands.none();
        }
            
        if (edge.start() == edge.end()) {
            edgeCommandMap.put(edge,Commands.none());
            return Commands.none();
        }

        if (edge.start() == SuperState.OVERRIDE) {
            var edgeCommand = Commands.sequence(
                Commands.runOnce(() -> updateGoalState(edge.end())),
                applyWristevatorStateSafe(
                    edge.end().height.meters,
                    edge.end().angle.radians
                ),
                Commands.parallel(
                    Commands.runOnce(() -> m_funnel.setVoltage(edge.end().funnelState.voltage)),
                    Commands.runOnce(() -> m_grabber.runVoltsDifferential(
                        edge.end().grabberState.leftVolts,
                        edge.end().grabberState.rightVolts
                    )),
                    Commands.runOnce(() -> updateCurrentState(edge.end()))
                )
            );
            edgeCommandMap.put(edge, edgeCommand);
            return edgeCommand;
        }

        Command wristevatorCommand;
        // determines wristevator command
        if (edge.start().height != edge.end().height) {
            wristevatorCommand = applyWristevatorStateSafe(
                edge.end().height.meters,
                edge.end().angle.radians
            );
        } else if (edge.start().angle != edge.end().angle) {
            wristevatorCommand = applyWristAngle(edge.end().angle.radians);
        } else {
            wristevatorCommand = Commands.none();
        }

        var edgeCommand = Commands.sequence(
            Commands.runOnce(() -> updateGoalState(edge.end())),
            wristevatorCommand,
            Commands.parallel(
                Commands.runOnce(() -> m_funnel.setVoltage(edge.end().funnelState.voltage)),
                Commands.runOnce(() -> m_grabber.runVoltsDifferential(
                    edge.end().grabberState.leftVolts,
                    edge.end().grabberState.rightVolts
                )),
                Commands.runOnce(() -> updateCurrentState(edge.end()))
            )
        );

        edgeCommandMap.put(edge,edgeCommand);
        return edgeCommand;
    }

    private Command getEdgeCommand(Edge edge) {
        return edgeCommandMap.getOrDefault(edge, buildEdgeCommand(edge));
    }

    private void setGoal(SuperState newGoal) {

        // Checks if we are already moving towards the goal
        if (newGoal == goalState) {
            return;
        }

        var edge = new Edge(currentState,newGoal);

        // Checks if edge is forbidden
        if (forbiddenEdges.contains(edge)) {
            return; // Edge is forbidden, do nothing
        }

        if (edge.end().possessionMap.map(possession) == PossessionState.FORBIDDEN) {
            // transition would result in an illegal or impossible possession state, do nothing
            return;
        }

        // Edge is valid and we are not already on it, schedule the corresponding edge command
        edgeCommand.cancel();
        edgeCommand = getEdgeCommand(edge);
        edgeCommand.schedule();
    }


    public double getElevatorHeightMeters() {
        return m_elevator.getPositionMeters();
    }

    public Rotation2d getWristAngle() {
        return m_wrist.getAngle();
    }

    //NOTE: All of these commands are instant
    //WARNING: These commands are for internal superstructure use ONLY, as they do not require the superstructure

    private Command applyElevatorHeight(double positionMeters) {
        return Commands.runOnce(() -> m_elevator.setPosition(positionMeters)).andThen(
            Commands.idle().until(m_elevator::atPositionSetpoint)
        );
    }

    private Command applyElevatorState(TrapezoidProfile.State state) {
        return Commands.runOnce(() -> m_elevator.setState(state)).andThen(
            Commands.idle().until(m_elevator::atPositionSetpoint)
        );
    }

    private Command applyElevatorStateImmediate(TrapezoidProfile.State state) {
        return Commands.runOnce(() -> m_elevator.setState(state));
    }

    private Command applyElevatorHeightImmediate(double positionMeters) {
        return Commands.runOnce(() -> m_elevator.setPosition(positionMeters));
    }
    
    private Command applyWristAngle(double wristAngleRadians) {
        return Commands.runOnce(() -> m_wrist.setAngleRadians(wristAngleRadians)).andThen(
            Commands.idle().until(m_wrist::atPositionSetpoint) 
        );
    }

    private Command applyWristAngleImmediate(double wristAngleRadians) {
        return Commands.runOnce(() -> m_wrist.setAngleRadians(wristAngleRadians));
    }

    private Command applyWristevatorStateSafe(double elevatorHeightMeters, double wristAngleRadians) {

        return Commands.sequence(
            applyGrabberVolts(grabberTravelVoltSupplier.getAsDouble()),
            premoveCommandFactory.buildCommand(),
            applyElevatorHeight(elevatorHeightMeters),
            applyWristAngle(wristAngleRadians)
        );
        
    }

    private Command applyGrabberVolts(double leftVoltsDifferential, double rightVoltsDifferential) {
        return Commands.runOnce(() -> m_grabber.runVoltsDifferential(leftVoltsDifferential, rightVoltsDifferential));
    }

    private Command applyGrabberVolts(double volts) {
        return Commands.runOnce(() -> m_grabber.runVolts(volts));
    }

    private Command applyFunnelVolts(double funnelVolts) {
        return Commands.runOnce(() -> m_funnel.setVoltage(funnelVolts));
    }

    private Command applyGrabberFunnelVolts(double grabberVolts, double funnelVolts) {
        return Commands.parallel(
            Commands.runOnce(() -> m_grabber.runVolts(grabberVolts)),
            Commands.runOnce(() -> m_funnel.setVoltage(funnelVolts))
        );
    }

    private Superstructure getSuperstructure() {
        return this;
    }

    public SuperState getCurrentState() {
        return currentState;
    }

    public SuperState getGoalState() {
        return goalState;
    }

    public boolean atGoal() {
        return goalState.equals(currentState);
    }

    private void updateCurrentState(SuperState newState) {
        currentState = newState;
        possession = currentState.possessionMap.map(possession);
        Logger.recordOutput("Superstructure/Possession",possession.name());
        Logger.recordOutput("Superstructure/State",currentState.name());
    }

    private void updateGoalState(SuperState newGoal) {
        goalState = newGoal;
        Logger.recordOutput("Superstructure/Goal",newGoal.name());
    }

    private void cacheCurrentState() {
        cachedState = currentState;
    }

    private void cacheState(SuperState state) {
        cachedState = state;
    }

    private SuperState getStateCache() {
        return cachedState;
    }

    private Command applyGrabberRotationsBangBang(double volts, double rotations) {
        double sign = Math.signum(rotations - m_grabber.getRotations());
        return Commands.sequence(
            runOnce(() -> m_grabber.runVolts(volts)),
            Commands.waitUntil(() -> Math.signum(rotations - m_grabber.getRotations()) != sign),
            runOnce(() -> m_grabber.stop())
        );
    }

    public class SuperstructureCommandFactory {

        private final Map<SuperState,Supplier<Command>> grabberActionCommandSupplierMap = new HashMap<>();
        private final Map<SuperState,Supplier<Command>> overrideGrabberActionCommandSupplierMap = new HashMap<>();

        private final SelectWithFallbackCommandFactory<SuperState> grabberActionCommandFactory;
        private final SelectWithFallbackCommandFactory<SuperState> overrideGrabberActionCommandFactory;

        //overrideGrabberActionCommandFactory is so we can have a command that applies only the grabber state, without performing the rest of the state transition
        //This is to replicate the behavior of the grabberActionCommand from the original codebase
        //When the state is overridden, the superstructure's current state is cached. Grabber action commands during overrides are determined by the cached state

        private SuperstructureCommandFactory() {
            // Initialize scoring edge userland commands
            for (Edge edge : scoringEdgeSet) {
                grabberActionCommandSupplierMap.put(edge.start(),() -> applyState(edge.end()).finallyDo(() -> setGoal(SuperState.EMPTY_TRAVEL)));
                overrideGrabberActionCommandSupplierMap.put(edge.start(),() -> Commands.startEnd(
                    () -> applyGrabberVolts(
                        edge.end().grabberState.leftVolts,
                        edge.end().grabberState.rightVolts
                    ),
                    () -> applyGrabberVolts(grabberTravelVoltSupplier.getAsDouble())
                ));
            }

            // Initialize algae intake edge commands and userland commands
            for (Edge edge : algaeIntakeEdges) {
                grabberActionCommandSupplierMap.put(edge.start(),() -> applyState(edge.end()).finallyDo(() -> setGoal(SuperState.ALGAE_TRAVEL)));
                overrideGrabberActionCommandSupplierMap.put(edge.start(),() -> Commands.startEnd(
                    () -> applyGrabberVolts(
                        edge.end().grabberState.leftVolts,
                        edge.end().grabberState.rightVolts
                    ),
                    () -> applyGrabberVolts(grabberTravelVoltSupplier.getAsDouble())
                ));
            }
            overrideGrabberActionCommandFactory = new SelectWithFallbackCommandFactory<>(overrideGrabberActionCommandSupplierMap, Commands::none, () -> cachedState);

            grabberActionCommandSupplierMap.put(SuperState.OVERRIDE,overrideGrabberActionCommandFactory::buildCommand);
            
            grabberActionCommandFactory = new SelectWithFallbackCommandFactory<>(grabberActionCommandSupplierMap, Commands::none, () -> currentState);
        }
        
        public Command applyState(SuperState newGoal){
            return Commands.runOnce(() -> setGoal(newGoal))
                .andThen(Commands.idle(getSuperstructure()));
        }

        public Command autoCoralIntake() {
            return Commands.sequence(
                applyState(SuperState.CORAL_INTAKE).until(getSuperstructure()::atGoal),
                Commands.waitUntil(m_transferBeamBreak), 
                Commands.waitUntil(() -> !m_transferBeamBreak.getAsBoolean()),//Makes sure the coral has fully passed the transfer beambreak before activating the Bang-Bang controller
                applyGrabberRotationsBangBang(kCoralEffectorVoltage,2), // TODO: Tune these constants
                applyState(SuperState.CORAL_TRAVEL) // Moves wristivator into the travel position
            );
        }

        public Command doGrabberAction() {
            return Commands.either(
                grabberActionCommandFactory.buildCommand(),
                Commands.none(),
                getSuperstructure()::atGoal
            );
        }

        public Command manualWristevatorControl(DoubleSupplier elevatorVoltSupplier, DoubleSupplier wristVoltSupplier) {
            return Commands.startRun(
                () -> {
                    edgeCommand.cancel();
                    if (!(currentState == SuperState.OVERRIDE)) {
                        cacheCurrentState(); // This makes sure we don't cache the override state if two override commands are scheduled consecutively
                    }
                    updateCurrentState(SuperState.OVERRIDE);
                    updateGoalState(SuperState.OVERRIDE); // TODO: Make goalstate an optional
                },
                () -> {
                    m_elevator.setVoltage(elevatorVoltSupplier.getAsDouble());
                    m_wrist.setVoltage(wristVoltSupplier.getAsDouble());
                },
                getSuperstructure()
            );
        }

        public Command preL1() {
            return applyState(SuperState.ALIGN_L1);
        }

        public Command preL2() {
            return applyState(SuperState.ALIGN_L2);
        }

        public Command preL3() {
            return applyState(SuperState.ALIGN_L3);
        }

        public Command preL4() {
            return applyState(SuperState.ALIGN_L4);
        }

        public Command preGroundIntake() {
            return applyState(SuperState.PRE_GROUND_INTAKE);
        }

        public Command preLowIntake() {
            return applyState(SuperState.PRE_LOW_INTAKE);
        }

        public Command preHighIntake() {
            return applyState(SuperState.PRE_HIGH_INTAKE);
        }

        public Command preProcessor() {
            return applyState(SuperState.PRE_PROCESSOR);
        }

        public Command preNet() {
            return applyState(SuperState.PRE_NET); // TODO: WRITE LOGIC FOR NET SCORING
        }

        public Command scoreCoralAuton() {
            return applyGrabberRotationsBangBang(12,3);
        }

    }

}