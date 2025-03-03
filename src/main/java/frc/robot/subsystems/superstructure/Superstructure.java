// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.GrabberConstants.kCoralEffectorVoltage;
import static frc.robot.Constants.GrabberConstants.kAlgaeHoldingVoltage;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.grabber.Grabber;
import frc.robot.subsystems.superstructure.state.SuperState;
import frc.robot.subsystems.superstructure.state.SuperState.PossessionState;
import frc.robot.subsystems.superstructure.wrist.Wrist;

import java.util.Map;
import lib.extendedcommands.SelectWithFallbackCommandFactory;

import java.util.HashMap;
import java.util.HashSet;
import lib.math.Combinatorics;

public class Superstructure extends SubsystemBase {
    
    private static final double algaeTravelRadians = Units.degreesToRadians(65);
    private static final double coralTravelRadians = Units.degreesToRadians(-80);
    private static final double emptyTravelRadians = Units.degreesToRadians(80);

    private static final double coralTravelHeight = 0.25;

    private PossessionState possession = PossessionState.EMPTY; //TODO: UPDATE DYNAMICALLY

    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Grabber m_grabber;
    private final Funnel m_funnel;
    private SuperState goalState;
    private SuperState currentState;
    private SuperState cachedState; // Stores the previous state for tempApplyState commands
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
        premoveCommandSupplierMap.put(PossessionState.CORAL,() -> Commands.parallel(
            applyElevatorHeight(coralTravelHeight).onlyIf(() -> m_elevator.getPositionMeters() < 0.25),
            applyWristAngle(coralTravelRadians)
        ));
        premoveCommandSupplierMap.put(PossessionState.ALGAE, () -> applyWristAngle(algaeTravelRadians));
        premoveCommandFactory = new SelectWithFallbackCommandFactory<>(premoveCommandSupplierMap, Commands::none, () -> possession);
    }

    @Override
    public void periodic() {
        m_elevator.periodic();
        m_wrist.periodic();
        m_grabber.periodic();
        m_funnel.periodic();
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
    }

    private void updateGoalState(SuperState newGoal) {
        goalState = newGoal;
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
        
        // STATIC LOADING

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
            new Edge(SuperState.PRE_L1, SuperState.SCORE_L1),
            new Edge(SuperState.PRE_L2, SuperState.SCORE_L2),
            new Edge(SuperState.PRE_L3, SuperState.SCORE_L3),
            new Edge(SuperState.PRE_L4, SuperState.SCORE_L4),
            new Edge(SuperState.PRE_NET, SuperState.NET_SCORE),
            new Edge(SuperState.PRE_PROCESSOR, SuperState.PROCESSOR_SCORE)
        );

        private static final Set<SuperState> coralTraversalStates = new HashSet<>();
        private static final Set<Edge> coralTraversalEdges = new HashSet<>();
        private static final Map<Edge,Edge> grabberActionSequenceMap = new HashMap<>();

        static {
            coralTraversalStates.add(SuperState.CORAL_TRAVEL);
            coralTraversalStates.add(SuperState.PRE_L1);
            coralTraversalStates.add(SuperState.PRE_L2);
            coralTraversalStates.add(SuperState.PRE_L3);
            coralTraversalStates.add(SuperState.PRE_L4);
            for (var edgeTuple : Combinatorics.permuteTwo(coralTraversalStates)) {
                coralTraversalEdges.add(new Edge(edgeTuple.get(0),edgeTuple.get(1)));
            }
        }

        private static final EdgeSet algaeIntakeEdges = new EdgeSet(
            new Edge(SuperState.PRE_GROUND_INTAKE,SuperState.GROUND_INTAKE),
            new Edge(SuperState.PRE_LOW_INTAKE,SuperState.LOW_INTAKE),
            new Edge(SuperState.PRE_HIGH_INTAKE,SuperState.HIGH_INTAKE)
        );
        
        
        private final Map<Edge,Command> edgeCommandMap = new HashMap<>();
        private final Set<Edge> forbiddenEdges = new HashSet<>();
        private final Map<SuperState,Supplier<Command>> scoringCommandSupplierMap = new HashMap<>();
        private final Map<SuperState,Supplier<Command>> algaeIntakeCommandSupplierMap = new HashMap<>();
        private final Map<SuperState,Supplier<Command>> grabberActionCommandSupplierMap = new HashMap<>();

        private final SelectWithFallbackCommandFactory<SuperState> scoreCommandFactory;
        private final SelectWithFallbackCommandFactory<SuperState> algaeIntakeCommandFactory;

        private Command edgeCommand;


        private SuperstructureCommandFactory() {
            // Initialize scoring edge commands
            for (Edge edge : scoringEdgeSet) {
                buildEdgeCommand(edge);
                grabberActionCommandSupplierMap.put(edge.start(),() -> applyTempState(edge.end()));
            }
            // Initialize coral traversal edge commands
            for (Edge edge : coralTraversalEdges) {
                buildEdgeCommand(edge);
            }

            for (Edge edge : algaeIntakeEdges) {
                buildEdgeCommand(edge);
                grabberActionCommandSupplierMap.put(edge.start(),() -> applyTempState(edge.end()))
            }

            // Initialize algae intake edge commands
            for (Edge edge : algaeIntakeEdges) {
                buildEdgeCommand(edge);
                grabberActionCommandSupplierMap.put(edge.start(),() -> applyTempState(edge.end()));
            }

            scoreCommandFactory = new SelectWithFallbackCommandFactory<>(scoringCommandSupplierMap,Commands::none,() -> currentState); //TODO: Should the selector be goalState or currentState?
            algaeIntakeCommandFactory = new SelectWithFallbackCommandFactory<>(algaeIntakeCommandSupplierMap,Commands::none,() -> currentState);

        }
        
        private Command buildEdgeCommand(Edge edge) {
            
            if (edge.start() == edge.end()) {
                return Commands.none();
            }

            if (edge.start() == SuperState.OVERRIDE) {
                return Commands.sequence(
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

            // Edge is valid and we are not already on it, schedule the corresponding edge command
            edgeCommand.cancel();
            edgeCommand = getEdgeCommand(edge);
            edgeCommand.schedule();
        }

        public Command applyState(SuperState newGoal){
            return Commands.runOnce(() -> setGoal(newGoal))
                .andThen(Commands.idle(getSuperstructure()));
        }

        /**
         * Transitions between two states, with a third intermediate state
         * @param tempGoal
         * @param endGoal
         * @return
         */
        public Command applyTempEndState(SuperState tempGoal, SuperState endGoal) {
            return Commands.startEnd(
                () -> {
                    cacheState(endGoal);
                    setGoal(tempGoal);
                },
                () -> setGoal(cachedState),
                getSuperstructure()
            );
        }

        public Command applyTempState(SuperState tempGoal){
            return Commands.startEnd(
                () -> {
                    cacheCurrentState();
                    setGoal(tempGoal);
                },
                () -> setGoal(cachedState),
                getSuperstructure()
            );
        }

        public Command autoCoralIntake() {
            return Commands.sequence(
                applyState(SuperState.CORAL_INTAKE).until(getSuperstructure()::atGoal),
                Commands.waitUntil(m_transferBeamBreak), 
                Commands.waitUntil(() -> !m_transferBeamBreak.getAsBoolean()),//Makes sure the coral has fully passed the transfer beambreak before activating the Bang-Bang controller
                applyGrabberRotationsBangBang(kCoralEffectorVoltage,0.5), // TODO: Tune these constants
                applyState(SuperState.CORAL_TRAVEL) // Moves wristivator into the travel position
            );
        }

        public Command score() {
            return Commands.either(
                scoreCommandFactory.buildCommand(),
                Commands.none(),
                getSuperstructure()::atGoal
            );
        }

        public Command manualWristevatorControl(DoubleSupplier elevatorVoltSupplier, DoubleSupplier wristVoltSupplier) {
            return Commands.startRun(
                () -> {
                    edgeCommand.cancel();
                    updateCurrentState(SuperState.OVERRIDE);
                    updateGoalState(null);
                },
                () -> {
                    m_elevator.setVoltage(elevatorVoltSupplier.getAsDouble());
                    m_wrist.setVoltage(wristVoltSupplier.getAsDouble());
                }
            );
        }

        public Command preL1() {
            return applyState(SuperState.PRE_L1);
        }

        public Command preL2() {
            return applyState(SuperState.PRE_L2);
        }

        public Command preL3() {
            return applyState(SuperState.PRE_L3);
        }

        public Command preL4() {
            return applyState(SuperState.PRE_L4);
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

        public Command algaeGroundIntake() {
            return Commands.sequence(
                applyState(SuperState.PRE_GROUND_INTAKE).until(getSuperstructure()::atGoal),
                applyState(SuperState.GROUND_INTAKE)
            ).finallyDo(
                () -> setGoal(SuperState.ALGAE_TRAVEL)
            );
        }

        public Command algaeLowIntake() {
            return Commands.sequence(
                applyState(SuperState.PRE_LOW_INTAKE).until(getSuperstructure()::atGoal),
                applyState(SuperState.LOW_INTAKE)
            ).finallyDo(
                () -> setGoal(SuperState.ALGAE_TRAVEL)
            );
        }

        public Command algaeHighIntake() {
            return Commands.sequence(
                applyState(SuperState.PRE_HIGH_INTAKE).until(getSuperstructure()::atGoal),
                applyState(SuperState.HIGH_INTAKE)
            ).finallyDo(
                () -> setGoal(SuperState.ALGAE_TRAVEL)
            );
        }

    }

}