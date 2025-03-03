// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import static frc.robot.Constants.GrabberConstants.kCoralIntakeVoltage;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.grabber.Grabber;
import frc.robot.subsystems.superstructure.state.SuperState;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import java.util.Map;
import lib.extendedcommands.SelectWithFallbackCommand;
import java.util.HashMap;
import java.util.HashSet;
import lib.math.Combinatorics;

public class Superstructure extends SubsystemBase {
    
    private static final double algaeTravelRadians = Units.degreesToRadians(65);
    private static final double coralTravelRadians = Units.degreesToRadians(90);

    private static class MutableSuperstate {
        GrabberPossession possession;
        GrabberState grabberState;
        WristevatorState wristevatorState;
    } //TODO: Update this dynamically

    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Grabber m_grabber;
    private final Funnel m_funnel;
    private SuperState goalState;
    private SuperState currentState;
    private SuperState cachedState; // Stores the previous state for tempApplyState commands
    private final BooleanSupplier m_transferBeamBreak;
    private final MutableSuperstate state = new MutableSuperstate();
    private final DoubleSupplier wristTravelAngleSupplier = () -> state.possession == GrabberPossession.ALGAE
        ? algaeTravelRadians
        : coralTravelRadians;
    
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
    
    private Command applyWristAngle(double wristAngleRadians) {
        return Commands.runOnce(() -> m_wrist.setAngleRadians(wristAngleRadians)).andThen(
            Commands.idle().until(m_wrist::atPositionSetpoint) 
        );
    }

    private Command applyWristevatorStateSafe(double elevatorHeightMeters, double wristAngleRadians) {
        
        return Commands.sequence(
            applyWristAngle(wristTravelAngleSupplier.getAsDouble()),
            applyElevatorHeight(elevatorHeightMeters),
            applyWristAngle(wristAngleRadians)
        );
        
    }

    private Command applyGrabberVolts(double leftVoltsDifferential, double rightVoltsDifferential) {
        return Commands.runOnce(() -> m_grabber.runVoltsDifferential(leftVoltsDifferential, rightVoltsDifferential));
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

        private static final Map<SuperState,Edge> coralScoringEdgeMap = new HashMap<>();

        static {
            coralScoringEdgeMap.put(SuperState.PRE_L1,new Edge(SuperState.PRE_L1, SuperState.SCORE_L1));
            coralScoringEdgeMap.put(SuperState.PRE_L2,new Edge(SuperState.PRE_L2, SuperState.SCORE_L2));
            coralScoringEdgeMap.put(SuperState.PRE_L3,new Edge(SuperState.PRE_L3, SuperState.SCORE_L3));
            coralScoringEdgeMap.put(SuperState.PRE_L4,new Edge(SuperState.PRE_L4, SuperState.SCORE_L4));
        }

        private static final Set<SuperState> coralTraversalStates = new HashSet<>();
        private static final Set<Edge> coralTraversalEdges = new HashSet<>();

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
        
        
        private final Map<Edge,Command> edgeCommandMap = new HashMap<>();
        private final Set<Edge> forbiddenEdges = new HashSet<>();
        private final Map<SuperState,Command> scoringCommandMap = new HashMap<>();
        private final Command scoreCommand;
        

        private Command edgeCommand;

        private SuperstructureCommandFactory() {
            // Initialize coral scoring edge commands
            for (Edge edge : coralScoringEdgeMap.values()) {
                edgeCommandMap.put(edge,buildEdgeCommand(edge));
            }
            // Initialize coral traversal edge commands
            for (Edge edge : coralTraversalEdges) {
                edgeCommandMap.put(edge,buildEdgeCommand(edge));
            }
            
            scoringCommandMap.put(SuperState.PRE_L1,applyTempState(SuperState.SCORE_L1));
            scoringCommandMap.put(SuperState.PRE_L2,applyTempState(SuperState.SCORE_L2));
            scoringCommandMap.put(SuperState.PRE_L3,applyTempState(SuperState.SCORE_L3));
            scoringCommandMap.put(SuperState.PRE_L4,applyTempState(SuperState.SCORE_L4));

            scoreCommand = new SelectWithFallbackCommand<>(scoringCommandMap,Commands.none(),() -> currentState); //TODO: Should the selector be goalState or currentState?

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
                applyGrabberRotationsBangBang(kCoralIntakeVoltage,0.5), // TODO: Tune these constants
                applyState(SuperState.CORAL_TRAVEL) // Moves wristivator into the travel position
            );
        }

        public Command score() {
            return Commands.either(
                scoreCommand.asProxy(),
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

    }

}