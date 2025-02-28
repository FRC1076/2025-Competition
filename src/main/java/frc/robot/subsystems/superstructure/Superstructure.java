package frc.robot.subsystems.superstructure;

import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.dense.row.decompose.hessenberg.TridiagonalDecompositionHouseholder_CDRM;
import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholder_DDRM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.grabber.Grabber;
import frc.robot.subsystems.superstructure.state.ElevatorHeight;
import frc.robot.subsystems.superstructure.state.SuperState;
import frc.robot.subsystems.superstructure.state.WristAngle;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.utils.VirtualSubsystem;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import java.util.Map;
import lib.extendedcommands.SelectWithFallbackCommand;
import java.util.HashMap;

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
    private SuperState currentState;
    private final BooleanSupplier m_transferBeamBreak;
    private final MutableSuperstate state = new MutableSuperstate();
    private final DoubleSupplier wristTravelAngleSupplier = () -> state.possession == GrabberPossession.ALGAE
        ? algaeTravelRadians
        : coralTravelRadians;

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

    //Only serves to add this as a requirement to command compositions
    private Command requirementCommand() {
        return this.runOnce(() -> {});
    }

    private Command applyWristevatorStateSafe(double elevatorHeightMeters, double wristAngleRadians) {
        
        return Commands.sequence(
            applyWristAngle(wristTravelAngleSupplier.getAsDouble()),
            applyElevatorHeight(elevatorHeightMeters),
            applyWristAngle(wristAngleRadians)
        );
        
    }

    private Command applyGrabberVolts(double leftVoltsDifferential, double rightVoltsDifferential) {
        return this.runOnce(() -> m_grabber.runVoltsDifferential(leftVoltsDifferential, rightVoltsDifferential));
    }


    private Command applyFunnelVolts(double funnelVolts) {
        return this.run(() -> m_funnel.setVoltage(funnelVolts));
    }

    private Command applyGrabberFunnelVolts(double grabberVolts, double funnelVolts) {
        return Commands.parallel(
            Commands.runOnce(() -> m_grabber.runVolts(grabberVolts)),
            Commands.runOnce(() -> m_funnel.setVoltage(funnelVolts)),
            requirementCommand()
        );
    }

    private Superstructure getSuperstructure() {
        return this;
    }

    private SuperState getState() {
        return currentState;
    }

    private void setState(SuperState newState) {
        currentState = newState;
    }

    private Command applyGrabberRotationsBangBang(double volts, double rotations) {
        double sign = Math.signum(rotations - m_grabber.getRotations());
        return Commands.sequence(
            runOnce(() -> m_grabber.runVolts(volts)),
            Commands.waitUntil(() -> Math.signum(rotations - m_grabber.getRotations()) != sign),
            runOnce(() -> m_grabber.stop()),
            requirementCommand()
        );
    }

    public class SuperstructureCommandFactory {
        
        private final Map<SuperState,Command> scoreCommandMap = new HashMap<>();
        private final Command scoreEdgeCommand; // An edge command that transfers from a pre-scoring state to a scoring state
        private final Command autoIntakeCommand; // An edge command that transfers from an arbitrary state to the coral intake state to the coral travel state

        private SuperstructureCommandFactory() {

            scoreCommandMap.put(SuperState.PRE_L1,buildEdgeCommand(SuperState.PRE_L1,SuperState.SCORE_L1));
            scoreCommandMap.put(SuperState.PRE_L2,buildEdgeCommand(SuperState.PRE_L2,SuperState.SCORE_L2));
            scoreCommandMap.put(SuperState.PRE_L3,buildEdgeCommand(SuperState.PRE_L3,SuperState.SCORE_L3));
            scoreCommandMap.put(SuperState.PRE_L4,buildEdgeCommand(SuperState.PRE_L4,SuperState.SCORE_L4));

            scoreEdgeCommand = new SelectWithFallbackCommand<SuperState>(scoreCommandMap,requirementCommand(),() -> currentState);

            autoIntakeCommand = Commands.sequence(
                applyState(SuperState.CORAL_INTAKE),
                Commands.waitUntil(m_transferBeamBreak),
                applyGrabberRotationsBangBang(8,2), //TODO: FIX THESE CONSTANTS
                buildEdgeCommand(SuperState.CORAL_INTAKE,SuperState.CORAL_TRAVEL)
            );
            
        }
        
        private Command buildEdgeCommand(SuperState startState, SuperState goalState) {
            if (startState.equals(goalState)) {
                return Commands.runOnce(() -> {});
            }
            Command wristevatorCommand;
            // determines wristevator command
            if (startState.height != goalState.height) {
                wristevatorCommand = applyWristevatorStateSafe(goalState.height.meters,goalState.angle.radians);
            } else if (startState.angle != goalState.angle) {
                wristevatorCommand = applyWristAngle(goalState.angle.radians);
            } else {
                wristevatorCommand = Commands.runOnce(() -> {});
            }

            return Commands.sequence(
                wristevatorCommand,
                Commands.parallel(
                    Commands.runOnce(() -> m_funnel.setVoltage(goalState.funnelState.voltage)),
                    Commands.runOnce(() -> m_grabber.runVoltsDifferential(
                        goalState.grabberState.leftVolts,
                        goalState.grabberState.rightVolts
                    )),
                    Commands.runOnce(() -> setState(goalState))
                ),
                requirementCommand()
            );
        }

        public Command applyState(SuperState goalState) {
            //return buildTransCommand(getState(),goalState);
            return new DeferredCommand(() -> buildEdgeCommand(getState(),goalState),Set.of(getSuperstructure()));
        }

        // Transitions from a pre-score state to a post-score state
        public Command scoreCommand() {
            return scoreEdgeCommand;
        }


        public Command autoIntakeCoral() {
            return autoIntakeCommand;
        }
    }

}