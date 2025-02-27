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
    private final BooleanSupplier m_grabberBeamBreak;
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
        m_grabberBeamBreak = grabberBeamBreak;
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
        return Commands.runOnce(() -> {},this);
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

    public class SuperstructureCommandFactory {

        private SuperstructureCommandFactory() {}
        
        private Command buildTransCommand(SuperState startState, SuperState goalState) {
            if (currentState.equals(goalState)) {
                return runOnce(() -> {});
            }
            Command wristevatorCommand;
            // determines wristevator command
            if (currentState.height != goalState.height) {
                wristevatorCommand = applyWristevatorStateSafe(goalState.height.meters,goalState.angle.radians);
            } else if (currentState.angle != goalState.angle) {
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
            return new DeferredCommand(() -> buildTransCommand(getState(),goalState),Set.of(getSuperstructure()));
        }

        public Command applyGrabberRotationsBangBang(double volts, double rotations) {
            double sign = Math.signum(rotations - m_grabber.getRotations());
            return Commands.sequence(
                runOnce(() -> m_grabber.runVolts(volts)),
                Commands.waitUntil(() -> Math.signum(rotations - m_grabber.getRotations()) != sign),
                runOnce(() -> m_grabber.stop())
            );
        }
    }

}