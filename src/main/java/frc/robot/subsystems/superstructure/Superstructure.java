package frc.robot.subsystems.superstructure;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import org.ejml.dense.row.decompose.hessenberg.TridiagonalDecompositionHouseholder_CDRM;
import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholder_DDRM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.funnel.Funnel;
import frc.robot.subsystems.superstructure.grabber.Grabber;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.utils.VirtualSubsystem;

public class Superstructure extends SubsystemBase {
    
    private class MutableSuperstate {
        boolean grabberPossessCoral;
        double elevatorPositionMeters;
        double wristPositionRadians;
        double grabberVolts;
        double funnelVolts;
    }

    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Grabber m_grabber;
    private final Funnel m_funnel;
    private final BooleanSupplier m_grabberBeamBreak;
    private final MutableSuperstate goalState = new MutableSuperstate();

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

    private Command setElevatorHeight(double positionMeters) {
        return Commands.runOnce(() -> m_elevator.setPosition(positionMeters)).andThen(
            Commands.idle().until(m_elevator::atPositionSetpoint)
        );
    }
    
    private Command setWristAngle(double wristAngleRadians) {
        return Commands.runOnce(() -> m_wrist.setAngleRadians(wristAngleRadians)).andThen(
            Commands.idle().until(m_wrist::atPositionSetpoint)
        );
    }

    //Only serves to add this as a requirement to command compositions
    private Command requirementCommand() {
        return Commands.runOnce(() -> {},this);
    }

    public Command applyWristevatorStateSafe(double elevatorHeightMeters, double wristAngleRadians, double travelAngleRadians) {
        return Commands.sequence(
            setWristAngle(travelAngleRadians),
            setElevatorHeight(elevatorHeightMeters),
            setWristAngle(wristAngleRadians),
            requirementCommand()
        );
    }
    

}