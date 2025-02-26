// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import lib.control.DynamicElevatorFeedforward;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Elevator {

    Optional<TrapezoidProfile.State> autoControlGoal;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ProfiledPIDController m_feedbackController;
    private final DynamicElevatorFeedforward m_feedforwardController;

    public Elevator(ElevatorIO io){
        this.io = io;
        var controlConstants = io.getControlConstants();
        m_feedbackController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(), 
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicElevatorFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

    }

    public void periodic(){
        //System.out.println("Elevator: " + this.getPositionMeters());
        io.updateInputs(inputs);
        //Logger.recordOutput("Elevator/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.processInputs("Elevator",inputs);
        autoControlGoal.ifPresent(
            (goal) -> {
                io.setVoltage(
                    m_feedbackController.calculate(inputs.elevatorHeightMeters,goal)
                    + m_feedforwardController.calculate(m_feedbackController.getSetpoint().velocity)
                );
            }
        );
    }

    public void simulationPeriodic(){
        io.simulationPeriodic();
    }
    
    /** Set desired position of the elevator
     * @param positionMeters Desired position of the elevator in meters
     */
    public void setPosition(double positionMeters) {
        m_feedbackController.reset(getPositionMeters(),getVelocityMetersPerSecond());
        autoControlGoal = Optional.of(new TrapezoidProfile.State(positionMeters,0));
    }

    

    /** 
     * Set voltage of the elevator motors, with added compensation for gravity.
     * Overrides automatic PID control
     * @param volts Desired voltage of the elevator
     */
    public void setVoltage(double volts) {

        autoControlGoal = Optional.empty();
        
        if (this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = 0;
        } else if (this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = 0;
        }
    
        io.setVoltage(volts + m_feedforwardController.getKg());

    }

    /** 
     * Set kG of the elevator's feedforward
     * @param kg New kG value in volts
     */
    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    /** Returns the velocity of the elevator, as a double */
    public double getVelocityMetersPerSecond() {
        return inputs.velocityMetersPerSecond;
    }

    public boolean atPositionSetpoint() {
        if (autoControlGoal.isPresent()) {
            return Math.abs(inputs.elevatorHeightMeters - autoControlGoal.get().position) <= ElevatorConstants.kElevatorPositionToleranceMeters;
        } else {
            return true;
        }
    }

}
