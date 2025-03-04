// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOInputsAutoLogged;
import lib.control.DynamicElevatorFeedforward;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.ElevatorConstants.Homing.*;

import org.littletonrobotics.junction.Logger;

public class Elevator {

    Optional<TrapezoidProfile.State> profileGoal = Optional.empty();

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ProfiledPIDController m_feedbackController;
    private final DynamicElevatorFeedforward m_feedforwardController;

    private boolean homed = false;
    private Debouncer homingDebouncer;

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
        Logger.recordOutput("Elevator/Goal", m_feedbackController.getGoal().position);
        Logger.processInputs("Elevator",inputs);
        profileGoal.ifPresent(
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
        profileGoal = Optional.of(new TrapezoidProfile.State(positionMeters,0));
    }

    public void setState(TrapezoidProfile.State state) {
        m_feedbackController.reset(getPositionMeters(),getVelocityMetersPerSecond());
        profileGoal = Optional.of(state);
    }
    

    /** 
     * Set voltage of the elevator motors, with added compensation for gravity.
     * Overrides automatic PID control
     * @param volts Desired voltage of the elevator
     */
    public void setVoltage(double volts) {

        profileGoal = Optional.empty();
        
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
        if (profileGoal.isPresent()) {
            return Math.abs(inputs.elevatorHeightMeters - profileGoal.get().position) <= ElevatorConstants.kElevatorPositionToleranceMeters;
        } else {
            return true;
        }
    }

    /** Automatically rehomes the elevator encoder */
    public Command autoHome() {
        return Commands.startRun(
            () -> {
                profileGoal = Optional.empty(); 
                homed = false;
                homingDebouncer = new Debouncer(homingDebounceTime);
                homingDebouncer.calculate(false);
                io.setVoltage(homingVolts);
            },
            () -> {
                homed = homingDebouncer.calculate(Math.abs(inputs.velocityMetersPerSecond) <= homingVelocityThreshold);
            }
        )
        .until(() -> homed)
        .andThen(io::stop)
        .finallyDo(io::zeroPosition);
    }



}
