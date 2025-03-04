// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.Constants.ElevatorSimConstants;

import static frc.robot.Constants.ElevatorSimConstants.Control.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;



public class ElevatorIOSim implements ElevatorIO {
    
    private static final ElevatorControlConstants simControlConstants = new ElevatorControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );
    
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

    private double appliedVoltage = 0.0;

    private final ElevatorSim m_elevatorSim;

    public ElevatorIOSim() {
        m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            ElevatorSimConstants.kElevatorGearing,
            ElevatorSimConstants.kCarriageMass,
            ElevatorSimConstants.kElevatorDrumRadius,
            ElevatorSimConstants.kMinElevatorHeightMeters,
            ElevatorSimConstants.kMaxElevatorHeightMeters,
            true,
            0,
            0,
            0
        );


    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
    }

    @Override 
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
        inputs.leadCurrentAmps = m_elevatorSim.getCurrentDrawAmps();
        inputs.followCurrentAmps = m_elevatorSim.getCurrentDrawAmps();
        inputs.elevatorHeightMeters = m_elevatorSim.getPositionMeters();
        inputs.velocityMetersPerSecond = m_elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public ElevatorControlConstants getControlConstants() {
        return simControlConstants;
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(appliedVoltage);

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

    }

    @Override
    public void resetPosition(double positionMeters) {
        //m_encoderSim.setPosition(positionMeters);
    }
}