// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSimConstants;

import static frc.robot.Constants.ElevatorSimConstants.Control.*;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.kCurrentLimit;
import static frc.robot.Constants.ElevatorConstants.Electrical.kVoltageCompensation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkRelativeEncoderSim;



public class ElevatorIOSim implements ElevatorIO {
    
    private static final ElevatorControlConstants simControlConstants = new ElevatorControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );
    
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

    private final SparkMax m_leadMotor;
    private final SparkMax m_followMotor;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;
    
    private final SparkRelativeEncoderSim m_encoderSim;

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
    
        m_leadMotor = new SparkMax(ElevatorSimConstants.kSimMotorPort0, SparkMax.MotorType.kBrushless);
        m_followMotor = new SparkMax(ElevatorSimConstants.kSimMotorPort1, SparkMax.MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotor.setCANTimeout(250);
        m_followMotor.setCANTimeout(250);

        m_leadMotorConfig
            .inverted(ElevatorConstants.leadMotorInverted)
            .smartCurrentLimit((int) kCurrentLimit)
            .voltageCompensation(kVoltageCompensation);

        m_leadMotorConfig.encoder
            .positionConversionFactor(kPositionConversionFactor)
            .velocityConversionFactor(kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        m_followMotorConfig
            .inverted(ElevatorConstants.followMotorInverted)
            .smartCurrentLimit((int) kCurrentLimit)
            .voltageCompensation(kVoltageCompensation)
            .follow(m_leadMotor);
        
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_elevatorGearbox);
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_elevatorGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

    }

    @Override
    public void setVoltage(double voltage) {
        m_leadMotorSim.setAppliedOutput(
            voltage/m_leadMotorSim.getBusVoltage()
        );
    }

    @Override 
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followCurrentAmps = m_followMotorSim.getMotorCurrent();
        inputs.elevatorHeightMeters = m_encoderSim.getPosition();
        inputs.velocityMetersPerSecond = m_encoderSim.getVelocity();
    }

    @Override
    public ElevatorControlConstants getControlConstants() {
        return simControlConstants;
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.setPosition(m_elevatorSim.getPositionMeters());

        // Update elevator visualization with position
    }

    @Override
    public void resetPosition(double positionMeters) {
        m_encoderSim.setPosition(positionMeters);
    }
}