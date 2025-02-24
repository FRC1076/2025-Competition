// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;
import static frc.robot.Constants.ElevatorConstants.Control.kProfileConstraints;

import static frc.robot.Constants.ElevatorConstants.kMotorPort0;
import static frc.robot.Constants.ElevatorConstants.kMotorPort1;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorIOHardware implements ElevatorIO {
    private static final ElevatorControlConstants realControlConstants = new ElevatorControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );
    // TODO: Figure out which motor is on which side
    private final SparkMax m_leadMotor; 
    private final SparkMax m_followMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;
    
    private final RelativeEncoder m_encoder;

    public ElevatorIOHardware() {
        m_leadMotor = new SparkMax(kMotorPort0,SparkMax.MotorType.kBrushless);
        m_followMotor = new SparkMax(kMotorPort1,SparkMax.MotorType.kBrushless);

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
            .smartCurrentLimit((int) kCurrentLimit)
            .voltageCompensation(kVoltageCompensation)
            .follow(m_leadMotor, ElevatorConstants.followMotorInverted != ElevatorConstants.leadMotorInverted);
        
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_encoder = m_leadMotor.getEncoder();

        m_encoder.setPosition(0);

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);
        
    }

    /** Sets the voltage of the leader and follower elevator motors 
     * @param volts The voltage to set the motors to
    */
    @Override
    public void setVoltage(double volts){
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public ElevatorControlConstants getControlConstants() {
        return realControlConstants;
    }

    /** Used to log elevator status */
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.appliedOutput = m_leadMotor.getAppliedOutput();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.followCurrentAmps = m_followMotor.getOutputCurrent();

        inputs.elevatorHeightMeters = m_encoder.getPosition();
        inputs.velocityMetersPerSecond = m_encoder.getVelocity();
    }
}