// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

public class GrabberIOHardware implements GrabberIO{
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;
    private final CANrange m_coralSensor;

    private final RelativeEncoder m_encoder;

    private final SparkMaxConfig m_leftMotorConfig;
    private final SparkMaxConfig m_rightMotorConfig;
    private final StatusSignal<Distance> coralSensorSignal;

    public GrabberIOHardware() {
        m_leftMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);
        m_coralSensor = new CANrange(6);

        m_encoder = m_leftMotor.getEncoder();

        m_leftMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig = new SparkMaxConfig();

        m_leftMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kLeftMotorInverted)
        .encoder
            .positionConversionFactor(GrabberConstants.kPositionConversionFactor);
            
        m_rightMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kRightMotorInverted);

        m_leftMotor.configure(m_leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightMotor.configure(m_rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        coralSensorSignal = m_coralSensor.getDistance(true);
    }

    /** Sets both motors to the same voltage */
    @Override
    public void runVolts(double volts) {
        m_leftMotor.setVoltage(volts);
        m_rightMotor.setVoltage(volts);
    }

    /** Sets the voltages of the left and right motors individually */
    @Override
    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        m_leftMotor.setVoltage(leftMotorVolts);
        m_rightMotor.setVoltage(rightMotorVolts);
    }

    /** Returns the average output current of the two grabber motors */
    @Override
    public double getOutputCurrent() {
        return (m_leftMotor.getOutputCurrent() + m_rightMotor.getOutputCurrent()) / 2;
    }

    /** Used for logging */
    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.leftMotorCurrent = m_leftMotor.getOutputCurrent();
        
        inputs.rightMotorAppliedVoltage = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        inputs.rightMotorCurrent = m_rightMotor.getOutputCurrent();

        inputs.motorPositionRadians = m_encoder.getPosition(); //This is used for bang-bang control
        inputs.hasCoral = coralSensorSignal.getValueAsDouble() < GrabberConstants.kCoralDistanceThreshold;
    }
}