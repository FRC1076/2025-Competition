// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;
import static frc.robot.Constants.WristConstants.Control.*;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class WristIOHardware implements WristIO {
    private static final WristControlConstants realControlConstants = new WristControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );

    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    //private final RelativeEncoder m_relativeEncoder;
    private final SparkAbsoluteEncoder m_absoluteEncoder;
    // private final RelativeEncoder m_alternateEncoder;

    public WristIOHardware() {
        m_leadMotor = new SparkMax(WristConstants.kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        

        // create motor configurations
        m_leadMotorConfig
            .inverted(WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) WristConstants.kSmartCurrentLimit);
 
            /* 
        m_leadMotorConfig.alternateEncoder
            .setSparkMaxDataPortConfig()
            .countsPerRevolution(WristConstants.kCountsPerRevolution)
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);*/

        m_leadMotorConfig.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);
        
        /*
        m_leadMotorConfig.alternateEncoder.apply(
            new AlternateEncoderConfig()
            .setSparkMaxDataPortConfig()
            .countsPerRevolution(WristConstants.kCountsPerRevolution)
            .positionConversionFactor(2 * Math.PI)
            .velocityConversionFactor(2 * Math.PI)
        );
        */
            
        
        
        m_leadMotorConfig.encoder
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        //m_relativeEncoder = m_leadMotor.getAlternateEncoder();
        //m_relativeEncoder.setPosition(Rotation2d.kCCW_90deg.getRadians());

        // m_alternateEncoder = m_leadMotor.Encoder();
        // m_alternateEncoder.setPosition(Rotation2d.kCCW_90deg.getRadians());

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();

    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public WristControlConstants getControlConstants() {
        return realControlConstants;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = m_absoluteEncoder.getPosition();
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();
        // inputs.angleRadians = m_alternateEncoder.getPosition();
        // inputs.velocityRadiansPerSecond = m_alternateEncoder.getVelocity();
    }

}
