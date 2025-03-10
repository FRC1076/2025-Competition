// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.index;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IndexConstants;

public class IndexIOHardware implements IndexIO {
    private final SparkMax m_leadMotor;
    private final SparkMaxConfig m_leadMotorConfig;

    public IndexIOHardware() {
        m_leadMotor = new SparkMax(IndexConstants.kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_leadMotorConfig
            .smartCurrentLimit((int) IndexConstants.kCurrentLimit)
            .inverted(IndexConstants.kLeadMotorInverted);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void runVolts(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IndexIOInputs inputs) {
        inputs.leadMotorAppliedVoltage = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadMotorCurrent = m_leadMotor.getOutputCurrent();
    }
}
