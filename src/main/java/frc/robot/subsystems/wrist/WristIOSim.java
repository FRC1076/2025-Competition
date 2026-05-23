// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristSimConstants;
import static frc.robot.Constants.WristSimConstants.Control.*;

public class WristIOSim implements WristIO {

    private static final WristControlConstants simControlConstants = new WristControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );

    private final DCMotor m_wristGearbox;

    private SparkMax m_leadMotor;

    private final SparkMaxSim m_leadMotorSim;

    private SparkMaxConfig m_leadMotorConfig;

    private final SparkRelativeEncoderSim m_encoderSim;

    private final SingleJointedArmSim m_wristSim;

    public WristIOSim() {
        m_wristGearbox = DCMotor.getNEO(2);

        m_leadMotor = new SparkMax(WristConstants.kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();

        // create motor configurations
        m_leadMotorConfig
            .inverted(WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);

        m_leadMotorConfig.encoder
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, null, null);

        m_wristSim = new SingleJointedArmSim(
            m_wristGearbox,
            WristSimConstants.kWristGearingReductions,
            SingleJointedArmSim.estimateMOI(WristSimConstants.kWristLength, WristSimConstants.kWristMass),
            WristSimConstants.kWristLength,
            WristSimConstants.kMinAngleRads,
            WristSimConstants.kMaxAngleRads,
            true,
            0,
            //WristSimConstants.kWristEncoderDistPerPulse,
            0.0,
            0.0
        );

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_wristGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

    }

    @Override
    public void simulationPeriodic() {
        m_wristSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        m_wristSim.update(0.020);

        m_encoderSim.setPosition(m_wristSim.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        m_leadMotorSim.setAppliedOutput(voltage/m_leadMotorSim.getBusVoltage());
    }

    @Override
    public WristControlConstants getControlConstants() {
        return simControlConstants;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.angleRadians = m_encoderSim.getPosition();
        inputs.velocityRadiansPerSecond = m_encoderSim.getVelocity();
    }
}