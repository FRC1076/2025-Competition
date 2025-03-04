// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.wrist;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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

    private final SingleJointedArmSim m_wristSim;

    private double appliedVoltage;

    public WristIOSim() {
        m_wristGearbox = DCMotor.getNEO(1);
        m_wristSim = new SingleJointedArmSim(
            m_wristGearbox,
            WristSimConstants.kWristGearingReductions,
            SingleJointedArmSim.estimateMOI(WristSimConstants.kWristLength, WristSimConstants.kWristMass),
            WristSimConstants.kWristLength,
            WristSimConstants.kMinAngleRads,
            WristSimConstants.kMaxAngleRads,
            true,
            0
            //WristSimConstants.kWristEncoderDistPerPulse,
        );

    }

    @Override
    public void simulationPeriodic() {
        m_wristSim.setInput(appliedVoltage);

        m_wristSim.update(0.020);
    }

    @Override
    public void setVoltage(double voltage) {
        this.appliedVoltage = voltage;
    }

    @Override
    public WristControlConstants getControlConstants() {
        return simControlConstants;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = appliedVoltage;
        inputs.leadCurrentAmps = m_wristSim.getCurrentDrawAmps();
        inputs.angleRadians = m_wristSim.getAngleRads();
        inputs.velocityRadiansPerSecond = m_wristSim.getVelocityRadPerSec();
    }
}