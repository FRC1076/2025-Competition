package frc.robot.subsystems.superstructure.funnel;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.FunnelConstants;

public class FunnelIOHardware implements FunnelIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    public FunnelIOHardware() {
        m_motor = new SparkMax(FunnelConstants.kMotorPort, MotorType.kBrushless);

        m_motorConfig = new SparkMaxConfig();
        m_motorConfig
            .smartCurrentLimit((int) FunnelConstants.kCurrentLimit)
            .inverted(FunnelConstants.kMotorInverted);

        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.appliedCurrent = m_motor.getOutputCurrent();
    }
}
