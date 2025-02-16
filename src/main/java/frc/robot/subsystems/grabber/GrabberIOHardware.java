package frc.robot.subsystems.grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GrabberIOHardware implements GrabberIO{
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;

    private final SparkMaxConfig m_leftMotorConfig;
    private final SparkMaxConfig m_rightMotorConfig;

    public GrabberIOHardware() {
        // motor port constant is currently unknown. Change when known.
        m_leftMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);

        m_leftMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig = new SparkMaxConfig();

        m_leftMotorConfig.smartCurrentLimit((int) GrabberConstants.kCurrentLimit);
        m_rightMotorConfig.smartCurrentLimit((int) GrabberConstants.kCurrentLimit);

        m_leftMotor.configure(m_leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightMotor.configure(m_rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

    /** Used for logging */
    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.leftMotorCurrent = m_leftMotor.getOutputCurrent();
        
        inputs.rightMotorAppliedVoltage = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        inputs.rightMotorCurrent = m_rightMotor.getOutputCurrent();
    }
}