package frc.robot.subsystems.elevator;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;
import static frc.robot.Constants.ElevatorConstants.kMotorPort0;
import static frc.robot.Constants.ElevatorConstants.kMotorPort1;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.*;

import java.util.Map;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO {
    private final SparkMax m_leadMotor; //Leader
    private final SparkMax m_followMotor; //Follower

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;
    
    private final RelativeEncoder m_encoder;

    private final SparkClosedLoopController m_closedLoopController;

    private double kG = 0.6;

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
        m_leadMotorConfig.closedLoop
            .pid(kP, kI, kD);
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

        m_encoder = m_leadMotor.getEncoder();
        m_closedLoopController = m_leadMotor.getClosedLoopController();

        m_encoder.setPosition(0);

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);
        
    }

    @Override
    public void setVoltage(double volts, double kG){
        m_leadMotor.setVoltage(volts + kG);
    }

    @Override
    public void setVoltage(double volts){
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double positionMeters){
        m_closedLoopController.setReference(
            positionMeters,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            kG,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setFFkG(double kG) {
        this.kG = kG;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.followCurrentAmps = m_followMotor.getOutputCurrent();
        inputs.elevatorHeightMeters = m_encoder.getPosition();
        inputs.velocityMetersPerSecond = m_encoder.getVelocity();
    }
}
