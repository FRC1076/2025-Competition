package frc.robot.subsystems.wrist;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristSimConstants;

public class WristIOSim implements WristIO {
    private final DCMotor m_wristGearbox;

    private SparkMax m_leadMotor;
    private SparkMax m_followMotor;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;

    private final SparkRelativeEncoderSim m_encoderSim;

    private ArmFeedforward m_FFController = new ArmFeedforward(
        WristSimConstants.Control.kS,
        WristSimConstants.Control.kG,
        WristSimConstants.Control.kV,
        WristSimConstants.Control.kA
    );

    private final SingleJointedArmSim m_wristSim;

    private final PIDController m_PIDController;

    public WristIOSim() {
        m_wristGearbox = DCMotor.getNEO(2);

        m_leadMotor = new SparkMax(WristConstants.kLeadMotorPort, MotorType.kBrushless);
        m_followMotor = new SparkMax(WristConstants.kFollowMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        // create motor configurations
        m_leadMotorConfig
            .inverted(WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);
            
        m_leadMotorConfig.closedLoop
        .pid(
            WristConstants.Control.kP,
            WristConstants.Control.kI,
            WristConstants.Control.kD
        );

        m_leadMotorConfig.encoder
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);
            

        m_followMotorConfig
            .follow(m_leadMotor)
            .inverted(WristConstants.kFollowMotorInverted)
            .idleMode(IdleMode.kBrake);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, null, null);
        m_followMotor.configure(m_followMotorConfig, null, null);

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
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_wristGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

        m_PIDController = new PIDController(WristSimConstants.Control.kP, WristSimConstants.Control.kI, WristSimConstants.Control.kD);
    }

    @Override
    public void simulationPeriodic() {
        m_wristSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());
        //System.out.println("Wrist Output: " + m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        m_wristSim.update(0.020);

        m_encoderSim.setPosition(m_wristSim.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        m_leadMotorSim.setAppliedOutput(voltage/m_leadMotorSim.getBusVoltage());
    }

    @Override
    public void setVoltageCharacterization(double voltage){
        setVoltage(voltage);
    }

    @Override
    public void setPosition(double positionRadians) {
        m_leadMotorSim.setAppliedOutput(
            (m_PIDController.calculate(m_encoderSim.getPosition(), positionRadians)
            + m_FFController.calculate(positionRadians, 0))
            / m_leadMotorSim.getBusVoltage()
        );
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followCurrentAmps = m_followMotorSim.getMotorCurrent();
        inputs.angle = Rotation2d.fromRadians(m_encoderSim.getPosition());
    }
}