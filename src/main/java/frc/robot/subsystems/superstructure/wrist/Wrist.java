// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.superstructure.wrist;

import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.superstructure.wrist.WristIOInputsAutoLogged;
import lib.control.DynamicArmFeedforward;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Wrist {

    private Optional<TrapezoidProfile.State> autoControlGoal = Optional.empty();

    private final WristIO io;
    private final ProfiledPIDController m_feedbackController;
    private final DynamicArmFeedforward m_feedforwardController;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    

    public Wrist(WristIO io) {
        this.io = io;

        var controlConstants = io.getControlConstants();
        m_feedbackController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(), 
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicArmFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

    }
    
    /** Sets the voltage of the wrist motors, compensating for gravity
     * Overrides automatic PID control
    */
    public void setVoltage(double volts) {

        autoControlGoal = Optional.empty();
        
        if (this.getAngleRadians() > WristConstants.kMaxWristAngleRadians && volts > 0) {
            volts = 0;
        } else if (this.getAngleRadians() < WristConstants.kMinWristAngleRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    /** Sets the desired rotation of the wrist */
    public void setAngle(Rotation2d position) {
        m_feedbackController.reset(getAngleRadians(),getVelocityRadsPerSec());
        autoControlGoal = Optional.of(new TrapezoidProfile.State(position.getRadians(),0));
    }

    public void setAngleRadians(double angle) {
        m_feedbackController.reset(getAngleRadians(),getVelocityRadsPerSec());
        autoControlGoal = Optional.of(new TrapezoidProfile.State(angle,0));
    }

    public void setState(TrapezoidProfile.State state) {
        m_feedbackController.reset(getAngleRadians(),getVelocityRadsPerSec());
        autoControlGoal = Optional.of(state);
    }

    /** Returns the angle of the wrist in radians */
    public double getAngleRadians() {
        return inputs.angleRadians;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.angleRadians);
    }

    public double getVelocityRadsPerSec() {
        return inputs.velocityRadiansPerSecond;
    }

    public void stop() {
        setVoltage(0);
    }

    /** Sets the feedforward kG value for the wrist */
    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    public void periodic() {
        //System.out.println("Wrist Angle: " + this.getAngleRadians());
        io.updateInputs(inputs);
        Logger.recordOutput("Wrist/PositionGoal", m_feedbackController.getGoal().position);
        Logger.processInputs("Wrist", inputs);
        autoControlGoal.ifPresent(
            (goal) -> {
                io.setVoltage(
                    m_feedbackController.calculate(inputs.angleRadians,goal)
                    + m_feedforwardController.calculate(inputs.angleRadians,m_feedbackController.getSetpoint().velocity)
                );
            }
        );
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public boolean atPositionSetpoint() {
        if (autoControlGoal.isPresent()) {
            return Math.abs(inputs.angleRadians - autoControlGoal.get().position) <= WristConstants.kWristAngleToleranceRadians;
        } else {
            return true;
        }
    }

}
