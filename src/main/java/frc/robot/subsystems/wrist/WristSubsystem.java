package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final SysIdRoutine sysid = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null,
            (state) -> Logger.recordOutput("Wrist/SysIDState", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltageCharacterization(voltage.in(Volts)),
            null,
            this
        )
    );

    public WristSubsystem(WristIO io) {
        this.io = io;
    }
    //Fix setvoltage
    public void setVoltage(double volts) {
        io.setVoltage(volts + WristConstants.Control.kG);
    }

    private void setVoltageCharacterization(double volts) {
        io.setVoltage(volts);
    }

    public void setPosition(Rotation2d position) {
        io.setPosition(position.getRadians());
    }

    public Rotation2d getAngle(){
        return inputs.angle;
    }


    public void stop() {
        setVoltage(0);
    }

    public double getAngleRadians() {
        return getAngle().getRadians();
    }

    public void setKg(double kg) {
        this.io.setFFkG(kg);
    }

    public void setVelocity(double velocityRadiansPerSecond) {
        this.io.setVelocity(velocityRadiansPerSecond);
    }

    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {},
            () -> setPosition(angle), 
            (interrupted) -> {}, 
            () -> Math.abs(angle.minus(getAngle()).getRadians()) < WristConstants.wristAngleToleranceRadians,
            this
        );
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVelocity(controlSupplier.getAsDouble() * WristConstants.maxOperatorControlVolts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return sysid.quasistatic(direction);
    }

    public Command wristSysIdDynamic(SysIdRoutine.Direction direction)
    {
        return sysid.dynamic(direction);
    }
}
