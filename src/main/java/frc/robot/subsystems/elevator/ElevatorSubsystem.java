// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;
import lib.control.DynamicElevatorFeedforward;
import lib.control.DynamicProfiledPIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    public static final double homingVolts = -0.1;
    public static final double homingDebounceTime = 0.25;
    public static final double homingVelocityThreshold = 0.1;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final DynamicProfiledPIDController m_profiledPIDController;
    private final DynamicElevatorFeedforward m_feedforwardController;
    
    private boolean homed = false;
    private Debouncer homingDebouncer;

    private final SysIdRoutine m_elevatorSysIdRoutine;

    public ElevatorSubsystem(ElevatorIO io, DoubleSupplier periodSupplier){
        this.io = io;
        var controlConstants = io.getControlConstants();
        m_profiledPIDController = new DynamicProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(), 
            periodSupplier,
            0.02,
            0.05,
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicElevatorFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

        m_elevatorSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(2.5), // Reduce dynamic step voltage to 4 V to prevent brownout 
                null,        // Use default timeout (10 s)
                // Log state with AdvantageKit
                (state) -> Logger.recordOutput("Elevator/SysIDState", state.toString())
            ) , 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), // Voltage consumer for the SysID routine (Represents a function that SysID uses to pass voltages to the subsystem being characterized)
                null,
                this
            )
        );
    }

    @Override
    public void periodic(){
        //System.out.println("Elevator: " + this.getPositionMeters());
        io.updateInputs(inputs);
        Logger.recordOutput("Elevator/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic(){
        io.simulationPeriodic();
    }
    
    /** Set desired position of the elevator
     * @param positionMeters Desired position of the elevator in meters
     */
    public void setPosition(double positionMeters) {
        io.setVoltage (
            m_profiledPIDController.calculate(getPositionMeters(), MathUtil.clamp(positionMeters, ElevatorConstants.kMinElevatorHeightMeters, ElevatorConstants.kMaxElevatorHeightMeters))
            + m_feedforwardController.calculate(m_profiledPIDController.getSetpoint().velocity)
        );
    }

    /** 
     * Set voltage of the elevator motors, with added compensation for gravity
     * @param volts Desired voltage of the elevator
     */
    public void setVoltage(double volts) {
        
        if (this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = 0;
        } else if (this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = 0;
        }
    
        io.setVoltage(volts + m_feedforwardController.getKg());
    }

    /** Set kG of the elevator's feedforward
     * @param kg New kG value in volts
     */
    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    /** This method isn't used for any command logic. It's only used for LEDs */
    public boolean isZeroed() {
        return homed;
    }

    
    /* ######################################################################## */
    /* # Public Command Factories                                             # */
    /* ######################################################################## */
    
    /** Returns a command that sets the position of the elevator
     * @param positionMeters Desired position of the elevator in meters
     */
    public Command applyPosition(double positionMeters) {
        return new FunctionalCommand(
            () -> m_profiledPIDController.reset(getPositionMeters()),
            () -> setPosition(positionMeters),
            (interrupted) -> {},
            // () -> {io.resetController();},
            // () -> {setPosition(positionMeters);},
            // (interrupted) -> {setVoltage(io.getFFkG());},
            () -> Math.abs(positionMeters - getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters,
            this
        );
    }
    
    /** Returns a command that sets the voltage of the elevator manually and adds kG.
     * @param controlSupplier Supplier that returns the desired voltage of the elevator
     * @param higherMaxSpeedSupplier Supplier that returns whether or not the max elevator control speed should use the lower (false) or higher (true) max voltage
     */
    public Command applyManualControl(DoubleSupplier controlSupplier, BooleanSupplier higherMaxSpeedSupplier) {
        return run(higherMaxSpeedSupplier.getAsBoolean()
            ? () -> setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.fasterMaxOperatorControlVolts)
            : () -> setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts)
        );
    }

    /** 
     * Temporarily allows the operator unrestricted control over the elevator's position, in order to let
     * them manually move the elevator to the zero position in order to rezero the encoder in the event that it
     * gets thrown off by acceleration. This version always uses the lower max control voltage.
     * NOTE: both software stops and gravity compensation are disabled during this command
     */
    public Command zeroEncoderJoystickControl(DoubleSupplier controlSupplier) {
        return run(() -> io.setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts))
            .finallyDo(() -> io.resetPosition(0));
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }

    public Command autoHome() {
        return startRun(
            () -> {
                homed = false;
                homingDebouncer = new Debouncer(homingDebounceTime);
                homingDebouncer.calculate(false);
                io.setVoltage(homingVolts);
            },
            () -> {
                homed = homingDebouncer.calculate(Math.abs(inputs.velocityMetersPerSecond) <= homingVelocityThreshold);
            }
        )
        .until(() -> homed)
        .andThen(() -> io.setVoltage(0))
        .finallyDo(() -> io.resetPosition(0));
    }
}
