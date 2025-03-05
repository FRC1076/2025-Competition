package lib.control.multithread;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Uses multithreading to ensure that the feedback controller is polled every loop */
public final class DiscretizedFeedbackController implements MultithreadedController {
    private AtomicReference<TrapezoidProfile.State> goal;
    private final DoubleFunction<Double> m_feedforwardCalculator;
    private final ProfiledPIDController m_controller;
    private final DoubleSupplier inputSupplier;
    private final DoubleConsumer outputConsumer;
    
    public DiscretizedFeedbackController(DoubleSupplier input, DoubleConsumer output, double kP, double kI, double kD, TrapezoidProfile.Constraints constraints, DoubleFunction<Double> feedforwardCalculator) {
        inputSupplier = input;
        outputConsumer = output;
        m_controller = new ProfiledPIDController(kP, kI, kD, constraints);
        m_feedforwardCalculator = feedforwardCalculator;
        ControlThread.getInstance().registerController(this);
    }

    @Override
    public void iterate() {
        double output = m_controller.calculate(inputSupplier.getAsDouble(),goal.get());
        outputConsumer.accept(output + m_feedforwardCalculator.apply(m_controller.getSetpoint().velocity));
    }

    public void reset(double measuredPosition) {
        synchronized (ControlThread.lock) {
            m_controller.reset(measuredPosition);
        }
    }
}
