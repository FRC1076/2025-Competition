package lib.control.multithread;

import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;
import edu.wpi.first.wpilibj.Notifier;

// A separate thread that manages PredictiveFeedbackControllers.
final class ControlThread {

    private static ControlThread inst;
    static final ReentrantLock lock = new ReentrantLock();

    static ControlThread getInstance() {
        if (inst == null) {
            inst = new ControlThread();
        }
        return inst;
    }

    private final Notifier controlNotifier = new Notifier(this::run);
    private final ArrayList<MultithreadedController> controllers = new ArrayList<>();

    void registerController(MultithreadedController controller) {
        synchronized (lock) {
            controllers.add(controller);
        }
    }

    private ControlThread() {
        controlNotifier.setName("ControlThread");
        controlNotifier.startPeriodic(0.02);
    }

    private void run() {
        synchronized (lock) {
            for (MultithreadedController controller : controllers) {
                controller.iterate();
            }
        }
    }
}
