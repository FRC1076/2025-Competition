package lib.control.multithread;

sealed interface MultithreadedController permits DiscretizedFeedbackController {
    public void iterate();
}
