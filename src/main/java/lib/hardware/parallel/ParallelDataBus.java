// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.parallel;

/** NOTE: ONLY HANDLES THE DATA CHANNELS. CONTROL SIGNALS MUST BE IMPLEMENTED EXTERNALLY */
public interface ParallelDataBus extends AutoCloseable {
    abstract int[] getChannels();
    abstract int[] getHALPortHandles();
    abstract int getWidth();
}