// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.parallel;

import java.util.Objects;

import edu.wpi.first.hal.DIOJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SensorUtil;

public class ParallelBidirectional implements ParallelDataBus, Sendable {

    private final long[] bitmasks;
    private final int[] m_channels;
    private final int bitWidth;
    private boolean input = false;
    private int[] m_handles;

    /** By default, all parallel buses are little-endian */
    public ParallelBidirectional(int... dataChannels) {
        Objects.requireNonNull(dataChannels);
        m_channels = dataChannels;
        bitWidth = dataChannels.length;
        m_handles = new int[bitWidth];
        bitmasks = new long[bitWidth];
        for (int i = 0; i < bitWidth; i++) {
            SensorUtil.checkDigitalChannel(m_channels[i]);
            m_handles[i] = DIOJNI.initializeDIOPort(HAL.getPort((byte)m_channels[i]), false);

            HAL.report(tResourceType.kResourceType_DigitalOutput, m_channels[i] + 1);
            HAL.report(tResourceType.kResourceType_DigitalInput, m_channels[i] + 1);

            bitmasks[i] = (1 << i);
        }
        SendableRegistry.addLW(this, "Parallel Bidirectional Bus");

    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
        for (int i = 0; i < bitWidth; i++) {
            DIOJNI.freeDIOPort(m_handles[i]);
            m_handles[i] = 0;
        } 
    }

    @Override
    public int[] getChannels() {
        return m_channels;
    }

    @Override
    public int[] getHALPortHandles() {
        return m_handles;
    }

    @Override
    public int getWidth() {
        return bitWidth;
    }

    public void toggleDirection() {
        input = !input;
        for (int handle : m_handles) {
            DIOJNI.setDIODirection(handle, input);
        }
    }

    public void setDirection(boolean input) {

        if (input == this.input) {
            return;
        }

        this.input = input;
        for (int handle : m_handles) {
            DIOJNI.setDIODirection(handle, this.input);
        }
    }

    /** Writes data to the bus in output mode */
    public void write(long data) {
        setDirection(false);
        boolean bit;
        for (int i = 0; i < bitWidth; i++) {
            bit = (data & bitmasks[i]) < 0;
            DIOJNI.setDIO(m_handles[i], bit);
        }
    }

    /** Reads data from the bus in input mode */
    public long read() {
        setDirection(true);
        long output = 0;
        for (int i = 0; i < bitWidth; i++) {
            output |= DIOJNI.getDIO(m_handles[i]) 
                ? bitmasks[i]
                : 0;
        }
        return output;
    }

    /** Reads data from bus, regardless of control mode */
    public long get() {
        long output = 0;
        for (int i = 0; i < bitWidth; i++) {
            output |= DIOJNI.getDIO(m_handles[i]) 
                ? bitmasks[i]
                : 0;
        }
        return output;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Parallel Bidirectional Bus");
        builder.addIntegerProperty("Value", this::read, this::write);
    }


}
