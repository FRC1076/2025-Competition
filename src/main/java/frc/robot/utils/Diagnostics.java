package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Diagnostics {

    private long FLstartHeartbeat = 0;
    private int FLendHeartbeat = 0;

    private int FRstartHeartbeat = 0;
    private int FRendHeartbeat = 0;

    private int RLstartHeartbeat = 0;
    private int RLendHeartbeat = 0;
    
    private int RRstartHeartbeat = 0;
    private int RRendHeartbeat = 0;

    public static long getFailedDaqs() {
        return NetworkTableInstance.getDefault().getEntry("AdvantageKit/Drive/FailedDaqs").getInteger(0);
    }
}
