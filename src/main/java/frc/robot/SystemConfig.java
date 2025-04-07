// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot;

public final class SystemConfig {

    public static class SystemModes {
        public static final int kReal = 0;
        public static final int kSim = 1;
        public static final int kReplay = 2;
    }

    public static class SysIDModes {
        public static final int kNone = 0;
        public static final int kDriveTranslation = 1;
        public static final int kDriveRotation = 2;
        public static final int kDriveSteer = 3;
        public static final int kElevator = 4;
        public static final int kWrist = 5;
    }

    public static class PrebuildModes {
        public static final int kDefined = 0; // Only prebuilds edges that are explicitly defined in prebuildEdges
        public static final int kNone = 1; // Prebuilds no edges at all
        public static final int kAll = 2; // Prebuilds all possible edges
    }

    /*
    * COMPETITION SYSTEM CONFIGURATION
    * 
    * systemMode: SystemModes.kReal
    * sysIDMode: SysIDModes.kNone
    * prebuildMode = PrebuildModes.kDefined
    * weldedField = true if welded, false if AndyMark (All official FiM fields are welded, The AADL field is AndyMark)
    * logOdometry: false
    * logSwerveModules: false
    * logCTRE: false
    * raiseThreadPriority: true
    * 
    * VERIFY THAT THE SYSTEM CONFIGURATION MATCHES WHAT IS SHOWN ABOVE BEFORE DEPLOYING CODE FOR A MATCH
    */
    public static final int systemMode = SystemModes.kReal;
    public static final int sysIDMode = SysIDModes.kNone;
    public static final int prebuildMode = PrebuildModes.kDefined;
    public static final boolean weldedField = true;
    public static final boolean logOdometry = false;
    public static final boolean logSwerveModules = false; // Whether swerve module logging should be enabled
    public static final boolean logCTRE = false; // Whether CTRE hoot logging should be enabled
    public static final boolean raiseThreadPriority = true; // Whether the main thread should have its priority raised
}