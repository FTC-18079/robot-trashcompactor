package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    // Upper conveyor velocity constants
    public static double kP_UPPER = 1.0;
    public static double kI_UPPER = 0.0;
    public static double kD_UPPER = 0.0;
    public static double kS_UPPER = 0;
    public static double kV_UPPER = 1;
    public static double kA_UPPER = 0;

    // Lower conveyor velocity constants
    public static double kP_LOWER = 1.0;
    public static double kI_LOWER = 0.0;
    public static double kD_LOWER = 0.0;
    public static double kS_LOWER = 0;
    public static double kV_LOWER = 1;
    public static double kA_LOWER = 0;

    public static int DEPLOY_DOWN_ANGLE = 0;
    public static int DEPLOY_UP_ANGLE = 1;

    // Conveyor constants
    public static double LOWER_GEAR_RATIO = 1.0; // 48.0 / 12.0 in reality
    public static double CONVEYOR_RPM = 60.0;
    public static final int CONVEYOR_TICKS = 288;

    public static double toTicksPerSec(double   rpm) {
        return (rpm / 60) * (double) CONVEYOR_TICKS;
    }
    public static double toRPM(double tps) {
        return (tps / (double) CONVEYOR_TICKS) * 60.0;
    }
}
