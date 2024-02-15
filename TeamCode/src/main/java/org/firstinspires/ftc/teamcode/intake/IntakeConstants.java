package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static int DEPLOY_MIN_ANGLE = 0;
    public static int DEPLOY_MAX_ANGLE = 180;
    public static int DEPLOY_DOWN_ANGLE = 180;
    public static int DEPLOY_UP_ANGLE = 105;
    public static double CONVEYOR_RPM = 40.0;

    public static final int CONVEYOR_TICKS = 288;

    public static double toTicksPerSec(double rpm) {
        return rpm / 60 * (double) CONVEYOR_TICKS;
    }
}
