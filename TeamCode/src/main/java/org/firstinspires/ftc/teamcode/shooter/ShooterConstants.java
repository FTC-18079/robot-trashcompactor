package org.firstinspires.ftc.teamcode.shooter;

public class ShooterConstants {
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static int FLICK_MIN_ANGLE = 0;
    public static int FLICK_MAX_ANGLE = 90;
    public static int FLICK_SHOOT_ANGLE = 80;
    public static int FLICK_REST_ANGLE = 0;
    public static double SHOOTER_RPM = 40.0;

    public static final int SHOOTER_TICKS = 0;

    public static double toTicksPerSec(double rpm) {
        return rpm / 60 * (double) SHOOTER_TICKS;
    }
}
