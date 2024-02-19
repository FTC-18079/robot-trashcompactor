package org.firstinspires.ftc.teamcode.shooter;

public class ShooterConstants {
    // Shooter velocity control
    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 0.0;
    public static double kV = 1.0;
    public static double kA = 0.0;

    // Flick positions
    public static int FLICK_SHOOT_ANGLE = 1;
    public static int FLICK_REST_ANGLE = 0;

    // Shooter velocity
    public static double SHOOTER_RPM = 40.0;
    public static double SHOOTER_VELOCITY_TOLERANCE = 0.5;
    public static final int SHOOTER_TICKS = 28;

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * (double) SHOOTER_TICKS;
    }
}
