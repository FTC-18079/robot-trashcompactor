package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Shooter velocity control
    public static double kP = 0.45;
    public static double kI = 0.0;
    public static double kD = 0.2;
    public static double kS = 0.5;
    public static double kV = 1.0;
    public static double kA = 0.0;

    // Flick positions
    public static double FLICK_SHOOT_ANGLE = 1;
    public static double FLICK_REST_ANGLE = 0.57;
    public static int RETRACT_DELAY = 1100; // ms

    // Shooter velocity
    public static double SHOOTER_RPM = 240.0;
    public static double SHOOTER_VELOCITY_TOLERANCE = 5.0;
    public static final int SHOOTER_TICKS = 560;

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * (double) SHOOTER_TICKS;
    }

    public static double toRPM(double tps) {
        return (tps / (double) SHOOTER_TICKS) * 60.0;
    }
}
