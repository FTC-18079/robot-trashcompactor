package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    // Conveyor velocity constants
    public static double kP_CONVEYOR = 0.015;
    public static double kI_CONVEYOR = 0.0;
    public static double kD_CONVEYOR = 0.01;
    public static double kS_CONVEYOR = 0.01;
    public static double kV_CONVEYOR = 1.0;
    public static double kA_CONVEYOR = 0.0;

    // Intake velocity constants
    public static double kP_INTAKE = 0.07;
    public static double kI_INTAKE = 0.1;
    public static double kD_INTAKE = 0.02;
    public static double kS_INTAKE = 0.0;
    public static double kV_INTAKE = 1.0;
    public static double kA_INTAKE = 0.0;

    // Intake constants
    public static int INTAKE_RPM = 200;
    public static double CONVEYOR_RPM = 290.0;
    public static final int MOTOR_TICKS = 560;

    public static double toTicksPerSec(double   rpm) {
        return (rpm / 60) * (double) MOTOR_TICKS;
    }
    public static double toRPM(double tps) {
        return (tps / (double) MOTOR_TICKS) * 60.0;
    }
}
