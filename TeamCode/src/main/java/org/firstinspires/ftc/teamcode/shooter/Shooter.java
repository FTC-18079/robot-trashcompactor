package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final MotorEx shooter;
    private final Servo flick;
    private boolean isShootingMode = false;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        shooter = new MotorEx(hardwareMap, RobotMap.MOTOR_SHOOTER);
        flick = hardwareMap.get(Servo.class, RobotMap.SERVO_FLICK);

        setupShooter();
    }

    public void setupShooter() {
        shooter.stopAndResetEncoder();
        shooter.setInverted(false);
        shooter.setVeloCoefficients(kP, kI, kD);
        shooter.setFeedforwardCoefficients(kS, kV, kA);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void fire() {
        flick.setPosition(FLICK_SHOOT_ANGLE);
    }

    public void rest() {
        flick.setPosition(FLICK_REST_ANGLE);
    }

    public void enable() {
        shooter.setVelocity(toTicksPerSec(SHOOTER_RPM));
    }

    public void stop() {
        shooter.stopMotor();
    }

    public boolean reachedTargetVel() {
        return Math.abs(SHOOTER_RPM - toRPM(getShooterVelocity())) <= SHOOTER_VELOCITY_TOLERANCE;
    }

    public double getShooterVelocity() {
        return shooter.getCorrectedVelocity();
    }

    public void toggleShootingMode() {
        isShootingMode = !isShootingMode;
    }

    public boolean isShootingMode() {
        return isShootingMode;
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter Velocity", toRPM(getShooterVelocity()));
//        telemetry.addData("Shooting Mode?", isShootingMode);
    }
}
