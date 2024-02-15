package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Shooter extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final MotorEx shooter;
    private final ServoEx flick;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        shooter = new MotorEx(hardwareMap, RobotMap.MOTOR_SHOOTER);
        flick = new SimpleServo(hardwareMap, RobotMap.SERVO_FLICK, ShooterConstants.FLICK_MIN_ANGLE, ShooterConstants.FLICK_MAX_ANGLE);

        setupShooter();
    }

    public void setupShooter() {
        shooter.setVeloCoefficients(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.resetEncoder();
    }

    public void fire() {
        flick.turnToAngle(ShooterConstants.FLICK_SHOOT_ANGLE);
    }

    public void rest() {
        flick.turnToAngle(ShooterConstants.FLICK_REST_ANGLE);
    }

    public void shoot() {
        shooter.set(ShooterConstants.toTicksPerSec(ShooterConstants.SHOOTER_RPM));
    }

    public void stop() {
        shooter.set(0);
    }

    public boolean reachedTargetVel() {
        return getShooterVelocity() == ShooterConstants.SHOOTER_RPM;
    }

    public double getShooterVelocity() {
        return shooter.getCorrectedVelocity();
    }
}
