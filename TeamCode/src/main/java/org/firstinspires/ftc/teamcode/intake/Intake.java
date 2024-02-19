package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final CRServo intake;
    private final Servo deploy;
    private final MotorEx upperConveyor;
    private final MotorEx lowerConveyor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new CRServo(hardwareMap, RobotMap.SERVO_INTAKE);
        deploy = hardwareMap.get(Servo.class, RobotMap.SERVO_DEPLOY);
        upperConveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_UPPER_CONVEYOR);
        lowerConveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_LOWER_CONVEYOR);

        setupUpperConveyor();
        setUpLowerConveyor();
    }

    public void setupUpperConveyor() {
        upperConveyor.stopAndResetEncoder();
        upperConveyor.setVeloCoefficients(kP, kI, kD);
        upperConveyor.setFeedforwardCoefficients(kS, kV, kA);
        upperConveyor.setRunMode(MotorEx.RunMode.VelocityControl);
        upperConveyor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setUpLowerConveyor() {
        lowerConveyor.stopAndResetEncoder();
        lowerConveyor.setRunMode(Motor.RunMode.VelocityControl);
        upperConveyor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        intake.set(-1);
        upperConveyor.setVelocity(toTicksPerSec(-CONVEYOR_RPM));
        telemetry.addData("Conveyor velocity", upperConveyor.getCorrectedVelocity());
        telemetry.update();
    }

    public void out() {
        intake.set(1);
        upperConveyor.setVelocity(toTicksPerSec(CONVEYOR_RPM));
        telemetry.addData("Conveyor velocity", upperConveyor.getCorrectedVelocity());
        telemetry.update();
    }

    public void stop() {
        intake.stop();
        upperConveyor.stopMotor();
        telemetry.addData("Conveyor velocity", upperConveyor.getCorrectedVelocity());
        telemetry.update();
    }

    public void deploy() {
        deploy.setPosition(DEPLOY_DOWN_ANGLE);
    }

    public void retract() {
        deploy.setPosition(DEPLOY_UP_ANGLE);
    }
}
