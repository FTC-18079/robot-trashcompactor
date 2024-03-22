package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
    private final CRServo helper;
    private final Servo deploy;
    private final MotorEx upperConveyor;
    private final MotorEx lowerConveyor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new CRServo(hardwareMap, RobotMap.SERVO_INTAKE);
        helper = new CRServo(hardwareMap, RobotMap.SERVO_HELPER);
        deploy = hardwareMap.get(Servo.class, RobotMap.SERVO_DEPLOY);
        upperConveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_UPPER_CONVEYOR);
        lowerConveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_LOWER_CONVEYOR);

        setupUpperConveyor();
        setUpLowerConveyor();
    }

    public void setupUpperConveyor() {
        upperConveyor.stopAndResetEncoder();
        upperConveyor.setInverted(false);
        upperConveyor.setVeloCoefficients(kP_UPPER, kI_UPPER, kD_UPPER);
        upperConveyor.setFeedforwardCoefficients(kS_UPPER, kV_UPPER, kA_UPPER);
        upperConveyor.setRunMode(MotorEx.RunMode.VelocityControl);
        upperConveyor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setUpLowerConveyor() {
        lowerConveyor.stopAndResetEncoder();
        lowerConveyor.setInverted(true);
        lowerConveyor.setVeloCoefficients(kP_LOWER, kI_LOWER, kD_LOWER);
        lowerConveyor.setFeedforwardCoefficients(kS_LOWER, kV_LOWER, kA_LOWER);
        lowerConveyor.setRunMode(Motor.RunMode.VelocityControl);
        lowerConveyor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        intake.set(-1);
        helper.set(-1);
        upperConveyor.setVelocity(toTicksPerSec(-CONVEYOR_RPM));
        lowerConveyor.setVelocity(toTicksPerSec(-CONVEYOR_RPM / LOWER_GEAR_RATIO));
    }

    public void out() {
        intake.set(1);
        helper.set(1);
        upperConveyor.setVelocity(toTicksPerSec(CONVEYOR_RPM));
        lowerConveyor.setVelocity(toTicksPerSec(CONVEYOR_RPM / LOWER_GEAR_RATIO));
    }

    public void stop() {
        intake.stop();
        helper.stop();
        upperConveyor.stopMotor();
        lowerConveyor.stopMotor();
    }

    public void deploy() {
        deploy.setPosition(DEPLOY_DOWN_ANGLE);
    }

    public void retract() {
        deploy.setPosition(DEPLOY_UP_ANGLE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Upper Conveyor Velocity", upperConveyor.getCorrectedVelocity());
        telemetry.addData("Upper Conveyor Target", toTicksPerSec(CONVEYOR_RPM));
        telemetry.addData("Lower Conveyor Velocity", lowerConveyor.getCorrectedVelocity());
        telemetry.addData("Lower Conveyor Target", toTicksPerSec(CONVEYOR_RPM / LOWER_GEAR_RATIO));
    }
}
