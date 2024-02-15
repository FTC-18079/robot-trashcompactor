package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Intake extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final CRServo intake;
    private final ServoEx deploy;
    private final MotorEx conveyor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new CRServo(hardwareMap, RobotMap.SERVO_INTAKE);
        deploy = new SimpleServo(hardwareMap, RobotMap.SERVO_DEPLOY, 0, 180, AngleUnit.DEGREES);
        conveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_CONVEYOR);

        setupConveyor();
    }

    public void setupConveyor() {
        conveyor.setVeloCoefficients(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        conveyor.setRunMode(MotorEx.RunMode.VelocityControl);
        conveyor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        conveyor.resetEncoder();
    }

    public void in() {
        intake.set(1);
    }

    public void out() {
        intake.set(-1);
    }

    public void deploy() {

    }

    public void retract() {

    }
}
