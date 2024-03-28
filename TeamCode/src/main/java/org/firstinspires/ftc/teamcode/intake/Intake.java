package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import static org.firstinspires.ftc.teamcode.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final MotorEx conveyor;
    private final MotorEx conveyorFollower;
    private final MotorEx intake;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        conveyor = new MotorEx(hardwareMap, RobotMap.MOTOR_CONVEYOR);
        conveyorFollower = new MotorEx(hardwareMap, RobotMap.MOTOR_CONVEYOR_FOLLOWER);
        intake = new MotorEx(hardwareMap, RobotMap.MOTOR_INTAKE);

        configConveyor();
        configIntake();
    }

    public void configConveyor() {
        conveyor.stopAndResetEncoder();
        conveyor.setInverted(true);
        conveyor.setVeloCoefficients(kP_CONVEYOR, kI_CONVEYOR, kD_CONVEYOR);
        conveyor.setFeedforwardCoefficients(kS_CONVEYOR, kV_CONVEYOR, kA_CONVEYOR);
        conveyor.setRunMode(Motor.RunMode.VelocityControl);
        conveyor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        conveyorFollower.stopAndResetEncoder();
        conveyorFollower.setInverted(false);
        conveyorFollower.setVeloCoefficients(kP_CONVEYOR, kI_CONVEYOR, kD_CONVEYOR);
        conveyorFollower.setFeedforwardCoefficients(kS_CONVEYOR, kV_CONVEYOR, kA_CONVEYOR);
        conveyorFollower.setRunMode(Motor.RunMode.VelocityControl);
        conveyorFollower.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void configIntake() {
        intake.stopAndResetEncoder();
        intake.setInverted(true);
        intake.setVeloCoefficients(kP_INTAKE, kI_INTAKE, kD_INTAKE);
        intake.setFeedforwardCoefficients(kS_INTAKE, kV_INTAKE, kA_INTAKE);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void in() {
        intake.setVelocity(toTicksPerSec(INTAKE_RPM));
        conveyor.setVelocity(toTicksPerSec(CONVEYOR_RPM));
        conveyorFollower.setVelocity(toTicksPerSec(CONVEYOR_RPM));
    }

    public void out() {
        intake.setVelocity(toTicksPerSec(-INTAKE_RPM));
        conveyor.setVelocity(toTicksPerSec(-CONVEYOR_RPM));
        conveyorFollower.setVelocity(toTicksPerSec(-CONVEYOR_RPM));
    }

    public void stop() {
        intake.stopMotor();
        conveyor.stopMotor();
        conveyorFollower.stopMotor();
    }

    @Override
    public void periodic() {
        telemetry.addData("Left Conveyor Velocity", toRPM(conveyor.getCorrectedVelocity()));
        telemetry.addData("Right Conveyor Velocity", toRPM(conveyorFollower.getCorrectedVelocity()));
        telemetry.addData("Intake Velocity", toRPM(intake.getCorrectedVelocity()));
    }
}
