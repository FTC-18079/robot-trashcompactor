package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.commands.ShootCommand;

public class TeamRobot extends Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Pose2d initialPose;

    // Subsystems
    Chassis chassis;
    Intake intake;
    Shooter shooter;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO
    }

    public TeamRobot(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2, Pose2d initialPose) {
        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);
        this.initialPose = initialPose;

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.addData("Status", "Initializing");
        this.telemetry.update();

        // Init Subsystems
        chassis = new Chassis(hardwareMap, this.telemetry);
        intake = new Intake(hardwareMap, this.telemetry);
        shooter = new Shooter(hardwareMap, this.telemetry);

        // Set up OpMode
        setupOpMode(type);
        this.telemetry.addData("Status", "Initialized");
        this.telemetry.update();
    }

    // Determine OpMode to run
    private void setupOpMode(OpModeType type) {
        switch (type) {
            case TELEOP:
                initTele();
                break;
            case AUTO:
                initAuto();
                break;
        }
    }

    // Initialize TeleOp scheduler
    private void initTele() {
        // Chassis: Driver left & right joysticks
        chassis.setDefaultCommand(
                new TeleOpDriveCommand(chassis)
        );

        // Shoot: Manip A button
        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ShootCommand(shooter));

        // Intake: Manip left & right bumpers
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake::in)
                .whenReleased(intake::stop);
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(intake::out)
                .whenReleased(intake::stop);

        // Deploy: Manip up & down dpad
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(intake::retract);
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intake::deploy);

        // Temp Debug:
        manipController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(intake::setupUpperConveyor)
                .whenPressed(intake::setUpLowerConveyor);
    }

    // Initialize Autonomous scheduler
    private void initAuto() {
        // TODO: Schedule auto path sequences
    }

    public void updateTelemetry() {
        this.telemetry.update();
    }
}