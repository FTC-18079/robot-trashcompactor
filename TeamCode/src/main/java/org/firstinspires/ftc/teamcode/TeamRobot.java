package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    // Subsystems
    Chassis chassis;
    Intake intake;
    Shooter shooter;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO
    }

    public TeamRobot(OpModeType type, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamePad1, Gamepad gamePad2) {
        this.hardwareMap = hardwareMap;
        this.driveController = new GamepadEx(gamePad1);
        this.manipController = new GamepadEx(gamePad2);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Init Subsystems
        chassis = new Chassis(hardwareMap, this.telemetry);
        intake = new Intake(hardwareMap, this.telemetry);
        shooter = new Shooter(hardwareMap, this.telemetry);

        // Set up OpMode
        setupOpMode(type);
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

    // TODO: Add more autos as needed

    // Initialize TeleOp scheduler
    private void initTele() {
        // TODO: Add default commands, button bindings, and triggers
        chassis.setDefaultCommand(
                new TeleOpDriveCommand(chassis)
        );

        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ShootCommand(shooter));

        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake::in);
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(intake::out);
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intake::retract);
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intake::deploy);
    }

    // Initialize Autonomous scheduler
    private void initAuto() {
        // TODO: Schedule auto path sequences
    }
}