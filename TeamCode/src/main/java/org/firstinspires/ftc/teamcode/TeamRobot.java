package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
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
                new TeleOpDriveCommand(
                        chassis,
                        () -> driveController.getLeftX(),
                        () -> driveController.getLeftY(),
                        () -> driveController.getRightX()
                )
        );
        driveController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(chassis::resetHeading);
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(chassis::toggleFieldCentric);



        // Shoot: Manip right trigger
        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
                .whenActive(new ShootCommand(shooter));

        // Toggle shooting: Manip right bumper
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(shooter::toggleShootingMode);

        // Intake: Manip left trigger
        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2)
                .whenActive(intake::in)
                .whenInactive(intake::stop);

        // Intake: Manip left bumper (change to right trigger in conjunction with shoot)
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake::in)
                .whenReleased(intake::stop);

        // Eject: Manip right d-pad
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(intake::out)
                .whenReleased(intake::stop);

        // Temp Debug:
        manipController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(intake::configConveyor)
                .whenPressed(intake::configIntake);

    }

    // Initialize Autonomous scheduler
    private void initAuto() {
        // TODO: Schedule auto path sequences
    }

    public void updateTelemetry() {
        this.telemetry.update();
    }
}