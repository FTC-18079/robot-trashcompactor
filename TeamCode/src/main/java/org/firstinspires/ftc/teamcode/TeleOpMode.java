package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.TransferData;

@TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = TransferData.currentPose;

        // Instantiate robot
        TeamRobot robot = new TeamRobot(
                TeamRobot.OpModeType.TELEOP,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2
        );

        waitForStart();

        // Run until end or stopped
        while (opModeIsActive() || !isStopRequested()) {
            robot.run();
        }

        // Reset currentPose in case opmode stops early
        TransferData.currentPose = robot.chassis.getPoseEstimate();
        robot.reset();
    }
}
