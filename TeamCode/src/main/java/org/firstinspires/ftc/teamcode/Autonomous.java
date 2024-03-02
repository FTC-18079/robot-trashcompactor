package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.TransferData;

public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d();
        long waitMillis = 0;
        final TeamRobot robot = new TeamRobot(
                TeamRobot.OpModeType.AUTO,
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                initialPose
        );

        // Add auto delay
        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                waitMillis += 100;
            } else if (gamepad1.dpad_down) {
                waitMillis -= 100;
            }
        }

        waitForStart();
        sleep(waitMillis);

        // Run until end or stopped
        while (opModeIsActive() || !isStopRequested()) {
            robot.run();
            robot.updateTelemetry();
        }

        // Save pose data
        TransferData.currentPose = robot.chassis.getPoseEstimate();
        robot.reset();
    }
}
