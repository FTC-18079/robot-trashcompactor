package org.firstinspires.ftc.teamcode.chassis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Chassis extends SubsystemBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    SampleMecanumDrive chassis;
    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize chassis as a new SampleMecanumDrive
        // This template assumes we will continue to use mecanum drive, and as such will not implement tank. Change this if needed.
        // TODO: Tune roadrunner DriveConstants and Localizer
        chassis = new SampleMecanumDrive(hardwareMap);
    }

    public void setPoseEstimate(Pose2d pose) {
        chassis.setPoseEstimate(pose);
    }

    public Pose2d getPoseEstimate() {
        return chassis.getPoseEstimate();
    }

    public void update() {
        chassis.update();
    }

    public void drive() {
        // TODO: Add driving code
    }

    public void followTrajectory(Trajectory trajectory) {
        chassis.followTrajectoryAsync(trajectory);
    }

    public void turn(double radians) {
        chassis.turn(radians);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return chassis.trajectorySequenceBuilder(startPose);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        chassis.followTrajectorySequence(trajectorySequence);
    }

    public boolean isBusy() {
        return chassis.isBusy();
    }
}
