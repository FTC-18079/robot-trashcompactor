package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryFollowerCommand extends CommandBase {
    private Chassis chassis;
    private TrajectorySequence trajectory;

    public TrajectoryFollowerCommand(Chassis chassis, TrajectorySequence trajectory) {
        this.chassis = chassis;
        this.trajectory = trajectory;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        chassis.update();
    }

    @Override
    public boolean isFinished() {
        return !chassis.isBusy();
    }
}
