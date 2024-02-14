package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

public class TurnCommand extends CommandBase {
    private final Chassis chassis;
    private final double angle;

    public TurnCommand(Chassis chassis, double angle) {
        this.chassis = chassis;
        this.angle = angle;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.turn(angle);
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
