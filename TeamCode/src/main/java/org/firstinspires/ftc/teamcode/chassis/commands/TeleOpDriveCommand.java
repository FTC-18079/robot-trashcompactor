package org.firstinspires.ftc.teamcode.chassis.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.chassis.Chassis;

public class TeleOpDriveCommand extends CommandBase {
    private final Chassis chassis;

    public TeleOpDriveCommand(Chassis chassis) {
        this.chassis = chassis;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        // TODO: Add init code here
    }

    @Override
    public void execute() {
        chassis.drive();
    }
}
