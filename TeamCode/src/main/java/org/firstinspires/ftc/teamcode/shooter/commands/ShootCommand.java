package org.firstinspires.ftc.teamcode.shooter.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;

    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;

        if(shooter.isShootingMode()) {
            addCommands(
                    new InstantCommand(shooter::enable),
                    new WaitUntilCommand(shooter::reachedTargetVel),
                    new InstantCommand(shooter::fire),
                    new WaitCommand(1500),
                    new InstantCommand(shooter::stop),
                    new InstantCommand(shooter::rest)
            );
        } else end(true);
        addRequirements(shooter);
    }
}