package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Main.Commands.Shooter.AngleHoodCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Shooter.SpinupFlywheelCommand;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

public class PrimeShooter extends ParallelCommandGroup {
    //Im dependency injectioning all over the place!!! :)
    public PrimeShooter(Shooter shooter,
                        MecanumDrivetrain mecanumDrivetrain) {
        addCommands(
                new SpinupFlywheelCommand(shooter, mecanumDrivetrain),
                new AngleHoodCommand(shooter, mecanumDrivetrain)
        );
    }
}
