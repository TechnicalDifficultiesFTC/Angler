package org.firstinspires.ftc.teamcode.Main.Commands.Groups;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Main.Commands.Shooter.AngleHoodCommand;
import org.firstinspires.ftc.teamcode.Main.Commands.Shooter.SpinupFlywheelCommand;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

/**
 * Changes flywheel and hood angle to align to goal
 */
public class PrimeShooter extends ParallelCommandGroup {
    //Im dependency injectioning all over the place!!! :)
    Shooter shooter;
    public PrimeShooter(Shooter shooter, MecanumDrivetrain mecanumDrivetrain) {
        this.shooter = shooter;
        addCommands(
                new SpinupFlywheelCommand(shooter, mecanumDrivetrain),
                new AngleHoodCommand(shooter, mecanumDrivetrain)
        );
    }

    public boolean isFinished() {
       return super.isFinished() && shooter.isFlywheelReady();
    }
}
