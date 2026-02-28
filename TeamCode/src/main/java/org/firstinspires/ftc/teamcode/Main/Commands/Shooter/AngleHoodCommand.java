package org.firstinspires.ftc.teamcode.Main.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

import java.util.concurrent.TimeUnit;

public class AngleHoodCommand extends CommandBase {
    private final Shooter shooter;
    private final MecanumDrivetrain mecanumDrivetrain;
    double distance;
    double targetAngle;
    Timing.Timer hoodAngleTimer;
    public AngleHoodCommand(Shooter shooter, MecanumDrivetrain mecanumDrivetrain) {
        this.shooter = shooter;
        this.mecanumDrivetrain = mecanumDrivetrain;
    }

    @Override
    public void initialize() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetAngle = shooter.getHoodILUTValue(distance);

        shooter.setHoodAngleDegrees(targetAngle);

        //Start a timer based on estimated milliseconds to new target
        hoodAngleTimer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
        hoodAngleTimer.start();
    }

    @Override
    //Continue to recalculate based on distance in case we get pushed
    public void execute() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetAngle = shooter.getHoodILUTValue(distance);
        shooter.setHoodAngleDegrees(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return hoodAngleTimer.done();
    }
}
