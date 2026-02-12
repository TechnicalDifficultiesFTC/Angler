package org.firstinspires.ftc.teamcode.Main.Commands.Shooter;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

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

    public void initialize() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetAngle = shooter.getHoodILUTValue(distance);

        double currentAngle = shooter.hoodServo.getPosition();
        double angularDistance = Utils.linearDist(currentAngle, targetAngle);

        shooter.setHoodAngle(targetAngle);

        //Convert distance to milliseconds
        long estimatedTravelTime = (long) (angularDistance/
                Config.ShooterConstants.TIME_TO_TRAVEL_POINT_ZERO_FIVE_ANGLE_MILLISECONDS);

        //Start a timer based on estimated milliseconds to new target
        hoodAngleTimer = new Timing.Timer(estimatedTravelTime);
        hoodAngleTimer.start();
    }

    //Continue to recalculate based on distance in case we get pushed
    public void execute() {
        distance = mecanumDrivetrain.getEstimatedDistanceToGoal();
        targetAngle = shooter.getHoodILUTValue(distance);
        shooter.setFlywheelTargetVelocityAsPercentage(targetAngle);
    }

    public boolean isFinished() {
        return hoodAngleTimer.done();
    }
}
