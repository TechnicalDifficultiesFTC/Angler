package org.firstinspires.ftc.teamcode.Main.Commands.Turret;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

/**
 * This command should run indefinitely
 */
public class AimTurretCommand extends CommandBase {
    Turret turret;
    Shooter shooter;
    MecanumDrivetrain mecanumDrivetrain;
    //distance of turret center to pose core
    double turretOffsetX = 0;
    double turretOffsetY = 0;
    double goalPositionX = 0;
    double goalPositionY = 0;
    Telemetry telemetry;
    public AimTurretCommand(Turret turret, MecanumDrivetrain mecanumDrivetrain, Shooter shooter, Telemetry telemetry) {
        this.shooter = shooter;
        this.telemetry = telemetry;
        this.turret = turret;
        this.mecanumDrivetrain = mecanumDrivetrain;
        if (mecanumDrivetrain.isBlue()) {
            goalPositionX = Config.FieldPositions.blueGoalX;
            goalPositionY = Config.FieldPositions.blueGoalY;
        } else {
            goalPositionX = Config.FieldPositions.redGoalX;
            goalPositionY = Config.FieldPositions.redGoalY;
        }
        addRequirements(turret);
    }

    public void initialize() {
        super.initialize();
    }

    public void execute() {
        Follower followerReference = mecanumDrivetrain.getFollower();

        followerReference.update();
        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();

        double positionX = goalPositionX - (followerReference.getPose().getX() + turretOffsetX);
        double positionY = goalPositionY - (followerReference.getPose().getY() + turretOffsetY);

        //Shooting on the fly stuff
        //double velocityX = followerReference.getVelocity().getXComponent();
        //double velocityY = followerReference.getVelocity().getYComponent();
        //telemetry.addLine("velocityX: " + velocityX);
        //telemetry.addLine("velocityY" + velocityY);
        double currentShooterHoodTicks = shooter.hoodServo.getPosition();
        double currentShooterHoodDegrees = shooter.getHoodExitDegrees(currentShooterHoodTicks);

        //TODO make hood angle ticks to degrees conversion
        telemetry.addLine("Hood angle: " + currentShooterHoodDegrees);

        double ballOutputWheelSpeed = shooter.getSpeedILUTValue(distance);
        double ballVelocity = ballOutputWheelSpeed * Math.cos(currentShooterHoodDegrees);

        telemetry.addLine("Ball vel: " + ballVelocity);
        double ballFlightTime = distance / ballVelocity;

        telemetry.addLine("Ball flight time: " + ballFlightTime);

        double targetX = (positionX / ballFlightTime);// - velocityX;
        double targetY = (positionY / ballFlightTime);// - velocityY;

        telemetry.addLine("targetX " + targetX);
        telemetry.addLine("targetY" + targetY);

        double theta;
        if (0 > targetX) {
            theta = Math.toDegrees(Math.atan(targetY/targetX)) + 180;
        } else {
            theta = Math.toDegrees(Math.atan(targetY/targetX));
        }

        telemetry.addLine("theta" + theta);
        //Make turret field centric
        double robotHeading = Math.toDegrees(followerReference.getHeading());

        telemetry.addLine("Robot heading " + robotHeading);
        telemetry.addLine("Robot desired angle " +  (theta - robotHeading));

        //turret.setTurretPositionAsDegrees(theta - robotHeading);
    }
}
