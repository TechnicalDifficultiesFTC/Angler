package org.firstinspires.ftc.teamcode.Main.Commands.Turret;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

//Indefinite command
public class AimTurretCommand extends CommandBase {
    Turret turret;
    Shooter shooter;
    MecanumDrivetrain mecanumDrivetrain;
    //distance of turret center to pose core
    double turretOffsetX = 0;
    double turretOffsetY = 0;
    double goalPositionX = 0;
    double goalPositionY = 0;
    public AimTurretCommand(Turret turret, Shooter shooter, MecanumDrivetrain mecanumDrivetrain) {
        this.turret = turret;
        this.mecanumDrivetrain = mecanumDrivetrain;
        if (mecanumDrivetrain.isBlue()) {
            goalPositionX = Config.FieldPositions.blueGoalX;
            goalPositionY = Config.FieldPositions.blueGoalY;
        } else {
            goalPositionX = Config.FieldPositions.redGoalX;
            goalPositionY = Config.FieldPositions.redGoalY;
        }
    }

    public void initialize() {
        super.initialize();
    }

    public void execute() {
        Follower followerReference = mecanumDrivetrain.getFollower();

        followerReference.update();
        double distance = mecanumDrivetrain.getEstimatedDistanceToGoal();

        double positionX = goalPositionX - followerReference.getPose().getX();
        double positionY = goalPositionY - followerReference.getPose().getY();

        //Shooting on the fly stuff
        double velocityX = followerReference.getVelocity().getXComponent();
        double velocityY = followerReference.getVelocity().getYComponent();

        double ballOutputVelocity = shooter.getSpeedILUTValue(distance);
        double flightTime = distance / ballOutputVelocity;

        double targetX = (positionX / flightTime) - velocityX;
        double targetY = (positionY / flightTime) - velocityY;

        double theta;
        if (0 > targetX) {
            theta = Math.toDegrees(Math.atan(targetY/targetX)) + 180;
        } else {
            theta = Math.toDegrees(Math.atan(targetY/targetX));
        }

        //Make turret field centric
        double robotHeading = Math.toDegrees(followerReference.getHeading());
        turret.setCurrentPositionAsDegrees(theta - robotHeading);
    }
}
