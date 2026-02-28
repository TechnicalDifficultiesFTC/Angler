package org.firstinspires.ftc.teamcode.Main.Commands.Turret;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
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

    // Rolling average variables
    double[] setPointSamples = new double[10];
    int sampleIndex = 0;
    boolean bufferFilled = false;
    public double rcTheta;

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
        boolean isBlue = mecanumDrivetrain.isBlue();
        Pose botPose = mecanumDrivetrain.getPose();

        double blueGoalX = Config.FieldPositions.blueGoalX;
        double blueGoalY = Config.FieldPositions.blueGoalY;
        double redGoalX  = Config.FieldPositions.redGoalX;
        double redGoalY  = Config.FieldPositions.redGoalY;

        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double robotX = botPose.getX();
        double robotY = botPose.getY();

        // Turret offset in robot frame (measure these from robot center to turret center)
        double turretOffsetX = -2.52; //to pedro x
        double turretOffsetY = -1.909; //to pedro y

        double robotHeadingDegrees = Math.toDegrees(mecanumDrivetrain.getPose().getHeading());
        double robotHeadingRadians = mecanumDrivetrain.getPose().getHeading();

        // Convert turret offset to field coordinates
        double turretFieldX = robotX + (turretOffsetX * Math.cos(robotHeadingRadians) -
                turretOffsetY * Math.sin(robotHeadingRadians));

        double turretFieldY = robotY + (turretOffsetX * Math.sin(robotHeadingRadians) +
                turretOffsetY * Math.cos(robotHeadingRadians));


        // Calculate angle from TURRET position to goal
        double deltaX = goalX - turretFieldX;
        double deltaY = goalY - turretFieldY;

        //telemetry.addLine("TF X/Y: " + Utils.ras(turretFieldX) + "/" + Utils.ras(turretFieldY));
        //telemetry.addData("Delta X/Y: ", Utils.ras(deltaX) + "/" + Utils.ras(deltaY));


        double fieldAngleToGoal = Math.toDegrees((Math.atan2(deltaY, deltaX))) ;
        rcTheta = fieldAngleToGoal - robotHeadingDegrees;

        //telemetry.addData("fieldAngleToGoal: ", Utils.ras(fieldAngleToGoal));

        turret.realSetTurretPositionAsDegrees(rcTheta);
    }

    @Override
    public boolean isFinished() { //this method will never end
        return false;
    }
}
