package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Commands.Turret.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@Disabled
@TeleOp(name = "Turret Lock On Test", group = "Turret")
public class TurretLockOnTesting extends OpMode {
    Turret turret;
    MecanumDrivetrain mecanumDrivetrain;
    //This start is flush to the center of the goal
    Pose startingPose = new Pose(120,125,Math.toRadians(36));
    AimTurretCommand aimTurretCommand;
    Shooter shooter;
    double targetDegrees = Math.toDegrees(startingPose.getHeading());
    double rcTheta = 0;

    // Rolling average variables
    double[] setPointSamples = new double[10];
    int sampleIndex = 0;
    boolean bufferFilled = false;

    private double calculateRollingAverage(double newSample) {
        setPointSamples[sampleIndex] = newSample;
        sampleIndex = (sampleIndex + 1) % setPointSamples.length;

        if (sampleIndex == 0) {
            bufferFilled = true;
        }

        int count = bufferFilled ? setPointSamples.length : sampleIndex;
        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += setPointSamples[i];
        }
        return sum / count;
    }

    @Override
    public void init() {
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,startingPose,false);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

//      aimTurretCommand = new AimTurretCommand(turret, mecanumDrivetrain, shooter, telemetry);
//      CommandScheduler.getInstance().schedule(aimTurretCommand);
    }


    @Override
    public void loop() {
        boolean isBlue = false;
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
        telemetry.addLine("Delta X/Y: " + Utils.ras(deltaX) + "/" + Utils.ras(deltaY));


        double fieldAngleToGoal = Math.toDegrees((Math.atan2(deltaY, deltaX))) ;
        rcTheta = fieldAngleToGoal - robotHeadingDegrees;

        telemetry.addLine("fieldAngleToGoal: " + Utils.ras(fieldAngleToGoal));

        // Calculate rolling average of setpoint
        double averageTheta = calculateRollingAverage(rcTheta);
        turret.realSetTurretPositionAsDegrees(averageTheta);
        //Move dt
        mecanumDrivetrain.processInputRC(gamepad1);

        //CommandScheduler.getInstance().run();

        //log on field
        Drawing.drawDebug(mecanumDrivetrain.getFollower());
        telemetry.addLine("Using short?: " + turret.usingShortRange);
        telemetry.addLine("Average theta: " + averageTheta);
        telemetry.addLine();
        telemetry.addLine("Turret ready?: " + turret.isTurretReady());
        telemetry.addLine();
        telemetry.addLine("Target Degrees: " + Utils.ras(rcTheta));
        telemetry.addLine("Turret Angle: " + Utils.ras(turret.getCurrentPositionAsDegrees()));
        telemetry.addLine("Turret Error: " + Utils.ras(Utils.xDist(rcTheta,turret.getCurrentPositionAsDegrees())));
        telemetry.addLine();
        telemetry.addLine("Heading Degrees: " + Utils.ras(robotHeadingDegrees));
        telemetry.addLine();

        //double normalizedDegrees = turret.normalizeDegrees(rcTheta);
        //double pidfvalue = turret.runPIDFControllers(normalizedDegrees);

        boolean pLimBroke = turret.getCurrentPositionAsDegrees() > Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS;
        boolean nLimBroke = turret.getCurrentPositionAsDegrees() < Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS;
       // telemetry.addLine("Degrees Post Normalization: " + Utils.ras(normalizedDegrees));
        telemetry.addLine("Pos/Neg Limit Broken?: " + pLimBroke + "/" + nLimBroke);
        //telemetry.addLine("PIDF Returned: " + Utils.ras(pidfvalue));
        telemetry.addLine();
        telemetry.addLine("Long:" + turret.getLongPIDFCoefficients());
        telemetry.addLine("Short: " + turret.getShortPIDFCoefficients());
        //telemetry.addLine("Turret command: " + CommandScheduler.getInstance().isScheduled(aimTurretCommand));
        telemetry.update();
    }
}
