package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Commands.Turret.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name = "Turret Lock On Test", group = "Turret")
public class TurretLockOnTesting extends OpMode {
    Turret turret;
    MecanumDrivetrain mecanumDrivetrain;
    //This start is flush to the center of the goal
    Pose startingPose = new Pose(120,125,Math.toRadians(36));
    AimTurretCommand aimTurretCommand;
    Shooter shooter;
    double targetDegrees = Math.toDegrees(startingPose.getHeading());
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
        double rcTheta = targetDegrees - Math.toDegrees(mecanumDrivetrain.getFollower().getHeading());

        double normalizedDegrees = turret.normalizeDegrees(rcTheta);
        double pidfvalue = turret.runPIDFController(normalizedDegrees);

        boolean pLimBroke = turret.getCurrentPositionAsDegrees() > Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS;
        boolean nLimBroke = turret.getCurrentPositionAsDegrees() < Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS;

        //Move dt
        mecanumDrivetrain.processInputRC(gamepad1);
        turret.realSetTurretPositionAsDegrees(rcTheta);

        //CommandScheduler.getInstance().run();

        //log on field
        Drawing.drawDebug(mecanumDrivetrain.getFollower());
        telemetry.addLine("Turret ready?: " + turret.isTurretReady());
        telemetry.addLine();
        telemetry.addLine("Target Degrees: " + targetDegrees);
        telemetry.addLine("Turret Angle: " + Utils.ras(turret.getCurrentPositionAsDegrees()));
        telemetry.addLine("Turret Error: " + Utils.xDist(targetDegrees,turret.getCurrentPositionAsDegrees()));
        telemetry.addLine();
        telemetry.addLine("Heading Degrees: " + Math.toDegrees(mecanumDrivetrain.getFollower().getHeading()));
        telemetry.addLine("RC Theta: " + rcTheta);
        telemetry.addLine();
        telemetry.addLine("Degrees Post Normalization: " + normalizedDegrees);
        telemetry.addLine("Pos/Neg Limit Broken?: " + pLimBroke + "/" + nLimBroke);
        telemetry.addLine("PIDF Returned: " + pidfvalue);
        //telemetry.addLine("Turret command: " + CommandScheduler.getInstance().isScheduled(aimTurretCommand));
        telemetry.update();
    }
}
