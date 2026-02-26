package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Commands.Turret.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name = "Turret Aimbot 100!", group = "Turret")
public class TurretAimbotTesting extends OpMode {
    Turret turret;
    MecanumDrivetrain mecanumDrivetrain;
    //This start is flush to the center of the goal
    Pose startingPose = new Pose(120,125,Math.toRadians(36));
    AimTurretCommand aimTurretCommand;
    Shooter shooter;

    @Override
    public void init() {
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,startingPose,false);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        aimTurretCommand = new AimTurretCommand(turret, mecanumDrivetrain, shooter, telemetry);
        CommandScheduler.getInstance().schedule(aimTurretCommand);
    }

    @Override
    public void loop() {
        //Move dt
        mecanumDrivetrain.processInputRC(gamepad1);
        CommandScheduler.getInstance().run();
        //log on field
        Drawing.drawDebug(mecanumDrivetrain.getFollower());

        telemetry.addLine("Turret ready?: " + turret.isTurretReady());
        telemetry.addLine("Turret angle: " + Utils.ras(turret.getCurrentPositionAsDegrees()));
        telemetry.addLine("Turret command: " + CommandScheduler.getInstance().isScheduled(aimTurretCommand));
        telemetry.update();
    }
}
