package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Distance Tuning", group = "Tuning")
public class TurretDistanceTest extends OpMode {
    Turret turret;
    Indexer indexer;
    Intake intake;
    int targetVel = 0;
    double hoodAngleTicks = 0;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();

        turret = new Turret(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.setMsTransmissionInterval(5);
    }

    public void loop() {
        //ctrls:
        /*
        x = +5 vel
        y = -5 vel

        dpad up = +5 hood angle
        dpad down = -5 hood angle

        -standard gp1 intake/indexer functions binded to gp2
         */
        if (gamepad1.xWasPressed()) {
            targetVel += 5;
        }
        if (gamepad1.yWasPressed()) {
            targetVel -= 5;
        }

        if (gamepad1.dpadUpWasPressed()) {
            hoodAngleTicks += .05;

        }
        if (gamepad1.dpadDownWasPressed()) {
            hoodAngleTicks -= .05;
        }

        //Manual turret setting functions
        turret.setFlywheelTargetVelocityPercentage(targetVel);
        turret.setHoodAngle(hoodAngleTicks);
        turret.flywheelMotor.setPower(1);

        //Other robot functions
        indexer.processInput(gamepad1);
        intake.processInput(gamepad1);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Shooter running? " + turret.shooterRunning);
        telemetry.addLine("Shooter power: " + turret.flywheelMotor.getPower());
        telemetry.addLine("Hood Angle (software ticks): " + Utils.ras(turret.hoodServo.getPosition()));
        telemetry.addLine();
        telemetry.addLine("Target Velocity: " + Utils.ras(turret.getFlywheelTargetVelocityPercentage()));
        telemetry.addLine("Velocity %: " + Utils.ras(turret.getFlywheelVelocityPercentage()));
        telemetry.addLine();
        telemetry.addLine("Ready?: " + (turret.getFlywheelReady() ? "OK" : "NO"));
    }
}
