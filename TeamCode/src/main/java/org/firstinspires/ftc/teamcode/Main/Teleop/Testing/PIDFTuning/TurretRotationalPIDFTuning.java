package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.PIDFTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Rotational PIDF Tuning", group = "Turret Servo Stuff")
public class TurretRotationalPIDFTuning extends OpMode {
    TelemetryManager panelsTelemetry;
    Shooter shooter;
    public double posTarget = 0;
    public static double posPositive = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
    public static double posNegative = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);
    double P = Config.TurretConstants.TurretPIDF.p;
    double I = 0;
    double D = Config.TurretConstants.TurretPIDF.d;
    double F = Config.TurretConstants.TurretPIDF.f;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    String MOTM;
    Turret turret;

    public void init() {
        MOTM = Utils.generateMOTM();
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void loop() {

        /*
        ctrls:
        a: set new PIDF coeffiecients
        b: change step index
        options: set new target position

        dpad left: decrease F
        dpad right: increase F

        dpad down: decrease P
        dpad up: increase P

        y: increase D
        x: decrease D

         */
        if (gamepad1.backWasPressed()) {
            if (posTarget == posPositive) {
                posTarget = posNegative;
            } else { posTarget = posPositive; }
        }

        if (gamepad1.aWasPressed()) {
            turret.setPIDFCoeffiecients(new PIDFCoefficients(P,I,D,F));
        }

        //change step index
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        //decrement f
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        //increment f
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        //increment p
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        //decrement p
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        //increment d
        if (gamepad1.yWasPressed()) {
            D += stepSizes[stepIndex];
        }

        //decrement d
        if (gamepad1.xWasPressed()) {
            D -= stepSizes[stepIndex];
        }

        double actualPos = turret.getCurrentPositionAsDegrees();

        //set pos
        turret.setTurretPositionAsDegrees(posTarget);

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");

        panelsTelemetry.addData("Position ", Utils.ras(actualPos));
        panelsTelemetry.addData("Setpoint ", Utils.ras(posTarget));
        panelsTelemetry.addData("Error", Utils.ras(Utils.xDist(posTarget,actualPos)));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("P: " + P);
        panelsTelemetry.addLine("D: " + D);
        panelsTelemetry.addLine("F: " + F);
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Current PIDF: " + turret.getPIDFCoefficients().toString());
        panelsTelemetry.addLine("Current PIDF Power Value: " + Utils.ras(turret.runPIDFController(posTarget),3));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Adjustment Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addLine("Pos limit ticks: " + Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
        panelsTelemetry.addLine("Neg limit ticks: " + Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);
        panelsTelemetry.update(telemetry);

    }
}
