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
@TeleOp(name = "Turret Rotational PIDF Tuning | Standard", group = "PID")
public class TurretRotationalPIDFTuning extends OpMode {
    TelemetryManager panelsTelemetry;
    Shooter shooter;
    public double positionTarget = 0;
    public static double bigPositiveTarget = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
    public static double bigNegativeTarget = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);
    public static double smallPositiveTarget = Utils.turretTicksToDegrees((double) Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS/3);
    public static double smallNegativeTarget = Utils.turretTicksToDegrees((double) Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS/3);
    public static double positiveTarget = bigPositiveTarget;
    public static double negativeTarget = bigNegativeTarget;

    double P = Config.TurretConstants.TurretPIDFLarge.p;
    double I = Config.TurretConstants.TurretPIDFLarge.i;
    double D = Config.TurretConstants.TurretPIDFLarge.d;
    double F = Config.TurretConstants.TurretPIDFLarge.f;
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
        GP1:

        a: switch big/small targets
        b: change step index
        back: set new target position
        options: run to zero

        //PIDF 1
        dpad down: decrease P
        dpad up: increase P

        lb: increase I
        rb: decrease I

        y: increase D
        x: decrease D

        dpad left: decrease F
        dpad right: increase F

        //GP 2:
        dpad down: decrease P
        dpad up: increase P

        lb: increase I
        rb: decrease I

        y: increase D
        x: decrease D

        dpad left: decrease F
        dpad right: increase F
        */

        if (gamepad1.backWasPressed()) {
            boolean isPositiveTarget = positionTarget > negativeTarget;
            if (isPositiveTarget) {
                positionTarget = negativeTarget;

            } else { positionTarget = positiveTarget; }
        }

        turret.setPIDFCoeffiecients(new PIDFCoefficients(P,I,D,F));

        if (gamepad1.aWasPressed()) {
            //if we are on big targets
            if (positiveTarget == bigPositiveTarget) {
                //switch to small targets
                positiveTarget = smallPositiveTarget;
                negativeTarget = smallNegativeTarget;
            } else {
                //otherwise switch to big targets
                positiveTarget = bigPositiveTarget;
                negativeTarget = bigNegativeTarget;
            }
        }

        if (gamepad2.optionsWasPressed()) {
            positionTarget = 0;
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

        if (gamepad1.leftBumperWasPressed()) {
            I += stepSizes[stepIndex];
        }

        if (gamepad1.rightBumperWasPressed()) {
            I -= stepSizes[stepIndex];
        }

        double actualPos = turret.getCurrentPositionAsDegrees();

        //set pos
        turret.tuningSetTurretPositionAsDegrees(positionTarget);

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");

        panelsTelemetry.addData("Position ", Utils.ras(actualPos));
        panelsTelemetry.addData("Setpoint ", Utils.ras(positionTarget));
        panelsTelemetry.addData("Error", Utils.ras(turret.getPIDFController().getError()));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("P: " + Utils.ras(P,4));
        panelsTelemetry.addLine("I: " + Utils.ras(I,4));
        panelsTelemetry.addLine("D: " + Utils.ras(D,4));
        panelsTelemetry.addLine("F: " + Utils.ras(F,4));
        panelsTelemetry.addLine("F dir: " + (turret.getPIDFController().getTargetPosition() - turret.getCurrentPositionAsDegrees()));

        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Current PIDF: " + turret.getPIDFCoefficients().toString());
        panelsTelemetry.addLine("Current PIDF Power Value: " + Utils.ras(turret.runPIDFController(positionTarget),3));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Adjustment Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addLine("Pos limit ticks: " + Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
        panelsTelemetry.addLine("Neg limit ticks: " + Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);
        panelsTelemetry.update(telemetry);

    }
}
