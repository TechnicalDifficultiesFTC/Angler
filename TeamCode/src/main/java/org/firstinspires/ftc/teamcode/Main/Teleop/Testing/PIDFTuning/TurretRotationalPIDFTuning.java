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

    //Long
    double pL = Config.TurretConstants.TurretPIDFLarge.p;
    double iL = Config.TurretConstants.TurretPIDFLarge.i;
    double dL = Config.TurretConstants.TurretPIDFLarge.d;
    double fL = Config.TurretConstants.TurretPIDFLarge.f;

    //Small
    double pS = Config.TurretConstants.TurretPIDFSmall.p;
    double iS = Config.TurretConstants.TurretPIDFSmall.i;
    double dS = Config.TurretConstants.TurretPIDFSmall.d;
    double fS = Config.TurretConstants.TurretPIDFSmall.f;
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
        dpad down: decrease pL
        dpad up: increase pL

        lb: increase I
        rb: decrease I

        y: increase D
        x: decrease D

        dpad left: decrease F
        dpad right: increase F

        //GP 2:
        dpad down: decrease pL
        dpad up: increase pL

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

        if (gamepad1.optionsWasPressed()) {
            positionTarget = 0;
        }
        //change step index
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        //decrement f
        if (gamepad1.dpadLeftWasPressed()) {
            fL -= stepSizes[stepIndex];
        }

        //increment f
        if (gamepad1.dpadRightWasPressed()) {
            fL += stepSizes[stepIndex];
        }

        //increment p
        if (gamepad1.dpadUpWasPressed()) {
            pL += stepSizes[stepIndex];
        }

        //decrement p
        if (gamepad1.dpadDownWasPressed()) {
            pL -= stepSizes[stepIndex];
        }

        //increment d
        if (gamepad1.yWasPressed()) {
            dL += stepSizes[stepIndex];
        }

        //decrement d
        if (gamepad1.xWasPressed()) {
            dL -= stepSizes[stepIndex];
        }

        if (gamepad1.leftBumperWasPressed()) {
            iL += stepSizes[stepIndex];
        }

        if (gamepad1.rightBumperWasPressed()) {
            iL -= stepSizes[stepIndex];
        }

        /*
        SHORT RANGE PID TUNING
        ______________________
         */

        //decrement f
        if (gamepad2.dpadLeftWasPressed()) {
            fS -= stepSizes[stepIndex];
        }

        //increment f
        if (gamepad2.dpadRightWasPressed()) {
            fS += stepSizes[stepIndex];
        }

        //increment p
        if (gamepad2.dpadUpWasPressed()) {
            pS += stepSizes[stepIndex];
        }

        //decrement p
        if (gamepad2.dpadDownWasPressed()) {
            pS -= stepSizes[stepIndex];
        }

        //increment d
        if (gamepad2.yWasPressed()) {
            dS += stepSizes[stepIndex];
        }

        //decrement d
        if (gamepad2.xWasPressed()) {
            dS -= stepSizes[stepIndex];
        }

        if (gamepad2.leftBumperWasPressed()) {
            iS += stepSizes[stepIndex];
        }

        if (gamepad2.rightBumperWasPressed()) {
            iS -= stepSizes[stepIndex];
        }

        double actualPos = turret.getCurrentPositionAsDegrees();

        turret.setLongPIDFCoeffecients(new PIDFCoefficients(pL, iL, dL, fL));
        turret.setShortPIDFCoeffecients(new PIDFCoefficients(pS, iS, dS, fS));

        //set pos
        turret.tuningSetTurretPositionAsDegrees(positionTarget);

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");

        panelsTelemetry.addData("Position ", Utils.ras(actualPos));
        panelsTelemetry.addData("Setpoint ", Utils.ras(positionTarget));
        panelsTelemetry.addData("Error: ", Utils.ras(Utils.xDist(positionTarget,
                turret.getCurrentPositionAsDegrees())));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Using Short?: " + turret.usingShortRange);

        panelsTelemetry.addLine("pL: " + Utils.ras(pL,4));
        panelsTelemetry.addLine("iL: " + Utils.ras(iL,4));
        panelsTelemetry.addLine("dL: " + Utils.ras(dL,4));
        panelsTelemetry.addLine("fL: " + Utils.ras(fL,4));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("pS: " + Utils.ras(pS,4));
        panelsTelemetry.addLine("iS: " + Utils.ras(iS,4));
        panelsTelemetry.addLine("dS: " + Utils.ras(dS,4));
        panelsTelemetry.addLine("fS: " + Utils.ras(fS,4));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Current Long PIDF: " + turret.getLongPIDFCoefficients().toString());
        panelsTelemetry.addLine("Current Short PIDF: " + turret.getShortPIDFCoefficients().toString());
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Current PIDF Power Value: " + Utils.ras(turret.runPIDFControllers(positionTarget),3));
        panelsTelemetry.addLine("");

        panelsTelemetry.addLine("Adjustment Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addLine("Pos limit ticks: " + Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
        panelsTelemetry.addLine("Neg limit ticks: " + Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);
        panelsTelemetry.update(telemetry);

    }
}
