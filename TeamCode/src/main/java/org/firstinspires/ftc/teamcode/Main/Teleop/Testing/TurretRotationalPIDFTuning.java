package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Rotational PIDF Tuning", group = "Tuning")
public class TurretRotationalPIDFTuning extends OpMode {
    TelemetryManager panelsTelemetry;
    Turret turret;
    public int pos = 0;
    public static int posFar = 1000;
    public static int posClose = -500;

    //TODO tune
    double P = 90;
    double I = 0;
    double D = 0;
    double F = 17.5;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();
        turret = new Turret(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void loop() {
        if (gamepad1.aWasPressed()) {
            if (pos == posClose) {
                pos = posFar;
            } else { pos = posClose; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.bWasPressed()) {
            D += stepSizes[stepIndex];
        }

        if (gamepad1.xWasPressed()) {
            D -= stepSizes[stepIndex];
        }

        turret.setTurretPosition(pos);
        turret.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(P, I, D, F));

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addData("Position", turret.getActualTurretPos());
        panelsTelemetry.addData("Positional target", pos);
        panelsTelemetry.addData("Error", (turret.getActualTurretPos() - pos));

        panelsTelemetry.addLine("P: " + P);
        panelsTelemetry.addLine("D: " + D);
        panelsTelemetry.addLine("F: " + F);

        panelsTelemetry.addLine("Adjustment Value: " + stepSizes[stepIndex]);

        panelsTelemetry.update(telemetry);

    }
}
