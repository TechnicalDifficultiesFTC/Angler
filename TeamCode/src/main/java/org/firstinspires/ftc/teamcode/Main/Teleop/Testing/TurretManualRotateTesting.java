package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@Configurable
@TeleOp(name = "Turret Manual Rotation Testing", group = "Manual Tests")
public class TurretManualRotateTesting extends OpMode {
    TelemetryManager panelsTelemetry;
    Turret turret;
    public int posTarget = 0;
    public static int posFar = 1000;
    public static int posClose = -500;
    double P = Config.TurretConstants.TurretPIDF.p;
    int stepIndex;
    double[] stepSizes = {5.0, 1.0, 0.1, 0.001, 0.0001};
    double targetDegrees = 0;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();
        turret = new Turret(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(5);
    }

    public void loop() {
        if (gamepad1.triangleWasPressed()) {
            turret.setTurretPositionAsTicks(Utils.turretDegreesToTicks(turret.normalizeAngleDegrees(targetDegrees)));
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.aWasPressed()) {
            targetDegrees += stepSizes[stepIndex];
        }

        if (gamepad1.xWasPressed()) {
            targetDegrees -= stepSizes[stepIndex];
        }
//        if (gamepad1.optionsWasPressed()) {
//
//        }

        double actualPosTicks = turret.getActualTurretPos();
        turret.turretMotor.setPositionPIDFCoefficients(P);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("");
        telemetry.addLine("Adjustment Value: " + stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addLine("Degrees Target: " + targetDegrees);
        telemetry.addLine("Degrees Traveled: " + Utils.turretTicksToDegrees(actualPosTicks));
        telemetry.addLine("Degrees Error: " + (targetDegrees - Utils.turretTicksToDegrees(actualPosTicks)));
        telemetry.addLine();
        telemetry.addLine("Actual ticks: " + actualPosTicks);
        telemetry.addLine("Ticks error: " + (Utils.turretDegreesToTicks(targetDegrees) - actualPosTicks));
        telemetry.addLine();

        telemetry.update();
        panelsTelemetry.update(telemetry);

    }
}
