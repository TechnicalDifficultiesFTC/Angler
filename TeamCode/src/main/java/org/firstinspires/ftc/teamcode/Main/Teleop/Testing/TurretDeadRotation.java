package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Dead Rotation", group = "Manual Tests")
public class TurretDeadRotation extends OpMode {
    TelemetryManager panelsTelemetry;
    Turret turret;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();
        turret = new Turret(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(5);
    }

    public void loop() {
        telemetry.addLine("Position (Ticks): " + turret.getActualTurretPos());
        telemetry.addLine("Position (Degrees): " + Utils.turretTicksToDegrees(
                turret.getActualTurretPos()));

        telemetry.update();
        panelsTelemetry.update(telemetry);

    }
}
