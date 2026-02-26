package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.LimitsTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Dead Rotation", group = "Tuning")
public class TurretDeadRotation extends OpMode {
    TelemetryManager panelsTelemetry;
    Shooter shooter;
    Turret turret;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.setMsTransmissionInterval(5);
    }

    public void loop() {
        double posLimit = Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS;
        double negLimit = Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS;
        double degsTraveled = turret.getCurrentPositionAsDegrees();
        telemetry.addLine("Position: " + turret.getCurrentPositionAsTicks());
        telemetry.addLine("Degs Traveled: " + Utils.ras(degsTraveled));

        telemetry.addLine("Turret Pos/Neg lims as ticks: " + posLimit + "/" + negLimit);
        telemetry.addLine("Past pos lim?: " + (degsTraveled > Utils.turretTicksToDegrees(posLimit)));
        telemetry.addLine("Past neg lim?: " + (degsTraveled < Utils.turretTicksToDegrees(negLimit)));

        telemetry.update();
        panelsTelemetry.update(telemetry);

    }
}
