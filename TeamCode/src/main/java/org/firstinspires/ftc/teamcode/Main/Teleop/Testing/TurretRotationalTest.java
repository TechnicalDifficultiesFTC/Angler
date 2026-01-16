package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Rotational Tuning", group = "Tuning")
public class TurretRotationalTest extends OpMode {
    Turret turret;
    public static int pos = 0;

    //TODO tune
    public static double p = 90;
    public static double i = 0;
    public static double d = 0;
    public static double f = 17.5;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();
        turret = new Turret(hardwareMap);
    }

    public void loop() {

        turret.setTurretPosition(pos);
        //turret.setTurretPIDF(p,i,d,f);
        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Position: " + turret.getActualTurretPos());
        telemetry.addLine("Positional target: " + pos);
        telemetry.addLine("Error: " + (turret.getActualTurretPos() - pos));
    }
}
