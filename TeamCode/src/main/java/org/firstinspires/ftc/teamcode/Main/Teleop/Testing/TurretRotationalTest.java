package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    public void init() {
        turret = new Turret(hardwareMap);
    }

    public void loop() {

        turret.setTurretPosition(pos);
        turret.setTurretPIDF(p,i,d,f);

        telemetry.addData("Position: ", turret.getTurretPos());
        telemetry.addData("Positional target: ", pos);
    }
}
