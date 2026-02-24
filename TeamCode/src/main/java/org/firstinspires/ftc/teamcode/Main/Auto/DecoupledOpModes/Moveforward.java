package org.firstinspires.ftc.teamcode.Main.Auto.DecoupledOpModes;


import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

@Autonomous(name="MOVE FORWARD AUTO", group="MiniModes")
public class Moveforward extends LinearOpMode {
    String MOTM = Utils.generateMOTM();
    MecanumDrivetrain mecanumDrivetrain;
    boolean setup = false;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!setup) {
            setup();
            setup = true;
        }

        waitForStart();
        if(isStopRequested()) return;

        mecanumDrivetrain.goForward(.5,5000);
        mecanumDrivetrain.stopMotors();
    }

    private void setup() {
        telemetry.addLine(MOTM);
        telemetry.update();
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,new Pose(), Config.GlobalConstats.defaultIsBlueValue);
    }
}

