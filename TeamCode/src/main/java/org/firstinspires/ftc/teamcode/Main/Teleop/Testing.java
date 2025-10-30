package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

@TeleOp(name="Testing", group="Test")
public class Testing extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain = new MecanumDrivetrain();
    String MOTM = Utils.generateMOTM();

    @Override
    public void runOpMode() {
        //Setup
        telemetry.setMsTransmissionInterval(10);
        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("INITIALIZED DRIVETRAIN");

        while(opModeIsActive()) {
            //Subsystem calls
            mecanumDrivetrain.processInputRC(gamepad1);

            //Telemetry
            telemetry.addLine(MOTM);
            telemetry.addData(Config.DataCodes.lowPowerMode, mecanumDrivetrain.isLowPowerMode());
            telemetry.update();
        }
    }
}
