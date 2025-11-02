package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

@TeleOp(name="Testing/RUN ME!!!", group="Test")
public class Testing extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    String MOTM = Utils.generateMOTM();

    @Override
    public void runOpMode() {
        //Construct DT
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);

        //Setup
        telemetry.setMsTransmissionInterval(10);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("INITIALIZED DRIVETRAIN");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //Subsystem calls
            mecanumDrivetrain.processInputFC(gamepad1);

           //Telemetry
            telemetry.addLine(MOTM);
            telemetry.addData(Config.DataCodes.lowPowerMode, mecanumDrivetrain.isLowPowerMode());
            telemetry.addLine("Run Mode = " + mecanumDrivetrain.runmode);
            telemetry.addLine("Bot Heading: " + Utils.roundAsDouble(
                    Math.toDegrees(mecanumDrivetrain.botHeading), 3));
            telemetry.update();
        }
    }
}
