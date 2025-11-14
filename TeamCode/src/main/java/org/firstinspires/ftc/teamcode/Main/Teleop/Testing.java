package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

@TeleOp(name="Testing/RUN ME!!!", group="Test")
public class Testing extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    String MOTM = Utils.generateMOTM();

    @Override
    public void runOpMode() {
        //Construct DT
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);

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
            mecanumDrivetrain.processInputFC(gamepad1); //DT
            intake.processInput(gamepad1); //Intake



           //Telemetry
            telemetry.addLine(MOTM);
            //Drivetrain
            telemetry.addData(Config.DataCodes.lowPowerMode, mecanumDrivetrain.isLowPowerMode());
            telemetry.addLine("Run Mode = " + mecanumDrivetrain.runmode + " Behavior: " +
                    mecanumDrivetrain.currentZeroPowerBehavior);
            telemetry.addLine("Bot Heading: " + Utils.roundAsDouble(
                    Math.toDegrees(mecanumDrivetrain.botHeading), 3));

            //Intake
            telemetry.addLine("Intake Power: " + intake.intakeMotor.getPower());

            //Indexer
            telemetry.update();
        }
    }
}
