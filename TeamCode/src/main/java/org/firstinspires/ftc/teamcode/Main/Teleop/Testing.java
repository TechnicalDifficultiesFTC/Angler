package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name="Testing/RUN ME!!!", group="Test")
public class Testing extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    double velocity;
    double amps;
    double maxVel = 0;

    @Override
    public void runOpMode() {
        //Construct DT
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        turret = new Turret(hardwareMap);

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
            //mecanumDrivetrain.processInputFC(gamepad1); //DT
            intake.processInput(gamepad1);
            indexer.processInput(gamepad1);
            turret.processInput(gamepad2);



           //Telemetry
            telemetry.addLine(MOTM);
            //Drivetrain
//            telemetry.addData(Config.DataCodes.lowPowerMode, mecanumDrivetrain.isLowPowerMode());
//            telemetry.addLine("Run Mode = " + mecanumDrivetrain.runmode + " Behavior: " +
//                    mecanumDrivetrain.currentZeroPowerBehavior);
//            telemetry.addLine("Bot Heading: " + Utils.roundAsDouble(
//                    Math.toDegrees(mecanumDrivetrain.botHeading), 3));

            //Intake
            telemetry.addLine("Intake Power: " + intake.intakeMotor.getPower());

            //Turret
            velocity = turret.getFlywheelVelocity(AngleUnit.RADIANS);
            amps = turret.getFlywheelAmps();

            if (velocity > maxVel) {
                maxVel = velocity;
            }

            telemetry.addLine("\nFlywheel Rads/Secs: " + velocity);
            telemetry.addLine("Max Velocity Rads/Secs: " + maxVel);
            telemetry.addLine("Flywheel Amps: " + amps);

            //Indexer
            telemetry.update();
        }
    }
}
