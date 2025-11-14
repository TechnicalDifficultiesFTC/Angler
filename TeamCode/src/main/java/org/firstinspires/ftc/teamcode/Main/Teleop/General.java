package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name="GP Static Shooter v0.1 | Solo", group="Solo")
public class General extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();

    @Override
    public void runOpMode() {

        /*
        CONSTRUCTION!!!!!!!!!!!!!!!!!!!!!!!!!!
         */

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap); //Construct DT
        intake = new Intake(hardwareMap); //Construct Intake
        indexer = new Indexer(hardwareMap); //Construct Indexer
        turret = new Turret(hardwareMap); //Construct Turret

        //Setup
        telemetry.setMsTransmissionInterval(10);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("INITIALIZED DRIVETRAIN, INTAKE, INDEXER, TURRET :)");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //Subsystem calls
            mecanumDrivetrain.processInputRC(gamepad1); //DT
            intake.processInput(gamepad1); //Intake
            indexer.processInput(gamepad1); //Indexer
            turret.processInput(gamepad1); //Turret

            /* TELEMETRY!!!!! */
            telemetry.addLine(MOTM);

            //Drivetrain
            telemetry.addLine("DT: ");
            telemetry.addLine("\nLPM: " + mecanumDrivetrain.isLowPowerMode());
            telemetry.addLine("Run Mode = " + mecanumDrivetrain.runmode + " Behavior: " +
                    mecanumDrivetrain.currentZeroPowerBehavior);

            //Intake
            telemetry.addLine("INTAKE: " +
                    "\nIntake Power: " + intake.intakeMotor.getPower());

            //Indexer
            telemetry.addLine("INDEXER: " +
                    "\nStatus: " + indexer.indexingStatus);
            telemetry.addLine();
            //Turret
            telemetry.addLine( "TURRET: " +
                    "Flywheel ZPB: " + turret.getZeroPowerBehaviorAsString() +
                    "\nFlywheel Power: " + turret.getFlywheelPower() +
                    "\nHood Servo Power: " + turret.getHoodPower());
            telemetry.update();
        }
    }
}
