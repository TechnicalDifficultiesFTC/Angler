package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@Configurable
@TeleOp(name="GP Static Shooter v1 | Solo", group="Solo")
public class General extends OpMode {
    //Declarations
    double percent = 50;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    TelemetryManager panelsTelemetry;



    @Override
    public void init() {
        /*
        CONSTRUCTION!!!!!!!!!!!!!!!!!!!!!!!!!!
         */

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap); //Construct DT
        intake = new Intake(hardwareMap); //Construct Intake
        indexer = new Indexer(hardwareMap); //Construct Indexer
        turret = new Turret(hardwareMap); //Construct Turret

        //Setup
        telemetry.setMsTransmissionInterval(5);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("INITIALIZED DRIVETRAIN, INTAKE, INDEXER, TURRET :)");
        telemetry.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

            //Subsystem calls
            mecanumDrivetrain.processInputFC(gamepad1); //DT
            indexer.processInput(gamepad1); //Indexer
            intake.processInput(gamepad1); //Intake

            //Controls:


            //Shooter testing
            if (gamepad1.dpadUpWasPressed()) {
                percent += 5;
            }

            if (gamepad1.dpadDownWasPressed()) {
                percent -= 5;
            }

            if (gamepad1.yWasPressed()) {
                turret.setFlywheelTargetVelocityAsPercentage(percent);
            }


//            // Shooter
//            if (gamepad1.xWasPressed()) {
//                turret.handleTurretRotation();
//                turret.handleFlywheelVelocity();
//            }


            double curVelocityAsRadians = turret.getFlywheelVelocityAsRadians();
            double curTargetVelocityAsRadians = turret.flywheelTargetVelocityAsRadians;
            double error = curTargetVelocityAsRadians - curVelocityAsRadians;


            /* TELEMETRY!!!!! */
            telemetry.addLine(MOTM);

            //Drivetrain
            telemetry.addLine("\nDT LPM: " + mecanumDrivetrain.isLowPowerMode());

            //Indexer
            telemetry.addLine("Indexer Status: " + indexer.getIndexingStatus());
            telemetry.addLine("Ball Seen?: " + indexer.ballHeld());
            panelsTelemetry.addData("Error ", Utils.ras(Math.abs(error)));
            panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsRadians));
            panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsRadians));

            telemetry.update();
            panelsTelemetry.update(telemetry);
    }
}
