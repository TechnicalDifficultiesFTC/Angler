package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

@Configurable
@TeleOp(name="Ball Held Test", group="Testing/Sensors")
public class BallHeldTest extends OpMode {
    //Declarations
    double percent = 50;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    String MOTM = Utils.generateMOTM();
    TelemetryManager panelsTelemetry;



    @Override
    public void init() {
        /*
        CONSTRUCTION!!!!!!!!!!!!!!!!!!!!!!!!!!
         */

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, new Pose(), true); //Construct DT
        intake = new Intake(hardwareMap); //Construct Intake
        indexer = new Indexer(hardwareMap); //Construct Indexer
        shooter = new Shooter(hardwareMap); //Construct Turret

        //Setup
        telemetry.setMsTransmissionInterval(5);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        indexer.setup();
    }
    @Override
    public void loop() {

        //Subsystem calls
        mecanumDrivetrain.processInputFC(gamepad1); //DT
        indexer.processInput(gamepad1,true); //Indexer
        intake.processInput(gamepad1); //Intake


        /* TELEMETRY!!!!! */
        telemetry.addLine(MOTM);
        telemetry.addLine();

        //Indexer
        telemetry.addLine("Indexer Status: " + indexer.getIndexingStatus());
        telemetry.addLine("Ball Seen?: " + indexer.ballDetected());
        telemetry.addLine("Distance Reported: " + indexer.getDistanceReported());
        telemetry.addLine("Arm in the way?: " + indexer.isArmInTheWay());
        telemetry.addLine("Is Ball Blocked?: " + indexer.isBallBlocked());
        telemetry.addLine("Did auto recover?: "+ indexer.didAutoRecover);
        telemetry.addLine("Indexer Power: " + indexer.indexerMotor.getPower());
        telemetry.addLine("Indexer reversing: " + indexer.indexerReversing);

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }
}
