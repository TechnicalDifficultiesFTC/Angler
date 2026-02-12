package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;


@Configurable
@TeleOp(name = "Servo Indexer and Hood Test", group = "Testing/Composite")
public class ServoIndexerAndHoodTest extends OpMode {
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    double indexerServoPosition = 0;
    double shooterServoPosition = 0;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();

        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.setMsTransmissionInterval(5);
    }

    public void start() {
        indexer.setup();
    }

    public void loop() {
        //ctrls:
        /*
        x = +5 vel
        y = -5 vel

        dpad up = +5 hood angle
        dpad down = -5 hood angle

        -standard gp1 intake/indexer functions binded to gp2
         */
        if (gamepad1.xWasPressed()) {
            shooterServoPosition += .05;
        }
        if (gamepad1.yWasPressed()) {
            shooterServoPosition -= .05;
        }

        if (gamepad1.dpadUpWasPressed()) {
            indexerServoPosition += .005;

        }
        if (gamepad1.dpadDownWasPressed()) {
            indexerServoPosition -= .005;
        }

        indexer.indexerServo.setPosition(indexerServoPosition);
        shooter.hoodServo.setPosition(shooterServoPosition);

        //Other robot functions
        indexer.processInput(gamepad2,true);
        intake.processInput(gamepad2);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Hood Angle (software ticks): " + Utils.ras(shooter.hoodServo.getPosition()));
        telemetry.addLine("Indexer Servo Position (software ticks): " + Utils.ras(indexer.indexerServo.getPosition()));
    }
}
