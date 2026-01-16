package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Servo Test", group = "Tuning")
public class ServoTest extends OpMode {
    Turret turret;
    Indexer indexer;
    Intake intake;
    double indexerServoPosition = 0;
    double shooterServoPosition = 0;
    String MOTM;

    public void init() {
        MOTM = Utils.generateMOTM();

        turret = new Turret(hardwareMap);
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
            indexerServoPosition += .05;

        }
        if (gamepad1.dpadDownWasPressed()) {
            indexerServoPosition -= .05;
        }

        indexer.indexerServo.setPosition(indexerServoPosition);
        turret.hoodServo.setPosition(shooterServoPosition);

        //Other robot functions
        indexer.processInput(gamepad2);
        intake.processInput(gamepad2);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Hood Angle (software ticks): " + Utils.ras(turret.hoodServo.getPosition()));
        telemetry.addLine("Indexer Servo Position (software ticks): " + Utils.ras(indexer.indexerServo.getPosition()));
    }
}
