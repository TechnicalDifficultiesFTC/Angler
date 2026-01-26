package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Distance Tuning", group = "Tuning")
public class TurretAutoDistanceTuning extends OpMode {
    Turret turret;
    Indexer indexer;
    Intake intake;
    double targetVelAsPercentage = 0;
    double hoodAngleTicks = 0;
    String MOTM;
    TelemetryManager panelsTelemetry;

    int shooterStepIndex = 0;
    double[] shooterStepSizes = {5,2.5,1,0.5};

    int hoodStepIndex = 0;
    double[] hoodStepSizes = {0.05,0.025,0.005};
    GoBildaPinpointDriver pinpointDriver;
    MecanumDrivetrain drivetrain;

    public void init() {
        MOTM = Utils.generateMOTM();

        turret = new Turret(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);

        pinpointDriver = drivetrain.getPinpoint();

        telemetry.setMsTransmissionInterval(5);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        indexer.setup();
        turret.setup();
    }

    @Override
    public void loop() {
        //ctrls:
        /*
        x = +5 vel
        y = -5 vel

        dpad up = +5 hood angle
        dpad down = -5 hood angle

        -standard gp1 intake/indexer functions binded to gp2
         */

        //Sensitivity toggling
        if (gamepad2.triangleWasPressed()) {
            shooterStepIndex = (shooterStepIndex + 1) % shooterStepSizes.length;
        }

        if (gamepad2.circleWasPressed()) {
            hoodStepIndex = (hoodStepIndex + 1) % hoodStepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetVelAsPercentage += shooterStepSizes[shooterStepIndex];
            turret.setFlywheelTargetVelocityAsPercentage(targetVelAsPercentage); //set flywheel
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetVelAsPercentage -= shooterStepSizes[shooterStepIndex];
            turret.setFlywheelTargetVelocityAsPercentage(targetVelAsPercentage); //set flywheel
        }

        if (gamepad1.dpadRightWasPressed()) {
            hoodAngleTicks += hoodStepSizes[hoodStepIndex];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            hoodAngleTicks -= hoodStepSizes[hoodStepIndex];
        }

        if (gamepad1.squareWasPressed()) {
            turret.setFlywheelTargetVelocityAsPercentage(0);
        }

        //Manual turret setting functions

        turret.setHoodAngle(hoodAngleTicks); //set hood

        //Other robot functions
        indexer.processInput(gamepad1);
        intake.processInput(gamepad1);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Shooter running? " + turret.shooterRunning);
        telemetry.addLine("Shooter power: " + turret.flywheelMotor.getPower());
        telemetry.addLine("Hood Angle (software ticks): " + Utils.ras(turret.hoodServo.getPosition()));
        telemetry.addLine();
        panelsTelemetry.addData("Target Vel (%)", Utils.ras(turret.getFlywheelTargetVelocityAsPercentage()));
        panelsTelemetry.addData("Setpoint (rads)", Utils.ras(turret.flywheelTargetVelocityAsRadians));
        panelsTelemetry.addData("Velocity (rads)", Utils.ras(turret.getFlywheelVelocityAsRadians()));
        panelsTelemetry.addData("Error", Math.abs(Utils.dist(turret.flywheelTargetVelocityAsRadians,turret.getFlywheelVelocityAsRadians())));
        telemetry.addLine();
        telemetry.addLine("Ready?: " + (turret.getFlywheelReady() ? "OK" : "NO"));
        telemetry.update();
        panelsTelemetry.update(telemetry);
    }
}
