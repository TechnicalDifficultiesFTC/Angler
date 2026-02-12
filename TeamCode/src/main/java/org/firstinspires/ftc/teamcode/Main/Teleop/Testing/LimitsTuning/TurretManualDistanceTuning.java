package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;


@Configurable
@TeleOp(name = "Turret Manual Distance Calibration", group = "Manual Tests")
public class TurretManualDistanceTuning extends OpMode {
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    double targetVelAsPercentage = 0;
    double hoodAngleTicks = 0;
    String MOTM;
    TelemetryManager panelsTelemetry;

    int shooterStepIndex = 0;
    double[] shooterStepSizes = {5,2.5,1,0.05};

    int hoodStepIndex = 0;
    double[] hoodStepSizes = {0.05,0.025,0.005};
    GoBildaPinpointDriver pinpointDriver;
    MecanumDrivetrain drivetrain;

    public void init() {
        MOTM = Utils.generateMOTM();

        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap, new Pose(),true);

        telemetry.setMsTransmissionInterval(5);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        indexer.setup();
        shooter.setup();
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
            shooter.setFlywheelTargetVelocityAsPercentage(targetVelAsPercentage); //set flywheel
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetVelAsPercentage -= shooterStepSizes[shooterStepIndex];
            shooter.setFlywheelTargetVelocityAsPercentage(targetVelAsPercentage); //set flywheel
        }

        if (gamepad1.dpadRightWasPressed()) {
            hoodAngleTicks += hoodStepSizes[hoodStepIndex];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            hoodAngleTicks -= hoodStepSizes[hoodStepIndex];
        }

        if (gamepad1.squareWasPressed()) {
            shooter.setFlywheelTargetVelocityAsPercentage(0);
        }

        //Manual turret setting functions

        shooter.setHoodAngle(hoodAngleTicks); //set hood

        //Other robot functions
        indexer.processInput(gamepad1,true);
        intake.processInput(gamepad1);

        telemetry.addLine("MOTM: " + MOTM);
        telemetry.addLine("Hood Angle (software ticks): " + Utils.ras(shooter.hoodServo.getPosition()));
        telemetry.addLine("Target percent: " + targetVelAsPercentage);
        telemetry.addLine();
        telemetry.addLine("Ready?: " + (shooter.isFlywheelReady() ? "OK" : "NO"));
        telemetry.addLine("Indexer Motor Power?: " + indexer.indexerMotor.getPower());
        telemetry.addLine("Indexer Status: " + indexer.getIndexingStatus());
        telemetry.update();


        panelsTelemetry.addData("Target Vel (%)", Utils.ras(shooter.getFlywheelTargetVelocityAsPercentage()));
        panelsTelemetry.addData("Setpoint (rads)", Utils.ras(shooter.flywheelTargetVelocityAsRadians));
        panelsTelemetry.addData("Velocity (rads)", Utils.ras(shooter.getFlywheelVelocityAsRadians()));
        panelsTelemetry.addData("Error", Math.abs(Utils.linearDist(shooter.flywheelTargetVelocityAsRadians, shooter.getFlywheelVelocityAsRadians())));
        panelsTelemetry.update();
    }
}
