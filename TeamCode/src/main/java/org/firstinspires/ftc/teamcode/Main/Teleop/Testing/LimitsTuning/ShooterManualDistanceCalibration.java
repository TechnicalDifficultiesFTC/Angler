package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.LimitsTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Commands.Indexer.UpdateIndexerState;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;


@Configurable
@TeleOp(name = "Shooter Manual Distance Calibration", group = "Tuning")
public class ShooterManualDistanceCalibration extends OpMode {
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    double targetVelAsPercentage = 0;
    double hoodAngleDegs = 0;
    String MOTM;
    TelemetryManager panelsTelemetry;

    int shooterStepIndex = 0;
    double[] shooterStepSizes = {5,2.5,1,0.05};

    int hoodStepIndex = 0;
    double[] hoodStepSizes = {0.05,0.025,0.005};
    GoBildaPinpointDriver pinpointDriver;
    MecanumDrivetrain drivetrain;
    UpdateIndexerState updateIndexerState;

    public void init() {
        MOTM = Utils.generateMOTM();

        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap, new Pose(122.3151125401929, 123.61093247588425, Math.toRadians(36)),false);

        updateIndexerState = new UpdateIndexerState(indexer,intake,shooter,panelsTelemetry);
        telemetry.setMsTransmissionInterval(5);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        indexer.setup();
        shooter.setup();
        CommandScheduler.getInstance().schedule(updateIndexerState);
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
        if (gamepad2.xWasPressed()) {
            shooterStepIndex = (shooterStepIndex + 1) % shooterStepSizes.length;
        }

        if (gamepad2.yWasPressed()) {
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
            hoodAngleDegs += 2.5;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            hoodAngleDegs -= 2.5;
        }

        if (gamepad1.bWasPressed()) {
            shooter.setFlywheelTargetVelocityAsPercentage(0);
        }

        //Manual turret setting functions
        shooter.setHoodAngleDegrees(hoodAngleDegs);

        //Other robot functions
        intake.processInput(gamepad1);

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("Hood angle expected: " + hoodAngleDegs);
        panelsTelemetry.addLine("Hood Angle (degrees): " + Utils.ras(shooter.getHoodExitDegrees(shooter.hoodServo.getPosition())));
        panelsTelemetry.addLine("Target percent: " + targetVelAsPercentage);
        panelsTelemetry.addLine("Distance: " + drivetrain.getEstimatedDistanceToGoal());
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Ready?: " + (shooter.isFlywheelReady() ? "OK" : "NO"));
        panelsTelemetry.addLine("Indexer Motor Power?: " + indexer.indexerMotor.getPower());
        panelsTelemetry.addLine("Indexer Status: " + indexer.getIndexingStatus());
        panelsTelemetry.addLine("");
        panelsTelemetry.addData("Target Vel (%)", Utils.ras(shooter.getFlywheelTargetVelocityAsPercentage()));
        panelsTelemetry.addData("Setpoint (rads)", Utils.ras(shooter.flywheelTargetVelocityAsRadians));
        panelsTelemetry.addData("Velocity (rads)", Utils.ras(shooter.getFlywheelVelocityAsRadians()));
        panelsTelemetry.addData("Error", Math.abs(Utils.xDist(shooter.flywheelTargetVelocityAsRadians, shooter.getFlywheelVelocityAsRadians())));
        panelsTelemetry.update(telemetry);

        CommandScheduler.getInstance().run();
    }
}
