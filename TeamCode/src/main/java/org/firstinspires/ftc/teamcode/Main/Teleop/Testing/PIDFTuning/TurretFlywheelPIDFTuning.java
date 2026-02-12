package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
@Configurable
@TeleOp(name = "Turret Flywheel PIDF Tuning", group = "Tuning")
public class TurretFlywheelPIDFTuning extends OpMode {
    double curTargetVelocityAsPercentage = 0;
    double error = 0;
    double highVelocityAsPercent = 200;
    double lowVelocityAsPercent = 100;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    double P = Config.ShooterConstants.FlywheelPIDF.p;
    double I = Config.ShooterConstants.FlywheelPIDF.i;
    double D = Config.ShooterConstants.FlywheelPIDF.d;
    double F = Config.ShooterConstants.FlywheelPIDF.f;
    Shooter shooter;
    String MOTM;
    TelemetryManager panelsTelemetry;
    Intake intake;
    Indexer indexer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        MOTM = Utils.generateMOTM();
        shooter = new Shooter(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        //Switch between high velocity and low velocity

        intake.processInput(gamepad1);
        indexer.processInput(gamepad1,true);

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocityAsPercentage == highVelocityAsPercent) {
                curTargetVelocityAsPercentage = lowVelocityAsPercent;

            } else {
                curTargetVelocityAsPercentage = highVelocityAsPercent; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.squareWasPressed()) {
            curTargetVelocityAsPercentage = 0;
        }

        shooter.flywheelMotorLeft.setVelocityPIDFCoefficients(P,0,0,F);
        shooter.setFlywheelTargetVelocityAsPercentage(curTargetVelocityAsPercentage);

        double curVelocityAsPercentage = shooter.getFlywheelCurrentVelocityAsPercentage();
        error = curTargetVelocityAsPercentage - shooter.getFlywheelCurrentVelocityAsPercentage();

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Step Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addData("Error ", Utils.ras(Math.abs(error)));
        panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsPercentage));
        panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsPercentage));
        panelsTelemetry.addLine("Ready?: " + shooter.isFlywheelReady());

        panelsTelemetry.addData("P", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).p);
        panelsTelemetry.addData("F", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).f);

        panelsTelemetry.update(telemetry);

    }
}
