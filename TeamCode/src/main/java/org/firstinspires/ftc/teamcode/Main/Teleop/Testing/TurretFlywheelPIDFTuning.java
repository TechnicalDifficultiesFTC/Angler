package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
@Configurable
@TeleOp(name = "Turret Flywheel PIDF Tuning", group = "Tuning")
public class TurretFlywheelPIDFTuning extends OpMode {
    double curTargetVelocityAsRadians = 0;
    double curVelocityAsRadians;
    double error = 0;
    double highVelocityAsPercent = 100;
    double lowVelocityAsPercent = 50;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    double P = Config.TurretConstants.FlywheelPIDF.p;
    double I = Config.TurretConstants.FlywheelPIDF.i;
    double D = Config.TurretConstants.FlywheelPIDF.d;
    double F = Config.TurretConstants.FlywheelPIDF.f;
    Turret turret;
    String MOTM;
    TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        MOTM = Utils.generateMOTM();
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        //Switch between high velocity and low velocity
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocityAsRadians == Utils.velocityPercentToRadians(highVelocityAsPercent)) {
                curTargetVelocityAsRadians = Utils.velocityPercentToRadians(lowVelocityAsPercent);

            } else {
                curTargetVelocityAsRadians = Utils.velocityPercentToRadians(highVelocityAsPercent); }
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
            curTargetVelocityAsRadians = 0;
        }

        turret.flywheelMotor.setVelocityPIDFCoefficients(P,0,0,F);
        turret.setFlywheelTargetVelocityAsRadians(curTargetVelocityAsRadians);


        curVelocityAsRadians = turret.getFlywheelVelocityAsRadians();
        error = curTargetVelocityAsRadians - curVelocityAsRadians;

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Step Value: " + stepSizes[stepIndex]);
        panelsTelemetry.addData("Error ", Utils.ras(Math.abs(error)));
        panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsRadians));
        panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsRadians));

        panelsTelemetry.addData("P", turret.flywheelMotor.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).p);
        panelsTelemetry.addData("F", turret.flywheelMotor.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).f);

        panelsTelemetry.update(telemetry);

    }
}
