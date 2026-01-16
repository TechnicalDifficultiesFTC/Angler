package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
@TeleOp(name = "Turret Flywheel PIDF Tuning", group = "Tuning")
public class TurretFlywheelPIDFTuning extends OpMode {
    double curTargetVelocity;
    double highVelocity;
    double lowVelocity;
    int stepIndex;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    double P;
    double I;
    double D;
    double F;
    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
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

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        turret.flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidfCoefficients);

        turret.flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = turret.flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;
    }
}
