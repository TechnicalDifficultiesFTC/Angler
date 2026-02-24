package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Helpers.VelocityMotorGroup;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

@Configurable
@TeleOp(name = "Flywheel Direction Test", group = "Testing/Composite")
public class FlywheelDirectionalTesting extends OpMode {
    double curTargetVelocityAsPercentage = 0;
    double error = 0;
    double highVelocityAsPercent = 80;
    double lowVelocityAsPercent = 35;
    Shooter shooter;
    String MOTM;
    TelemetryManager panelsTelemetry;
    VelocityMotorGroup flywheelMotors;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        MOTM = Utils.generateMOTM();
        shooter = new Shooter(hardwareMap);
        flywheelMotors = new VelocityMotorGroup(shooter.flywheelMotorLeft, shooter.flywheelMotorRight);
    }

    @Override
    public void loop() {
        //Switch between high velocity and low velocity

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocityAsPercentage == highVelocityAsPercent) {
                curTargetVelocityAsPercentage = lowVelocityAsPercent;

            } else {
                curTargetVelocityAsPercentage = highVelocityAsPercent; }
        }

        if (gamepad1.bWasPressed()) {
            flywheelMotors.setVelocityTarget(0);
        }

        if (gamepad1.aWasPressed()) {
            flywheelMotors.setVelocityTarget(Utils.velocityPercentToRadians(10));
        }
        if (gamepad1.dpadRightWasPressed()) {
            shooter.flywheelMotorLeft.setVelocity(Utils.velocityPercentToRadians(curTargetVelocityAsPercentage));
        }

        if (gamepad1.dpadLeftWasPressed()) {
            shooter.flywheelMotorRight.setVelocity(Utils.velocityPercentToRadians(curTargetVelocityAsPercentage));
        }

        double curVelocityAsPercentage = shooter.getFlywheelCurrentVelocityAsPercentage();
        error = curTargetVelocityAsPercentage - shooter.getFlywheelCurrentVelocityAsPercentage();

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Left Motor: " + Utils.ras(shooter.flywheelMotorLeft.getPower()));
        panelsTelemetry.addLine("Right Motor: " + Utils.ras(shooter.flywheelMotorRight.getPower()));
        panelsTelemetry.addLine("");
        panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsPercentage));
        panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsPercentage));
        panelsTelemetry.addLine("Ready?: " + shooter.isFlywheelReady());

        panelsTelemetry.update(telemetry);

    }
}
