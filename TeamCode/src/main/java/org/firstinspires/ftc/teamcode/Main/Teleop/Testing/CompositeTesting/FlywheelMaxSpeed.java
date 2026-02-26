package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

@Disabled
@Configurable
@TeleOp(name = "Flywheel MAX Test", group = "Shooter")
public class FlywheelMaxSpeed extends OpMode {
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


        shooter.flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.flywheelMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.flywheelMotorLeft.setPower(1);
        shooter.flywheelMotorRight.setPower(1);


        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addData("Velocity ", Utils.ras(shooter.flywheelMotorLeft.getVelocity()));
        panelsTelemetry.addLine("Ready?: " + shooter.isFlywheelReady());

        panelsTelemetry.addData("P", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).p);
        panelsTelemetry.addData("F", shooter.flywheelMotorLeft.getPIDFCoefficients
                (DcMotor.RunMode.RUN_USING_ENCODER).f);

        panelsTelemetry.update(telemetry);

    }
}
