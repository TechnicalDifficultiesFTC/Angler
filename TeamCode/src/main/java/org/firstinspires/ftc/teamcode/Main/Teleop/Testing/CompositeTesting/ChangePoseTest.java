package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.CompositeTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

@Configurable
@TeleOp(name = "Change Pose Test", group = "Testing/Composite")
public class ChangePoseTest extends OpMode {
    MecanumDrivetrain mecanumDrivetrain;
    String MOTM;
    TelemetryManager panelsTelemetry;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        MOTM = Utils.generateMOTM();
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap,new Pose(100,100,Math.toDegrees(45)),true);
    }

    @Override
    public void loop() {
        //Switch between high velocity and low velocity

        if (gamepad1.aWasPressed()) {
            mecanumDrivetrain.getFollower().setPose(new Pose());
        }

        if (gamepad1.bWasPressed()) {
            mecanumDrivetrain.getFollower().getPoseTracker().getLocalizer().setPose(new Pose());
        }

        panelsTelemetry.addLine("MOTM: " + MOTM);
        panelsTelemetry.addLine("");
        panelsTelemetry.addLine("Pose: " + mecanumDrivetrain.getPose().toString());

        panelsTelemetry.update(telemetry);

    }
}
