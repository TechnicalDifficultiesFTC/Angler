package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

@TeleOp(name = "Field Centric Testing", group = "Testing/Systems")
public class FieldCentricTest extends OpMode {
    GoBildaPinpointDriver goBildaPinpointDriver;
    MecanumDrivetrain drivetrain;
    TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        goBildaPinpointDriver = drivetrain.getPinpoint();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        drivetrain.processInputFC(gamepad1);

        goBildaPinpointDriver.update();
        telemetry.addLine("pinpoint x: " + goBildaPinpointDriver.getEncoderX());
        telemetry.addLine("pinpoint y: " + goBildaPinpointDriver.getEncoderY());
        telemetry.addLine("pinpoint heading: " + goBildaPinpointDriver.getHeading(AngleUnit.DEGREES));
    }
}
