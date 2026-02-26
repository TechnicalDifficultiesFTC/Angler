package org.firstinspires.ftc.teamcode.Main.Teleop.Testing.SensorsTesting;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
@Disabled
@TeleOp(name = "Odometry Feedback Testing", group = "Drivetrain")
public class OdometryFeedbackTest extends OpMode {
    GoBildaPinpointDriver goBildaPinpointDriver;
    MecanumDrivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new MecanumDrivetrain(hardwareMap, new Pose(), Config.GlobalConstats.defaultIsBlueValue);

    }

    @Override
    public void loop() {
        goBildaPinpointDriver.update();

        telemetry.addLine("pinpoint x: " + goBildaPinpointDriver.getEncoderX());
        telemetry.addLine("pinpoint y: " + goBildaPinpointDriver.getEncoderY());
//        Drawing.init();
//        Drawing.drawRobot();
    }
}
