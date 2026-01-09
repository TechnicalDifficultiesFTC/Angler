package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    //TODO what is this
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)

            .rightFrontMotorName(DeviceRegistry.FRONT_RIGHT_MOTOR.str())
            .rightRearMotorName(DeviceRegistry.BACK_RIGHT_MOTOR.str())
            .leftRearMotorName(DeviceRegistry.BACK_LEFT_MOTOR.str())
            .leftFrontMotorName(DeviceRegistry.FRONT_LEFT_MOTOR.str())

            .leftFrontMotorDirection(Config.Drivetrain.FLMD)
            .leftRearMotorDirection(Config.Drivetrain.FRMD)
            .rightFrontMotorDirection(Config.Drivetrain.FRMD)
            .rightRearMotorDirection(Config.Drivetrain.FRMD)
            .useBrakeModeInTeleOp(true)
            //TODO what is this
            .yVelocity(55)
            .xVelocity(68);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-104)
            .strafePodX(104)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(DeviceRegistry.PINPOINT.str())
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
