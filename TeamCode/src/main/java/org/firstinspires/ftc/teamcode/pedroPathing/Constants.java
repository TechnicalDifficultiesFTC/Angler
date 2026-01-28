package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(Config.DrivetrainConstants.ROBOT_MASS_KGS)
            .forwardZeroPowerAcceleration(-30.9171137580957064)
            .lateralZeroPowerAcceleration(-67.54516826509496)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0,0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.35,0,0.02,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0))
            .centripetalScaling(0.0005);

    //TODO look into breaking strength error during line test
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1);

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

            .leftFrontMotorDirection(Config.DrivetrainConstants.FLMD)
            .leftRearMotorDirection(Config.DrivetrainConstants.FRMD)
            .rightFrontMotorDirection(Config.DrivetrainConstants.FRMD)
            .rightRearMotorDirection(Config.DrivetrainConstants.FRMD)
            .useBrakeModeInTeleOp(true)

            .xVelocity(61.60857721764272)
            .yVelocity(48.005766469841905);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-104)
            .strafePodX(104)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(DeviceRegistry.PINPOINT.str())
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
