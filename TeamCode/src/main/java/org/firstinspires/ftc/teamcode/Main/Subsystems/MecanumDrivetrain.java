package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class MecanumDrivetrain {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    double y;
    double x;
    double rx;
    double denominator;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    boolean lowPowerMode;
    Utils.Debounce aDebounce = new Utils.Debounce();

    public double botHeading;

    double modulator;
    public String error;
    public String runmode = "Null";
    public DcMotor.ZeroPowerBehavior currentZeroPowerBehavior;
    public GoBildaPinpointDriver pinpoint;

    IMU imu;
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        /* Motors */
        frontLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_LEFT_MOTOR.str());
        backLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_LEFT_MOTOR.str());
        frontRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_RIGHT_MOTOR.str());
        backRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_RIGHT_MOTOR.str());

        //Front Motors
        frontLeftMotor.setDirection(Config.Drivetrain.FLMD);
        frontRightMotor.setDirection(Config.Drivetrain.FRMD);

        //Back Motors
        backLeftMotor.setDirection(Config.Drivetrain.BLMD);
        backRightMotor.setDirection(Config.Drivetrain.BRMD);

        //ZPM
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Pinpoint */
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DeviceRegistry.PINPOINT.str());
        configurePinpoint();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        currentZeroPowerBehavior = zeroPowerBehavior;
    }

    /**
     * ROBOT-CENTRIC METHOD
     * -----------------------
     * Handle drivetrain logic and update motors as such
     * @param gamepad All input from gamepad (1)
     */
    public void processInputRC(Gamepad gamepad){
        runmode = "Robot-Centric";

        processInputPinpoint(gamepad);

        if (aDebounce.isPressed(gamepad.a)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        y = -gamepad.left_stick_y;
        x = gamepad.left_stick_x * 1.1;
        rx = gamepad.right_stick_x;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = ((y + x + rx)/denominator)*modulator;
        frontRightPower = ((y - x - rx)/denominator)*modulator;
        backLeftPower = ((y - x + rx)/denominator)*modulator;
        backRightPower = ((y + x - rx)/denominator)*modulator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * FIELD-CENTRIC METHOD
     * -----------------------
     * Handle drivetrain logic and update motors as such
     * @param gamepad All input from gamepad (1)
     */
    public void processInputFC(Gamepad gamepad) {

        runmode = "Field-Centric";

        processInputPinpoint(gamepad);

        if (aDebounce.isPressed(gamepad.square)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        y = -gamepad.left_stick_y;
        x = gamepad.left_stick_x * 1.1;
        rx = gamepad.right_stick_x;

        if (gamepad.options) {
            imu.resetYaw();
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX + rx) / denominator)*modulator;
        double backLeftPower = ((rotY - rotX + rx) / denominator)*modulator;
        double frontRightPower = ((rotY - rotX - rx) / denominator)*modulator;
        double backRightPower = ((rotY + rotX - rx) / denominator)*modulator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /* DEAD RECKONING CMDS */
    public void goForward(double pow,double actionLen) {
        frontLeftMotor.setPower(pow);
        backLeftMotor.setPower(pow);
        frontRightMotor.setPower(pow);
        backRightMotor.setPower(pow);

        Utils.halt((long) actionLen);
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public boolean isLowPowerMode() {
        return lowPowerMode;
    }

    /* PINPOINT */

    /**
     * Check controller for pinpoint re-zeroing and update pinpoint
     */
    public void processInputPinpoint(Gamepad gamepad) {
        if (gamepad.start) {
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
    }

    public void configurePinpoint() {
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(104, -104, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }
}
