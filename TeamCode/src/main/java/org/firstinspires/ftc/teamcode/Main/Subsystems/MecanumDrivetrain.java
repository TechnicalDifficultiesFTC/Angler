package org.firstinspires.ftc.teamcode.Main.Subsystems;

import android.bluetooth.BluetoothClass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    Utils.Debounce sqaureDebounce = new Utils.Debounce();

    public double botHeading;

    double modulator;
    public String error;
    public String runmode;
    public DcMotor.ZeroPowerBehavior currentZeroPowerBehavior;

    IMU imu;
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        /**
         * Motors setup
         */
        frontLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_LEFT_MOTOR.str());
        backLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_LEFT_MOTOR.str());
        frontRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_RIGHT_MOTOR.str());
        backRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_RIGHT_MOTOR.str());

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * IMU CONFIG
         */
        imu = hardwareMap.get(IMU.class, DeviceRegistry.IMU.str());
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: Adjust parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        //Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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
        if (sqaureDebounce.isPressed(gamepad.square)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        y = -gamepad.left_stick_y; //Forward and back

        x = gamepad.right_stick_x; //Rotation

        rx = gamepad.left_stick_x; //Mecanum Strafe

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
        if (sqaureDebounce.isPressed(gamepad.square)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        y = -gamepad.left_stick_y; //Forward and back

        rx = gamepad.left_stick_x; //Mecanum Strafe

        x = gamepad.right_stick_x; //Rotation

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

    public boolean isLowPowerMode() {
        return lowPowerMode;
    }
}
