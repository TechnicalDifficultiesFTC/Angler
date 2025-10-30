package org.firstinspires.ftc.teamcode.Main.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    double modulator;

    IMU imu;
    public MecanumDrivetrain() {
        frontLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_LEFT_MOTOR.str());
        backLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_LEFT_MOTOR.str());
        frontRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_RIGHT_MOTOR.str());
        backRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_RIGHT_MOTOR.str());
        imu = hardwareMap.get(IMU.class, DeviceRegistry.IMU.str()); //FIXME: Untested

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * ROBOT-CENTRIC METHOD
     * -----------------------
     * Handle drivetrain logic and update motors as such
     * @param gamepad All input from gamepad (1)
     */
    public void processInputRC(Gamepad gamepad){

        if (sqaureDebounce.isPressed(gamepad.square)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        y = -gamepad.left_stick_y;
        x = gamepad.left_stick_x * 1.1;
        rx = gamepad.right_stick_x;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        frontLeftPower = ((y + x + rx)*modulator);
        frontRightPower = ((y - x - rx)*modulator);
        backLeftPower = ((y - x + rx)*modulator);
        backRightPower = ((y + x - rx)*modulator);

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
        if (sqaureDebounce.isPressed(gamepad.square)) {
            lowPowerMode = !lowPowerMode;
        }

        modulator = lowPowerMode ? Config.Drivetrain.MIN_DT_SPEED : Config.Drivetrain.MAX_DT_SPEED;

        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        if (gamepad.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
