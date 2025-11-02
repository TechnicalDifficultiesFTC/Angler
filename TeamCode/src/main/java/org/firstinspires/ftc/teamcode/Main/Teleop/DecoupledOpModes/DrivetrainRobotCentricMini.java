package org.firstinspires.ftc.teamcode.Main.Teleop.DecoupledOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="RC Mini", group="MiniModes")
public class DrivetrainRobotCentricMini extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLM");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLM");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRM");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRM");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double y,x,rx;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y; //Forward and back

            x = gamepad1.right_stick_x; //Rotation

            rx = gamepad1.left_stick_x; //Mecanum Strafe

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}