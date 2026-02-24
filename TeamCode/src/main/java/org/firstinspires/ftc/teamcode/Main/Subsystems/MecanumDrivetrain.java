package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class MecanumDrivetrain extends SubsystemBase {
    boolean lowPowerMode;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    double lowPowerModeFactor;
    public String error;
    public String runmode = "Null";
    public DcMotor.ZeroPowerBehavior currentZeroPowerBehavior;
    Follower follower;
    Pose initialPose;
    boolean isBlue;

    /**
     * Update follower but not pinpoint
     */
    public void periodic() {
        follower.update();
        Drawing.drawDebug(follower);
    }

    public MecanumDrivetrain(HardwareMap hardwareMap, Pose initialPose, boolean isBlue) {
        this.initialPose = initialPose;
        this.isBlue = isBlue;

        /* Motors */
        frontLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_LEFT_MOTOR.str());
        backLeftMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_LEFT_MOTOR.str());
        frontRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.FRONT_RIGHT_MOTOR.str());
        backRightMotor = hardwareMap.dcMotor.get(DeviceRegistry.BACK_RIGHT_MOTOR.str());

        //Front Motors
        frontLeftMotor.setDirection(Config.DrivetrainConstants.FLMD);
        frontRightMotor.setDirection(Config.DrivetrainConstants.FRMD);

        //Back Motors
        backLeftMotor.setDirection(Config.DrivetrainConstants.BLMD);
        backRightMotor.setDirection(Config.DrivetrainConstants.BRMD);

        //ZPM
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Follower stuff
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initialPose);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        frontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        backRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        currentZeroPowerBehavior = zeroPowerBehavior;
    }

    public void setLowPowerMode(boolean lowPowerModeState) {
        lowPowerMode = lowPowerModeState;
    }
    /**
     * ROBOT-CENTRIC METHOD
     * -----------------------
     * Handle drivetrain logic and update motors as such
     * @param gamepad All input from gamepad (1)
     */
    public void processInputRC(Gamepad gamepad){

        runmode = "Robot-Centric";

        periodic();

        lowPowerModeFactor = lowPowerMode ? Config.DrivetrainConstants.MIN_DT_SPEED : Config.DrivetrainConstants.MAX_DT_SPEED;

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = ((y + x + rx)/denominator)* lowPowerModeFactor;
        double frontRightPower = ((y - x - rx)/denominator)* lowPowerModeFactor;
        double backLeftPower = ((y - x + rx)/denominator)* lowPowerModeFactor;
        double backRightPower = ((y + x - rx)/denominator)* lowPowerModeFactor;

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

        //re orient field centric
        if (gamepad.optionsWasPressed()) {
            //TODO look into posetracker offsets
            Pose currentPose = follower.getPose();

            follower.setPose(new Pose(currentPose.getX(),currentPose.getY(),0));
        }


        lowPowerModeFactor = lowPowerMode ? Config.DrivetrainConstants.MIN_DT_SPEED : Config.DrivetrainConstants.MAX_DT_SPEED;

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        //In radians
        double botHeading = follower.getHeading();

        //Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = ((rotY + rotX + rx) / denominator)* lowPowerModeFactor;
        double backLeftPower = ((rotY - rotX + rx) / denominator)* lowPowerModeFactor;
        double frontRightPower = ((rotY - rotX - rx) / denominator)* lowPowerModeFactor;
        double backRightPower = ((rotY + rotX - rx) / denominator)* lowPowerModeFactor;

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
    /* PINPOINT & FOLLOWER */

    public Follower getFollower() {
        return follower;
    }
    public Pose getPose() {
        periodic();
        return follower.getPose();
    }
    public boolean isBlue() {
        return isBlue;
    }
    public double getEstimatedDistanceToGoal() {
        Pose botPose = getPose();
        double blueGoalX = Config.FieldPositions.blueGoalX;
        double blueGoalY = Config.FieldPositions.blueGoalY;
        double redGoalX  = Config.FieldPositions.redGoalX;
        double redGoalY  = Config.FieldPositions.redGoalY;

        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double x = botPose.getX();
        double y = botPose.getY();

        //true distance + -offset so that we get the distance from the front of the robot
        return (Math.hypot((goalX - x), (goalY - y))) + Config.ShooterConstants.DISTANCE_OFFSET;
    }

}
