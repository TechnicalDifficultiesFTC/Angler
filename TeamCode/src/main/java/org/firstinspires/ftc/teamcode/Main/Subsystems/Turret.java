package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

@Configurable
public class Turret {
    private double flywheelTargetVelocityPercentage = 70;
    private double flywheelTargetVelocity = Config.TurretConstants.MAX_VELOCITY_RADIANS_PER_SEC *
            ((flywheelTargetVelocityPercentage)*.01);

    private int turretTargetPos = 0;

    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -2;
    boolean shooterRunning;
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;
    //CRServo hoodServo;
    public Servo hoodServo;
    Utils.Debounce leftArrowDebounce;
    Utils.Debounce rightArrowDebounce;
    Utils.Debounce yDebounce;
    Utils.Debounce xDebounce;

    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 144;
    public static double redGoalY  = 144;
    public static int pos = 0;

    private String rotationStatus = "";

    public Turret(HardwareMap hardwareMap) {
        flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.TURRET_MOTOR.str());

        //hoodServo = hardwareMap.crservo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo = hardwareMap.servo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo.setDirection(Servo.Direction.FORWARD);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Turret motor setup
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Zero turret motor encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArrowDebounce = new Utils.Debounce();
        rightArrowDebounce = new Utils.Debounce();

        yDebounce = new Utils.Debounce();
        xDebounce = new Utils.Debounce();
    }

    /* Input Handling */

    public String getRotationStatus() {
        return rotationStatus;
    }

    public void processInput(Gamepad gamepad2) {
        //handleHoodAngling(gamepad2);
        //handleTurretRotationAsGamepad(gamepad2);
        handleFlywheelTargetVelocity(gamepad2);
        handleFlywheel(gamepad2);
    }

//    private void handleTurretRotationAsGamepad(Gamepad gamepad) {
//        double turretPower = gamepad.right_stick_x;
//        turretMotor.setPower(turretPower/2);
//        if (turretMotor.getCurrentPosition() >=
//                Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS) {
//            rotationStatus = "hit positive rotation limit";
//        }
//        else if (turretMotor.getCurrentPosition() <=
//                Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS) {
//            rotationStatus = "hit negative rotation limit";
//        }
//        else { turretMotor.setPower(turretPower); }
//    }

    private void handleFlywheelTargetVelocity(Gamepad gamepad) {
        if (gamepad.xWasPressed()
            ) {
                flywheelTargetVelocityPercentage += 5;
        }
        else if (gamepad.yWasPressed()
                    && !(flywheelTargetVelocityPercentage <= 20) //Limit from going under 20% max speed
            ) {
                flywheelTargetVelocityPercentage -= 5;

        } else if (gamepad.backWasPressed()) {
                flywheelTargetVelocityPercentage = 70;
            }

        if (shooterRunning) { flywheelAccelerate(); }
    }

    private void handleFlywheel(Gamepad gamepad) {
        if (leftArrowDebounce.isPressed(gamepad.dpad_left)) {
            flywheelAccelerate();
        }
        else if (rightArrowDebounce.isPressed(gamepad.dpad_right)) { flywheelDecelerate(); }
    }

    public void setTurretPosition(int pos) {
        turretTargetPos = pos;
        turretMotor.setPositionPIDFCoefficients(Config.TurretConstants.POWER);
        turretMotor.setTargetPosition(pos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
    }
//    private void handleHoodAngling(Gamepad gamepad) {
//        if (gamepad.dpad_up) { raiseHood(); }
//        else if (gamepad.dpad_down) { lowerHood(); }
//        else { holdHood(); }
//    }

    public void setFlywheelTargetVelocityPercentage(double percent) {
        flywheelTargetVelocityPercentage = percent;
        flywheelAccelerate();
    }

    public void setTurretPIDF(double p, double i, double d, double f) {
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(p, i, d, f));
    }

    public void setFlywheelPIDF(double p, double i, double d, double f) {
        flywheelMotor.setVelocityPIDFCoefficients(p,i,d,f);
    }
    public void handleTurretRotation (Pose botPose, boolean isBlue) {
        double heading = botPose.getHeading();
        double x = botPose.getX();
        double y = botPose.getY();

        double headingDeg = Math.toDegrees(heading);

        x = turretOffsetX + x;
        y = turretOffsetY + y;

        // ---- Pick correct goal ----
        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));

        double turretAngle = angleToGoal + headingDeg - 90;

        //TODO tune tSlope
        //t slope = ticks/degs
        int targetTicks = (int) (Config.TurretConstants.TICKSPERDEG * turretAngle);

        final int TURRET_MIN = (int) Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS;
        final int TURRET_MAX = (int) Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS;

        if (targetTicks > TURRET_MAX || targetTicks < TURRET_MIN) {
            pos = 0;
        } else {
            pos = targetTicks;
        }

        setTurretPosition(pos);
    }

    public void handleFlywheelVelocity(Pose botPose, boolean isBlue) {
        // ---- Pick correct goal ----
        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double x = botPose.getX();
        double y = botPose.getY();

        double distance = Math.hypot(goalX - x, goalY - y);
        //double vel = (distance * fSlope + fIntercept);

        //flywheelMotor.setVelocity(vel);
    }

    /* Hood Methods */

    public void setHoodAngle(double pos) {
        hoodServo.setPosition(pos);
    }

    /* Flywheel Methods */
    private void flywheelAccelerate() {
        shooterRunning = true;
        flywheelTargetVelocity = Config.TurretConstants.MAX_VELOCITY_RADIANS_PER_SEC *
                ((flywheelTargetVelocityPercentage)*.01);

        flywheelMotor.setVelocity(flywheelTargetVelocity, AngleUnit.RADIANS);
    }

    private void flywheelDecelerate() {
        shooterRunning = false;
        flywheelMotor.setPower(0);
    }
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }
    public double getFlywheelAmps() {
        return flywheelMotor.getCurrent(CurrentUnit.AMPS);
    }


    /**
     * @return True if flywheel velocity is within an acceptable margin of error
     * Acceptable margin of error is defined in turret constants
     */
    public boolean getFlywheelReady() {
        double vel = getFlywheelVelocity(AngleUnit.RADIANS);
        double targetVel = getFlywheelTargetVelocity();
        return Utils.dist(vel, targetVel) <= Config.TurretConstants.FLYWHEEL_ERROR_MARGIN_RADS;
    }

    /**
     * @return Percent of total velocity achieved
     */
    public double getFlywheelVelocityPercentage() {
        return (getFlywheelVelocity(AngleUnit.RADIANS)/
                Config.TurretConstants.MAX_VELOCITY_RADIANS_PER_SEC)*100;
    }

    /**
     * @return Flywheel Velocity in param angleunit
     */
    public double getFlywheelVelocity(AngleUnit angleUnit) {
        return flywheelMotor.getVelocity(angleUnit);
    }

    public double getFlywheelTargetVelocity() {
        return flywheelTargetVelocity;
    }

    public boolean getTurretReady() {
        double pos = turretMotor.getCurrentPosition();
        double targetPos = getTurretTargetPos();
        return Utils.dist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
    }

    public double getTurretTargetPos() {
        return turretTargetPos;
    }

    public int getActualTurretPos() {
        return turretMotor.getCurrentPosition();
    }
    public int getTargetTurretPos() { return turretTargetPos; }
    public double getFlywheelTargetVelocityPercentage() {
        return flywheelTargetVelocityPercentage;
    }
}
