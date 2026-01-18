package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

@Configurable
public class Turret {
//    private double flywheelTargetVelocityPercentage = 70;
//    private double flywheelTargetVelocity = Config.TurretConstants.MAX_VELOCITY_RADIANS_PER_SEC *
//            ((flywheelTargetVelocityPercentage)*.01);

    //You can't retrieve target velocities in run to position PID mode so we track it internally
    //here!
    public double flywheelTargetVelocityAsRadians;
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -2;
    public boolean shooterRunning;
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;

    //CRServo hoodServo;
    public Servo hoodServo;

    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 144;
    public static double redGoalY  = 144;
    public static int pos = 0;


    public Turret(HardwareMap hardwareMap) {
        flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.TURRET_MOTOR.str());

        hoodServo = hardwareMap.servo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo.setDirection(Servo.Direction.FORWARD);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Turret motor setup
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Zero turret motor encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /* AUTOMATIC HANDLING */


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

    public void setTurretPosition(int pos) {
        //TODO Tune Power?? where did this come from lol
        turretMotor.setPositionPIDFCoefficients(Config.TurretConstants.POWER);
        turretMotor.setTargetPosition(pos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
    }

    public void setFlywheelTargetVelocityAsPercentage(double percent) {
        // Percent to radians
        double flywheelVelocity = Config.TurretConstants.MAX_VELOCITY_RADIANS_PER_SEC *
                ((percent)*.01);

        flywheelAccelerate(flywheelVelocity);
    }
    public void setFlywheelTargetVelocityAsRadians(double radians) {
        flywheelMotor.setVelocity(radians, AngleUnit.RADIANS);
    }

    /* Hood Methods */

    public void setHoodAngle(double pos) {
        hoodServo.setPosition(pos);
    }

    /* Flywheel Methods */
    private void flywheelAccelerate(double flywheelTargetVelocityAsRadians ) {
        shooterRunning = true;
        this.flywheelTargetVelocityAsRadians = flywheelTargetVelocityAsRadians;
        flywheelMotor.setVelocity(flywheelTargetVelocityAsRadians, AngleUnit.RADIANS);
    }

    private void flywheelDecelerate() {
        shooterRunning = false;
        flywheelMotor.setPower(0);
    }

    /* GETTERS!! */

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
        double vel = getFlywheelVelocityAsRadians();
        double targetVel = getFlywheelTargetVelocityAsRadians();
        return Utils.dist(vel, targetVel) <= Config.TurretConstants.FLYWHEEL_ERROR_MARGIN_RADS;
    }
    /**
     * @return Percent of total velocity achieved
     */
    public double getFlywheelVelocityAsPercentage() {
        return Utils.velocityRadiansToPercentage(getFlywheelVelocityAsRadians());
    }
    /**
     * @return Flywheel Velocity in param angleunit
     */
    public double getFlywheelVelocityAsRadians() {
        return flywheelMotor.getVelocity(AngleUnit.RADIANS);
    }
    public double getFlywheelTargetVelocityAsRadians() {
        return flywheelTargetVelocityAsRadians;
    }

    public double getFlywheelTargetVelocityAsPercentage() {
        return Utils.velocityRadiansToPercentage(getFlywheelTargetVelocityAsRadians());
    }

    public boolean getTurretReady() {
        double pos = turretMotor.getCurrentPosition();
        double targetPos = getTurretTargetPos();
        return Utils.dist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
    }
    public double getTurretTargetPos() {
        return turretMotor.getTargetPosition();
    }
    public int getActualTurretPos() {
        return turretMotor.getCurrentPosition();
    }
}
