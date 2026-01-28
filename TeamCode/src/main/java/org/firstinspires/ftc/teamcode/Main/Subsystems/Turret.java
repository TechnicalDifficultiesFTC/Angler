package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

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
    public double flywheelTargetVelocityAsRadians = 0;
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -2;
    public boolean shooterRunning = false;
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;

    //CRServo hoodServo;
    public Servo hoodServo;

    public static double blueGoalX = Config.FieldPositions.blueGoalX;
    public static double blueGoalY = Config.FieldPositions.blueGoalY;
    public static double redGoalX  = Config.FieldPositions.redGoalX;
    public static double redGoalY  = Config.FieldPositions.redGoalY;
    public static double distance;
    public static InterpLUT speedsLUT;
    public static InterpLUT hoodLUT;


    public Turret(HardwareMap hardwareMap) {
        //Motor declaration
        flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.TURRET_MOTOR.str());

        //Hood servo
        hoodServo = hardwareMap.servo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo.setDirection(Servo.Direction.FORWARD);

        //Flywheel setup
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocityPIDFCoefficients(Config.TurretConstants.FlywheelPIDF.p,0,0,Config.TurretConstants.FlywheelPIDF.f);

        //Turret motor setup
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Zero turret motor encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0); //Avoid TargetPositionNotSetException by setting pos to 0
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        constructILUTs();
    }

    public void constructILUTs() {
        /*
        Interpolated lookup table setup!
        All inputs are in inches
        Speed table outputs are in percentage of shooter speed
        Hood table outputs are in servo ticks
         */
        speedsLUT = new InterpLUT();
        hoodLUT = new InterpLUT();

        speedsLUT.add(24,80);
        speedsLUT.add(36,85);
        speedsLUT.add(48,90);
        speedsLUT.add(60,90);
        speedsLUT.add(72,95);
        speedsLUT.add(80,100);

        hoodLUT.add(24,0);
        hoodLUT.add(36,.2);
        hoodLUT.add(48,.4);
        hoodLUT.add(60,.45);
        hoodLUT.add(72,.45);
        hoodLUT.add(80,.5);


        //Construct ILUT's
        speedsLUT.createLUT();
        hoodLUT.createLUT();
    }
    public void setup() {
        setHoodAngle(0);
    }

    /* AUTOMATIC HANDLING */
    public void lockOnAsPosition(Pose botPose, boolean isBlue) {
        setTurretRotationAsPosition(botPose, isBlue);
        setFlywheelTargetVelocityAsPosition(botPose, isBlue);
    }

    public void setTurretRotationAsPosition(Pose botPose, boolean isBlue) {
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

        int turretTargetPosTicks = getTurretTicksAsDegrees(turretAngle);
        setTurretPositionAsTicks(turretTargetPosTicks);
    }

    public void setFlywheelTargetVelocityAsPosition(Pose botPose, boolean isBlue) {
        // ---- Pick correct goal ----
        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double x = botPose.getX();
        double y = botPose.getY();

        distance = Math.hypot(goalX - x, goalY - y);

        setFlywheelTargetVelocityAsDistance(distance);
    }
    public double getEstimatedDistanceToGoal(Pose botPose, boolean isBlue) {
        double goalX = isBlue ? blueGoalX : redGoalX;
        double goalY = isBlue ? blueGoalY : redGoalY;

        double x = botPose.getX();
        double y = botPose.getY();

        //true distance - offset so that we get the distance from the front of the robot
        distance = (Math.hypot(goalX - x, goalY - y)) + Config.TurretConstants.DISTANCE_OFFSET;
        return distance;
    }

    public void setFlywheelTargetVelocityAsDistance(double distance) {
        //double velocity = speedsLUT.get(distance);
        double velocity = 0;
        setFlywheelTargetVelocityAsRadians(velocity);
    }

    public void setTurretPositionAsTicks(int ticks) {
        turretMotor.setPositionPIDFCoefficients(Config.TurretConstants.TurretPIDF.p);
        turretMotor.setTargetPosition(ticks);
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

    public int getTurretTicksAsDegrees(double degrees) {
        //TODO test tslope
        int targetTicks = (int) (Config.TurretConstants.TICKSPERDEG * degrees);

        final int TURRET_MIN = (int) Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS;
        final int TURRET_MAX = (int) Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS;

        if (targetTicks > TURRET_MAX || targetTicks < TURRET_MIN) {
            return 0;
        } else {
            return targetTicks;
        }
    }

    public double getSpeedILUTValue(double distance) {
        distance = MathUtils.clamp(distance,Config.
                        TurretConstants.MIN_ILUT_DIST,
                Config.TurretConstants.MAX_ILUT_DIST);
        return speedsLUT.get(distance);
    }

    public double getHoodILUTValue(double distance) {
        distance = MathUtils.clamp(distance,Config.
                        TurretConstants.MIN_ILUT_DIST,
                Config.TurretConstants.MAX_ILUT_DIST);
        return speedsLUT.get(distance);
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

    public boolean getTurretReady() {
        double pos = turretMotor.getCurrentPosition();
        double targetPos = getTurretTargetPos();
        return Utils.dist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
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

    public boolean readyToFire() {
        return getTurretReady() && getFlywheelReady();
    }
    public double getTurretTargetPos() {
        return turretMotor.getTargetPosition();
    }
    public int getActualTurretPos() {
        return turretMotor.getCurrentPosition();
    }
}
