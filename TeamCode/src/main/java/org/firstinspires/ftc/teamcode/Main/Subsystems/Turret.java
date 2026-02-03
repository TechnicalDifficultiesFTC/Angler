package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

@Configurable
public class Turret {

    public int targetTicks;
    public double flywheelTargetVelocityAsRadians = 0;
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -2;
    public boolean shooterRunning = false;
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;

    //CRServo hoodServo;
    public Servo hoodServo;

    public static InterpLUT speedsLUT;
    public static InterpLUT hoodLUT;
    public double targetAngleDeg;
    public double error;
    public Follower follower;


    public Turret(HardwareMap hardwareMap) {
        //Motor declaration

        flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.TURRET_MOTOR.str());

        //Hood servo
        hoodServo = hardwareMap.servo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo.setDirection(Servo.Direction.FORWARD);

        //Flywheel setup
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

        speedsLUT.add(18,70);
        speedsLUT.add(24,75);
        speedsLUT.add(36,80);
        speedsLUT.add(48,85);
        speedsLUT.add(60,90);
        speedsLUT.add(72,95);
        speedsLUT.add(80,100);
        speedsLUT.add(101.5,180);

        hoodLUT.add(18,0);
        hoodLUT.add(24,.15);
        hoodLUT.add(36,.3);
        hoodLUT.add(48,.45);
        hoodLUT.add(60,.55);
        hoodLUT.add(72,.55);
        hoodLUT.add(80,.6);
        hoodLUT.add(101.5,.6);

        //Construct ILUT's
        speedsLUT.createLUT();
        hoodLUT.createLUT();
    }
    public void setup() {
        setHoodAngle(0);
    }

    /* AUTOMATIC HANDLING */

    public double normalizeAngleDegrees(double angle) {
        // Wrap to [-180, 180)
        angle = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return angle;
    }


    public void setFlywheelTargetVelocityAsDistance(double distance) {
        double velocity = speedsLUT.get(distance);
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
    public double getSuggestedTurretAngle(double angleToGoal, double headingDegrees) {
        return (angleToGoal + headingDegrees - 90);
    }
    public int getTurretTicksAsDegrees(double degrees) {
        int targetTicks = (int) (Config.TurretConstants.TICKSPERDEG * degrees);
        return targetTicks;
    }

    public double getSpeedILUTValue(double distance) {
        distance = MathUtils.clamp(distance,
                Config.TurretConstants.MIN_ILUT_DIST+.1,
                Config.TurretConstants.MAX_ILUT_DIST-.1);
        return speedsLUT.get(distance);
    }
    public double getHoodILUTValue(double distance) {
        distance = MathUtils.clamp(distance,
                Config.TurretConstants.MIN_ILUT_DIST+.1,
                Config.TurretConstants.MAX_ILUT_DIST-.1);
        return hoodLUT.get(distance);
    }
    /**s
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
