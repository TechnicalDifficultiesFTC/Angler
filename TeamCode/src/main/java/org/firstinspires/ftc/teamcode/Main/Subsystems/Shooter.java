package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.ShooterTracker;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Helpers.VelocityMotorGroup;

@Configurable
public class Shooter extends SubsystemBase {

    public int targetTicks;
    public double flywheelTargetVelocityAsRadians = 0;
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -2;
    public boolean shooterRunning = false;
    public DcMotorEx flywheelMotorLeft;
    public DcMotorEx flywheelMotorRight;
    public VelocityMotorGroup flywheelMotorGroup;

    //CRServo hoodServo;
    public Servo hoodServo;

    public static InterpLUT speedsLUT;
    public static InterpLUT hoodLUT;
    public double targetAngleDeg;
    public double error;
    public Follower follower;
    private ShooterTracker shooterTracker = new ShooterTracker();


    public Shooter(HardwareMap hardwareMap) {
        //Motor declaration
        flywheelMotorLeft = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR_LEFT.str());
        flywheelMotorRight = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR_RIGHT.str());

        //Hood servo
        hoodServo = hardwareMap.servo.get(DeviceRegistry.HOOD_SERVO.str());
        hoodServo.setDirection(Servo.Direction.FORWARD);

        //Flywheel setup
        //Left motor
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorLeft.setVelocityPIDFCoefficients(Config.ShooterConstants.FlywheelPIDF.p,0,0,Config.ShooterConstants.FlywheelPIDF.f);

        //Right motor
        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorRight.setVelocityPIDFCoefficients(Config.ShooterConstants.FlywheelPIDF.p,0,0,Config.ShooterConstants.FlywheelPIDF.f);

        flywheelMotorGroup = new VelocityMotorGroup(flywheelMotorLeft, flywheelMotorRight);
        constructILUTs();
    }

    public void periodic() {
        super.periodic();
    }

    //TODO make a method that can determine if a ball has been fired by checking the inertia drop
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

    public void setFlywheelTargetVelocityAsDistance(double distance) {
        double velocity = speedsLUT.get(distance);
        setFlywheelTargetVelocityAsRadians(velocity);
    }

    public void setFlywheelMotorLeftVelocityAsRadians(double radians) {
        flywheelMotorLeft.setVelocity(radians,AngleUnit.RADIANS);
    }

    public void setFlywheelMotorRightVelocityAsRadians(double radians) {
        flywheelMotorRight.setVelocity(radians,AngleUnit.RADIANS);
    }

    public void setFlywheelTargetVelocityAsPercentage(double percent) {
        // Percent to radians
        flywheelTargetVelocityAsRadians = Config.ShooterConstants.MAX_VELOCITY_RADIANS_PER_SEC *
                ((percent)*.01);

    }

    public void setFlywheelTargetVelocityAsRadians(double radians) {
        flywheelMotorLeft.setVelocity(radians, AngleUnit.RADIANS);
    }

    /* Hood Methods */

    public void setHoodAngle(double pos) {
        hoodServo.setPosition(pos);
    }

    private void flywheelDecelerate() {
        shooterRunning = false;
        flywheelMotorLeft.setPower(0);
    }


    /* GETTERS!! */

    public double getSpeedILUTValue(double distance) {
        distance = MathUtils.clamp(distance,
                Config.ShooterConstants.MIN_ILUT_DIST+.001,
                Config.ShooterConstants.MAX_ILUT_DIST-.001);
        return speedsLUT.get(distance);
    }
    public double getHoodILUTValue(double distance) {
        distance = MathUtils.clamp(distance,
                Config.ShooterConstants.MIN_ILUT_DIST+.001,
                Config.ShooterConstants.MAX_ILUT_DIST-.001);
        return hoodLUT.get(distance);
    }
    /**s
     * @return True if flywheel velocity is within an acceptable margin of error
     * Acceptable margin of error is defined in turret constants
     */
    public boolean isFlywheelReady() {
        double vel = getFlywheelVelocityAsRadians();
        double targetVel = getFlywheelTargetVelocityAsRadians();
        return Utils.linearDist(vel, targetVel) <= Config.ShooterConstants.FLYWHEEL_ERROR_MARGIN_RADS;
    }

    /**
     * @return Percent of total velocity achieved
     */
    public double getFlywheelCurrentVelocityAsPercentage() {
        return Utils.velocityRadiansToPercentage(getFlywheelVelocityAsRadians());
    }
    /**
     * @return Flywheel Velocity in param angleunit
     */
    public double getFlywheelVelocityAsRadians() {
        return flywheelMotorLeft.getVelocity(AngleUnit.RADIANS);
    }
    public double getFlywheelTargetVelocityAsRadians() {
        return flywheelTargetVelocityAsRadians;
    }
    public double getFlywheelTargetVelocityAsPercentage() {
        return Utils.velocityRadiansToPercentage(getFlywheelTargetVelocityAsRadians());
    }
    public ShooterTracker getShooterTracker() {
        return shooterTracker;
    }
}
