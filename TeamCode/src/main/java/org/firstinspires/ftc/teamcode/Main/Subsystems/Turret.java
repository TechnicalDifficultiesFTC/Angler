package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    boolean shooterRunning;
    public DcMotorEx turretMotor;
    public DcMotorEx flywheelMotor;
    CRServo hoodServo;
    Utils.Debounce leftArrowDebounce;
    Utils.Debounce rightArrowDebounce;
    Utils.Debounce yDebounce;
    Utils.Debounce xDebounce;

    private String rotationStatus = "";

    public Turret(HardwareMap hardwareMap) {
        flywheelMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get(DeviceRegistry.TURRET_MOTOR.str());
        hoodServo = hardwareMap.crservo.get(DeviceRegistry.HOOD_SERVO.str());

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        handleHoodAngling(gamepad2);
        handleTurretRotationAsGamepad(gamepad2);
        handleFlywheelTargetVelocity(gamepad2);
        handleFlywheel(gamepad2);
    }

    private void handleTurretRotationAsGamepad(Gamepad gamepad) {
        double turretPower = gamepad.right_stick_x;
        turretMotor.setPower(turretPower/2);
        if (turretMotor.getCurrentPosition() >=
                Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS) {
            rotationStatus = "hit positive rotation limit";
        }
        else if (turretMotor.getCurrentPosition() <=
                Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS) {
            rotationStatus = "hit negative rotation limit";
        }
        else { turretMotor.setPower(turretPower); }
    }
    private void handleFlywheelTargetVelocity(Gamepad gamepad) {
        if (xDebounce.isPressed(gamepad.x)
            ) {
                flywheelTargetVelocityPercentage += 5;
        }
        else if (yDebounce.isPressed(gamepad.y)
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

    private void handleHoodAngling(Gamepad gamepad) {
        if (gamepad.dpad_up) { raiseHood(); }
        else if (gamepad.dpad_down) { lowerHood(); }
        else { holdHood(); }
    }

    /* Hood Methods */

    private void raiseHood() { hoodServo.setPower(1); }
    private void lowerHood() { hoodServo.setPower(-1); }
    private void holdHood() { hoodServo.setPower(0); }

    public double getHoodPower() {
        return hoodServo.getPower();
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

    public String getFlywheelStatus() {
        if (getFlywheelVelocity(AngleUnit.RADIANS) >= flywheelTargetVelocity) { return "CHARGED"; }
        else {return "UNCHARGED"; }
    }

    public double getFlywheelTargetVelocityPercentage() {
        return flywheelTargetVelocityPercentage;
    }
}
