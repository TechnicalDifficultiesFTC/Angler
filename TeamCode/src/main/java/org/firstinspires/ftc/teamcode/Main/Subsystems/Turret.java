package org.firstinspires.ftc.teamcode.Main.Subsystems;

import android.bluetooth.BluetoothClass;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Turret {
    DcMotor flywheelMotor;
    CRServo hoodServo;
    Utils.Debounce leftArrowDebounce;
    Utils.Debounce rightArrowDebounce;
    Utils.Debounce yDebounce;

    public Turret(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.dcMotor.get(DeviceRegistry.FLYWHEEL_MOTOR.str());
        hoodServo = hardwareMap.crservo.get(DeviceRegistry.HOOD_SERVO.str());

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftArrowDebounce = new Utils.Debounce();
        rightArrowDebounce = new Utils.Debounce();
        yDebounce = new Utils.Debounce();
    }

    public void processInput(Gamepad gamepad) {
        handleHoodAngling(gamepad);
        handleFlywheel(gamepad);
    }
    private void handleFlywheel(Gamepad gamepad) {
        if (leftArrowDebounce.isPressed(gamepad.dpad_left)) {flywheelSpinup();}
        else if (rightArrowDebounce.isPressed(gamepad.dpad_right)) {flywheelSlowdown();}
        else if (yDebounce.isPressed(gamepad.y)) {flywheelESTOP();}
    }
    private void handleHoodAngling(Gamepad gamepad) {
        if (gamepad.dpad_up) { raiseHood(); }
        else if (gamepad.dpad_down) { lowerHood(); }
        else { holdHood(); }
    }

    private void raiseHood() { hoodServo.setPower(1); }
    private void lowerHood() { hoodServo.setPower(-1); }
    private void holdHood() { hoodServo.setPower(0); }
    private void flywheelSpinup() { flywheelMotor.setPower(1); }

    private void flywheelSlowdown() {
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setPower(0);
    }

    private void flywheelESTOP() {
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setPower(0);
    }

    public String getZeroPowerBehaviorAsString() {
        if (flywheelMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            return "BRAKE";
        } else if (flywheelMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT) {
            return "FLOAT";
        } else { return "UNKNOWN"; }
    }

    public double getHoodPower() {
        return hoodServo.getPower();
    }

    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }
}
