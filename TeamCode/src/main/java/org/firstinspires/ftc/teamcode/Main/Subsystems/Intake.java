package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;


public class Intake {
    public DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get(DeviceRegistry.INTAKE_MOTOR.str());
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Intake on leftbumper
     * Outtake on rightbumper
     * Default -> motor still
     */
    public void processInput(Gamepad gamepad) {
        if (Utils.triggerBoolean(gamepad.left_trigger)) { intakeSpinup(); }
        else if (Utils.triggerBoolean(gamepad.right_trigger)) { intakeReverse(); }
        else { intakeStop(); }
    }

    private void intakeSpinup() { intakeMotor.setPower(1); }
    private void intakeReverse() { intakeMotor.setPower(-1); }
    private void intakeStop() { intakeMotor.setPower(0); }
}
