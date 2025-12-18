package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Indexer {
    public DcMotor indexerMotor;
    public CRServo kickerTopServo;
    public CRServo kickerBottomServo;
    private final Utils.Debounce lbDebounce;
    private final Utils.Debounce rbDebounce;
    private final Utils.Debounce bDebounce;
    private String indexingStatus = "Indexer is not active";
    public Indexer(HardwareMap hardwareMap) {
        //Indexer Motor Setup
        indexerMotor = hardwareMap.dcMotor.get(DeviceRegistry.INDEXER_MOTOR.str());
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Kicker Servo Setup
        kickerTopServo = hardwareMap.crservo.get(DeviceRegistry.KICKER_SERVO_TOP.str());
        kickerBottomServo = hardwareMap.crservo.get(DeviceRegistry.KICKER_SERVO_BOTTOM.str());

        kickerTopServo.setDirection(DcMotorSimple.Direction.FORWARD);
        kickerBottomServo.setDirection(DcMotorSimple.Direction.FORWARD);

        //Debounce Setup
        lbDebounce = new Utils.Debounce();
        rbDebounce = new Utils.Debounce();
        bDebounce = new Utils.Debounce();
    }

    public void processInput(Gamepad gamepad) {
        //Controller handling x and y and b
        //Indexer main shaft
        if (lbDebounce.isPressed(gamepad.left_bumper)) {
            indexerMotor.setPower(1);
            indexingStatus = "Indexing Forward";
        } else if (rbDebounce.isPressed(gamepad.right_bumper)) {
            indexerMotor.setPower(-1);
            indexingStatus = "Indexing Backwards";
        } else if (bDebounce.isPressed(gamepad.b)) {
            indexerMotor.setPower(0);
            indexingStatus = "Holding Indexer";
        }

        //Kicker servo
        if (Utils.triggerBoolean(gamepad.left_trigger)) {
            setKickerGearboxPower(1);
        } else if (Utils.triggerBoolean(gamepad.right_trigger)) {
            setKickerGearboxPower(-1);
        } else { setKickerGearboxPower(0); }
    }

    public void setKickerGearboxPower(double power) {
        kickerTopServo.setPower(power);
        kickerBottomServo.setPower(-power);
    }

    public String getIndexingStatus() {
        return indexingStatus;
    }
}
