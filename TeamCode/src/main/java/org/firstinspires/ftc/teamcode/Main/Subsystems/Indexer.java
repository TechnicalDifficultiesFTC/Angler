package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Indexer {
    public DcMotor indexerMotor;
    private final Utils.Debounce lbDebounce;
    private final Utils.Debounce rbDebounce;
    private final Utils.Debounce bDebounce;
    public String indexingStatus = "Indexer is not active";
    public Indexer(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.dcMotor.get(DeviceRegistry.INDEXER_MOTOR.str());
        indexerMotor.setDirection(DcMotor.Direction.REVERSE);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lbDebounce = new Utils.Debounce();
        rbDebounce = new Utils.Debounce();
        bDebounce = new Utils.Debounce();
    }

    public void processInput(Gamepad gamepad) {
        //Controller handling x and y and b
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
    }
}
