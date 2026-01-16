package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Indexer {
    public DcMotor indexerMotor;
    public Servo indexerServo;
    private String indexingStatus = "Indexer is not active";
    public Indexer(HardwareMap hardwareMap) {
        //Indexer Motor Setup
        indexerMotor = hardwareMap.dcMotor.get(DeviceRegistry.INDEXER_MOTOR.str());
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Arm servo
        indexerServo = hardwareMap.servo.get(DeviceRegistry.INDEXER_SERVO.str());
        indexerServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setup() {
        indexerServo.setPosition(Config.IndexerConstants.servoExpansionTicks);
    }

    public void processInput(Gamepad gamepad) {
        //Controller handling x and y and b
        //Indexer main shaft
        if (gamepad.leftBumperWasPressed()) {
            indexerMotor.setPower(1);
            indexingStatus = "Indexing Forward";
        } else if (gamepad.rightBumperWasPressed()) {
            indexerMotor.setPower(-1);
            indexingStatus = "Indexing Backwards";
        } else if (gamepad.bWasPressed()) {
            indexerMotor.setPower(0);
            indexingStatus = "Holding Indexer";
        }

        //Arm servo
        if (gamepad.dpadRightWasPressed()) {
            indexerServo.setPosition(Config.IndexerConstants.servoIncisionTicks);
        } else if (gamepad.dpadLeftWasPressed()) {
            indexerServo.setPosition(Config.IndexerConstants.servoExpansionTicks);
        }
    }

    public void indexerForward() {
        indexerMotor.setPower(1);
    }
    public void indexerReverse() {
        indexerMotor.setPower(-1);
    }
    public void indexerStop() {
        indexerMotor.setPower (0);
    }
    public String getIndexingStatus() {
        return indexingStatus;
    }

    public void moveServoIn() {
        indexerServo.setPosition(Config.IndexerConstants.servoIncisionTicks);
    }

    public void moveServoOut() {
        indexerServo.setPosition(Config.IndexerConstants.servoExpansionTicks);
    }
}
