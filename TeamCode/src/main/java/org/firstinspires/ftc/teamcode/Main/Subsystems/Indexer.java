package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Indexer {
    NormalizedColorSensor colorSensor;
    public DcMotor indexerMotor;
    public Servo indexerServo;
    private String indexingStatus = "Indexer is not active";
    public Indexer(HardwareMap hardwareMap) {
        //Indexer Motor Setup
        indexerMotor = hardwareMap.dcMotor.get(DeviceRegistry.INDEXER_MOTOR.str());
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Arm Servo
        indexerServo = hardwareMap.servo.get(DeviceRegistry.INDEXER_SERVO.str());
        indexerServo.setDirection(Servo.Direction.FORWARD);

        //Color Sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, DeviceRegistry.COLOR_SENSOR.str());
    }

    public void setup() {
        indexerServo.setPosition(Config.IndexerConstants.servoExpansionTicks);
        //Ensure color sensor light is on
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public void processInput(Gamepad gamepad) {
        //Controller handling x and y and b
        //Indexer main shaft
        if (gamepad.leftBumperWasPressed()) {
            indexerForward(1);
            indexingStatus = "Indexing Forward";
        } else if (gamepad.rightBumperWasPressed()) {
            indexerReverse();
            indexingStatus = "Indexing Backwards";
        } else if (gamepad.bWasPressed()) {
            indexerStop();
            indexingStatus = "Holding Indexer";
        }

        //Arm servo
        if (gamepad.dpadRightWasPressed()) {
            moveServoIn();
        } else if (gamepad.dpadLeftWasPressed()) {
            moveServoOut();
        }
    }

    public void indexerForward(double power) {
        indexerMotor.setPower(power);
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
