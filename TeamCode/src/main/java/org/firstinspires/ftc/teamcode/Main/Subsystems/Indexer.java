package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;

public class Indexer {
    public DcMotor indexerMotor;
    public Servo indexerServo;
    private String indexingStatus = "Indexer is not active";
    private DistanceSensor distanceSensor;
    private boolean armInTheWay = false;
    private boolean ballBlocked = false;
    public boolean didAutoRecover = false;
    public boolean indexerReversing = false;
    Turret turret;
    public Indexer(HardwareMap hardwareMap, Turret turret) {
        //Indexer Motor Setup
        this.turret = turret;

        indexerMotor = hardwareMap.dcMotor.get(DeviceRegistry.INDEXER_MOTOR.str());
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Arm Servo
        indexerServo = hardwareMap.servo.get(DeviceRegistry.INDEXER_SERVO.str());
        indexerServo.setDirection(Servo.Direction.FORWARD);

        //Distance Sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, DeviceRegistry.DISTANCE_SENSOR.str());
    }

    public void setup() {
        moveArmOut();
    }

    public void processInput(Gamepad gamepad, boolean shooterIdle) {
        //Controller handling x and y and b
        //Indexer main shaft
        if (gamepad.leftBumperWasPressed()) {
            indexerForward(1);
            indexerReversing = false;
            indexingStatus = "Indexing Forward (quickly)";
        }

        if (gamepad.rightBumperWasPressed()) {
            indexerReverse();
            indexerReversing = true;
            indexingStatus = "Indexing Backwards";
        }

        if (gamepad.bWasPressed()) {
            indexerStop();
            indexingStatus = "Holding Indexer";
        }

        //Check to override automatic ball slowdown measures
        boolean ballIsFalselyBlocked = (ballBlocked && !ballHeld());
        if (ballIsFalselyBlocked && !indexerReversing) {
            ballBlocked = false;
            indexerForward(1);
        }

        else if (!turret.getFlywheelReady() && !shooterIdle) {
            indexerForward(0);
            indexingStatus = "Stopped because flywheel wasn't ready";
        }

        //Check to see if indexer power needs to be slowed down to prevent a voltage stall
        boolean ballIsInTheWay = ballHeld() && isArmInTheWay();
        if (ballIsInTheWay && !indexerReversing) { //Can only be run if no indexer reverse cmd
            indexerForward(0.15);
            ballBlocked = true;
            indexingStatus = "Indexing Forward (slowly)";
        }

        //Arm servo
        if (gamepad.dpadRightWasPressed()) {
            moveArmIn();
        } else if (gamepad.dpadLeftWasPressed()) {
            moveArmOut();
        }
    }

    /**
     * @return If a ball is held in the indexer
     */
    public boolean ballHeld() {
        return distanceSensor.getDistance(DistanceUnit.INCH) <
                Config.IndexerConstants.DISTANCE_SENSOR_BALL_HELD_THRESHOLD_INCHES;
    }
    public double getDistanceReported() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
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

    /**
     * Moves the servo to block balls
     */
    public void moveArmIn() {
        armInTheWay = true;
        indexerServo.setPosition(Config.IndexerConstants.SERVO_INCISION_TICKS);
    }

    /**
     * Moves the servo out of the way of balls
     */
    public void moveArmOut() {
        armInTheWay = false;
        indexerServo.setPosition(Config.IndexerConstants.SERVO_EXPANSION_TICKS);
    }

    public boolean isArmInTheWay() {
        return armInTheWay;
    }

    public boolean isBallBlocked() {
        return ballBlocked;
    }

    class ballTracker {
        ballTracker() {
            Timing.Timer timer = new Timing.Timer(500);
        }
    }
}
