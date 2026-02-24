package org.firstinspires.ftc.teamcode.Main.Helpers;

public enum DeviceRegistry {

    //Motors
    FRONT_LEFT_MOTOR("FLM"),
    BACK_LEFT_MOTOR("BLM"),
    FRONT_RIGHT_MOTOR("FRM"),
    BACK_RIGHT_MOTOR("BRM"),
    INTAKE_MOTOR("intake"),
    INDEXER_MOTOR("indexer"),
    FLYWHEEL_MOTOR_LEFT("flywheelLeft"),
    FLYWHEEL_MOTOR_RIGHT("flywheelRight"),
    //Webcam
    WEBCAM("webcam"),

    //EXTERNAL ENCODERS
    TURRET_ENCODER(FRONT_LEFT_MOTOR.str()), //inside is the motor where the encoder is attached

    //SERVOS
    HOOD_SERVO("hood"),
    INDEXER_SERVO("indexarm"),
    TURRET_SERVO_FRONT("ts1"),
    TURRET_SERVO_CENTER("ts2"),
    TURRET_SERVO_REAR("ts3"),

    //IMU
    IMU("imu"),
    PINPOINT("pinpoint"),
    DISTANCE_SENSOR("distance");

    private final String deviceName;

    DeviceRegistry(String deviceName) {
        this.deviceName = deviceName;
    }

    public String str() {
        return deviceName;
    }
}