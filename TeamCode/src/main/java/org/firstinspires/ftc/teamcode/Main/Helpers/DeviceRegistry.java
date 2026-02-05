package org.firstinspires.ftc.teamcode.Main.Helpers;

public enum DeviceRegistry {

    //Motors
    FRONT_LEFT_MOTOR("FLM"),
    BACK_LEFT_MOTOR("BLM"),
    FRONT_RIGHT_MOTOR("FRM"),
    BACK_RIGHT_MOTOR("BRM"),
    INTAKE_MOTOR("intake"),
    INDEXER_MOTOR("indexer"),
    FLYWHEEL_MOTOR("flywheel"),
    TURRET_MOTOR("turret"),

    //SERVOS
    HOOD_SERVO("hood"),
    INDEXER_SERVO("indexarm"),
    TURRET_SERVO_ONE("ts1"),
    TURRET_SERVO_TWO("ts2"),
    TURRET_SERVO_THREE("ts3"),

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