package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Holds static values
 */
public class Config {
    public static class Drivetrain {
        //Before motor power is sent to the motors, they are multiplied by min/max drivetrain speed
        //depending on whether lowPowerMode is in effect or not
        public static double MIN_DT_SPEED = .5; //this value should probably always be <1
        public static double MAX_DT_SPEED = 1; //this value should probably always be 1
    }
    public static class ControllerConstants {
        public static double TRIGGER_THRESHOLD = 0.1;
    }
    @Configurable
    public static class TurretConstants {
        public static double MAX_VELOCITY_RADIANS_PER_SEC = 3.563;
        public static double TURRET_POSITIVE_LIMIT_TICKS = 1652;
        public static double TURRET_NEGATIVE_LIMIT_TICKS = -953;
    }

    /**
     * Logging codes used in telemetry data logging
     */
    public static class DataCodes {
        public static String lowPowerMode = "lpm";
    }
    //Tag funzies :)
    public static String dasshTag = "Dassh01";
}