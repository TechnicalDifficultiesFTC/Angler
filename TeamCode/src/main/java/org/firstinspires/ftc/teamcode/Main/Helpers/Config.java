package org.firstinspires.ftc.teamcode.Main.Helpers;

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

    public static class DataCodes {
        public static String lowPowerMode = "lpm";
    }
    //Tag funzies :)
    public static String dasshTag = "Dassh01";
}