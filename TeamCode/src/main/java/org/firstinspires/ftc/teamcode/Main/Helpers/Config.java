package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Holds static values
 */
public class Config {
    public static class Drivetrain {
        //Before motor power is sent to the motors, they are multiplied by min/max drivetrain speed
        //depending on whether lowPowerMode is in effect or not
        public static double MIN_DT_SPEED = .5; //this value should probably always be <1
        public static double MAX_DT_SPEED = 1; //this value should probably always be 1

        //The D stands for direction
        public static DcMotorSimple.Direction FLMD = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction FRMD = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction BLMD = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction BRMD = DcMotorSimple.Direction.FORWARD;
    }
    public static class ControllerConstants {
        public static double TRIGGER_THRESHOLD = 0.1;
    }
    @Configurable
    public static class TurretConstants {
        public static double MAX_VELOCITY_RADIANS_PER_SEC = 3.563;
        public static double FLYWHEEL_ERROR_MARGIN_RADS = .15;
        public static double TURRET_ERROR_MARGIN_TICKS = 75;
        public static double TURRET_POSITIVE_LIMIT_TICKS = 1652;
        public static double TURRET_NEGATIVE_LIMIT_TICKS = -953;
        public static double POWER = 17;
        public static double TICKSPERDEG = (double) -790 /90;
        public static class FlywheelPIDF {
            public static double p;
            public static double i;
            public static double d;
            public static double f;
        }
    }

    public static class IndexerConstants {
        //These are positions to go to represented by servo ticks (0,1)
        public static double servoIncisionTicks = .8;
        public static double servoExpansionTicks = .625;
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