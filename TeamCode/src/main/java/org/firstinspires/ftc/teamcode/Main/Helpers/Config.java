package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Holds static values
 */
public class Config {
    public static class DrivetrainConstants {
        //Before motor power is sent to the motors, they are multiplied by min/max drivetrain speed
        //depending on whether lowPowerMode is in effect or not
        public static double MIN_DT_SPEED = .5; //this value should probably always be <1
        public static double MAX_DT_SPEED = 1; //this value should probably always be 1

        //The D stands for direction
        public static DcMotorSimple.Direction FLMD = DcMotorSimple.Direction.REVERSE;
        public static DcMotorSimple.Direction FRMD = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction BLMD = DcMotorSimple.Direction.FORWARD;
        public static DcMotorSimple.Direction BRMD = DcMotorSimple.Direction.FORWARD;

        //Pedropathing

        //TODO weigh
        public static double ROBOT_MASS_KGS = 32;

        public static Pose2D DEFAULT_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }
    public static class ControllerConstants {
        public static double TRIGGER_THRESHOLD = 0.1;
    }
    @Configurable
    public static class TurretConstants {

        public static double MAX_VELOCITY_RADIANS_PER_SEC = 3.563;
        //Error margins for the "ready!" info message
        public static double FLYWHEEL_ERROR_MARGIN_RADS = .15;
        public static double TURRET_ERROR_MARGIN_TICKS = 75;

        //Turret limits
        public static double TURRET_POSITIVE_LIMIT_TICKS = 1652;
        public static double TURRET_NEGATIVE_LIMIT_TICKS = -953;
        public static double TICKSPERDEG = (double) 790/90;

        //Flywheel Things
        public static double linearInterpolationSlope = 0;
        public static double linearInterpolationIntercept = 0;
        public static class FlywheelPIDF {
            public static double p = 185;
            public static double i = 0;
            public static double d = 0;
            public static double f = 16;
        }

        public static class TurretPIDF {
            public static double p = 21;
        }
    }

    public static class IndexerConstants {
        //These are positions to go to represented by servo ticks (0,1)
        public static double SERVO_INCISION_TICKS = .8;
        public static double SERVO_EXPANSION_TICKS = .625;
        //TODO: Tune
        public static double DISTANCE_SENSOR_BALL_HELD_THRESHOLD_INCHES = 2;
    }

    public static class FieldPositions {
        public static double blueGoalX = 0;
        public static double blueGoalY = 144;
        public static double redGoalX  = 144;
        public static double redGoalY  = 144;
    }
    //Tag funzies :)
    public static String dasshTag = "Dassh01";
}