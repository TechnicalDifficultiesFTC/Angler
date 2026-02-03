package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Holds static values
 */
public class Config {
    public static class AutoPoses {
        public static Pose blueAutoStartPose = new Pose(
                27.611,131.935,Math.toRadians(145));
        public static Pose blueAutoEndPose = new Pose(
                58.84244372990358,130.60771704180067,Math.toRadians(170)
        );

        public static Pose redAutoStartPose = new Pose(116.743,131.472, Math.toRadians(35));
        public static Pose redAutoEndPose = new Pose(85.69774919614152, 130.14469453376208, Math.toRadians(11));
    }
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
        public static double ROBOT_MASS_KGS = 14.424;

        public static Pose2D DEFAULT_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }
    public static class ControllerConstants {
        public static double TRIGGER_THRESHOLD = 0.1;
    }
    @Configurable
    public static class TurretConstants {
        public static double FLYWHEEL_SPEED_HOVERING_PERCENTAGE = 30;

        public static double MAX_VELOCITY_RADIANS_PER_SEC = 3.563;
        //Error margins for the "ready!" info message
        public static double FLYWHEEL_ERROR_MARGIN_RADS = .15;
        public static double TURRET_ERROR_MARGIN_TICKS = 75;

        //Turret limits
        public static int TURRET_POSITIVE_LIMIT_TICKS = 1151; //+180 degs
        public static int TURRET_NEGATIVE_LIMIT_TICKS = -1151; //over -180 degs
        public static double TICKSPERDEG = (double) 575/90;

        public static double MAX_ILUT_DIST = 101.5;
        public static double MIN_ILUT_DIST = 24;
        public static double DISTANCE_OFFSET = -26.463347; //TODO Tune me

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
        public static double SERVO_INCISION_TICKS = .74;
        public static double SERVO_EXPANSION_TICKS = .66; //TODO test recalibration
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