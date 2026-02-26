package org.firstinspires.ftc.teamcode.Main.Helpers;

import java.util.Random;

public class Utils {

    public static double xDist(double a, double b) {
        return Math.abs(a-b);
    }
    public static void halt(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
            * Rounds the input to the amount of decimal places provided as the (int) second parameter
     * @param input Double to be rounded
     * @param decimalPlaces Decimal places to round to (EX: 4.2456 rounded to 2 would be 4.24)
     * @return Formatted string of double rounded
     */
    public static double rad(double input, int decimalPlaces) {
        double scale = Math.pow(10, decimalPlaces);
        return Math.round(input * scale) / scale;
    }

    /**
     * Rounds the input to the amount of decimal places provided as the (int) second parameter
     * @param input Double to be rounded
     * @param decimalPlaces Decimal places to round to (EX: 4.2456 rounded to 2 would be 4.24)
     * @return Formatted string of double rounded
     */
    public static String ras(double input, int decimalPlaces) {
        return String.valueOf(rad(input, decimalPlaces));
    }

    public static String ras(double input) {
        return String.valueOf(rad(input, 2));
    }
    /**
     * Generates a random "MOTM" (Message of the match) to display on the telemetry feed,
     * just for fun, serves no real purpose
     * @return [String]
     */
    public static String generateMOTM() {
        //Just for fun! Display a message of the match on initialization
        Random rand = new Random();
        String[] catchphrases = {
                "Your skills are impressive, Pilot. It is good to have you return.",
                "If brute force isn’t working, you aren’t using enough of it",
                "Welcome back, sharpshooter pilot",
                "The sword is yours",
                "No Technical Difficulties detected!",
                "Am I not merciful?",
                "42-42-564.",
                "Tolerance is a boolean",
                "Pushing P (Program)",
                "Its a terrible day for rain.",
                "Victory is never decided by mobile suit performance alone, nor by the skill of " +
                        "the pilot, alone. The result itself is only the truth!",
                "Both parties, deposit your souls on Libra's scales.",
                "Alea iacta est. The duel is approved.",
                "Fix, Release!",
        };
        return catchphrases[rand.nextInt(catchphrases.length)]+"\n"; //Grabs from a random position in the list
    }

    /**
     * Returns a boolean depending on the amount of depth on a trigger press, used to do an action with the trigger that only requires t/f
     * @param triggerValue Can be found at gamepad.left_trigger() or gamepad.right_trigger()
     * @return [boolean] If trigger has passed threshold
     */
    public static boolean triggerBoolean(double triggerValue) {
        //Compares the float value to threshold
        return (triggerValue > Config.ControllerConstants.TRIGGER_THRESHOLD);
    }


    /**
     * Convert a velocity in radians/sec to percent of configured max velocity.
     */
    public static double velocityRadiansToPercentage(double radians) {
        // Radians to percent
        return (radians / Config.ShooterConstants.MAX_VELOCITY_RADIANS_PER_SEC) * 100.0;
    }

    public static double velocityPercentToRadians(double percent) {
        return Config.ShooterConstants.MAX_VELOCITY_RADIANS_PER_SEC * ((percent)*.01);
    }

    public static int turretDegreesToTicks(double degrees) {
        return (int) (Config.TurretConstants.TICKSPERDEG * degrees);
    }

    public static double turretTicksToDegrees(double ticks) {
        return (ticks/Config.TurretConstants.TICKSPERDEG);
    }

    public static double getScaledHoodTicks(double ticks) {
        return (ticks*100);
    }
}

