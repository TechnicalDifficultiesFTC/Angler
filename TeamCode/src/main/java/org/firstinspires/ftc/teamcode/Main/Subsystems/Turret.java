package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Turret extends SubsystemBase {
    static class IsolatedThroughBoreEncoder {
        DcMotorEx encoderMotor;
        Motor.Direction encoderDirection = Motor.Direction.REVERSE;
        public IsolatedThroughBoreEncoder(HardwareMap hardwareMap, String id) {
            //Get motor that encoder is attached to
            encoderMotor = (DcMotorEx) hardwareMap.dcMotor.get(id);
        }
        public int getCurrentPosition() {

            if (encoderDirection == Motor.Direction.REVERSE) {
                return -encoderMotor.getCurrentPosition();
            }
            return encoderMotor.getCurrentPosition();
        }
    }

    CRServoEx frontServo;
    CRServoEx centerServo;
    CRServoEx rearServo;
    CRServoGroup servoGroup;
    IsolatedThroughBoreEncoder encoder;
    PIDFController longDistPIDF;
    PIDFController shortDistPIDF;
    public boolean usingShortRange = false;

    public Turret(HardwareMap hardwareMap) {
        double pL = Config.TurretConstants.TurretPIDFLarge.p;
        double iL = Config.TurretConstants.TurretPIDFLarge.i;
        double dL = Config.TurretConstants.TurretPIDFLarge.d;
        double fL = Config.TurretConstants.TurretPIDFLarge.f;

        double pS = Config.TurretConstants.TurretPIDFSmall.p;
        double iS = Config.TurretConstants.TurretPIDFSmall.i;
        double dS = Config.TurretConstants.TurretPIDFSmall.d;
        double fS = Config.TurretConstants.TurretPIDFSmall.f;

        encoder = new IsolatedThroughBoreEncoder(hardwareMap, DeviceRegistry.TURRET_ENCODER.str());
        longDistPIDF = new PIDFController(new PIDFCoefficients(pL,iL,dL,fL));
        shortDistPIDF = new PIDFController(new PIDFCoefficients(pS,iS,dS,fS));

        //init
        frontServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_FRONT.str()); //leader
        centerServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_CENTER.str());
        rearServo = new CRServoEx(hardwareMap, DeviceRegistry.TURRET_SERVO_REAR.str());

        //directions
        frontServo.setInverted(Config.TurretConstants.TurretServoDirections.frontServoInverted);
        centerServo.setInverted(Config.TurretConstants.TurretServoDirections.centerServoInverted);
        rearServo.setInverted(Config.TurretConstants.TurretServoDirections.rearServoInverted);

        //servo group
        servoGroup = new CRServoGroup(frontServo, centerServo, rearServo);
    }

    public boolean isTurretReady() {
        double pos = encoder.getCurrentPosition();
        double targetPos = longDistPIDF.getTargetPosition();
        return Utils.xDist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
    }

    public double getCurrentPositionAsDegrees() {
        return Utils.turretTicksToDegrees(getCurrentPositionAsTicks());
    }

    public void tuningSetTurretPositionAsDegrees(double targetDegrees) {
        //Add degrees wrapping here
        targetDegrees = normalizeDegrees(targetDegrees);
        double power = runPIDFControllers(targetDegrees);
        servoGroup.set(power);
    }

    /**
     * For some reason, to make the function work in a real application thats not PIDF tuning,
     * I have to reverse the power output?? freaky stuff...
     */
    public void realSetTurretPositionAsDegrees(double targetDegrees) {
        //Add degrees wrapping here
        targetDegrees = normalizeDegrees(targetDegrees);
        double power = runPIDFControllers(targetDegrees);
        servoGroup.set(-power);
    }

    public double runPIDFControllers(double targetDegrees) {
        final double SWITCH_THRESHOLD_HIGH = 25; // Switch to long range
        final double SWITCH_THRESHOLD_LOW = 12;  // Switch back to short range

        double absError = Utils.xDist(targetDegrees,getCurrentPositionAsDegrees());

//        if (absError < 2) { //Tolerance of +- 2 degrees
//            return 0;
//        }

        // Hysteresis to prevent rapid switching
        if (absError > SWITCH_THRESHOLD_HIGH) {
            if (usingShortRange) {
                longDistPIDF.reset(); // Reset before switching to long range
            }
            usingShortRange = false;
        }
        else if (absError < SWITCH_THRESHOLD_LOW) {
            if (!usingShortRange) {
                shortDistPIDF.reset(); // Reset before switching to short range
            }
            usingShortRange = true;
        }
        // Between LOW and HIGH: maintain current mode (hysteresis)

        double power = usingShortRange
                ? shortRangePIDF(targetDegrees)
                : longRangePIDF(targetDegrees);

        //tanh to smoothly scale PIDF values to (-1, 1)
        //smaller = more aggressive power
        //bigger = less aggressive power
        power = Math.tanh(power / 125);
        return power;
    }


    public double shortRangePIDF(double targetDegrees) {
        double p = Config.TurretConstants.TurretPIDFSmall.p;
        double i = Config.TurretConstants.TurretPIDFSmall.i;
        double d = Config.TurretConstants.TurretPIDFSmall.d;
        double f = Config.TurretConstants.TurretPIDFSmall.f;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        shortDistPIDF.updatePosition(getCurrentPositionAsDegrees());
        shortDistPIDF.setTargetPosition(targetDegrees);

        //TODO disable for tuning
        shortDistPIDF.setCoefficients(pidfCoefficients);

//        //Change ff input based on error direction
        double error = targetDegrees - getCurrentPositionAsDegrees();
        shortDistPIDF.updateFeedForwardInput(Math.signum(error));

        return shortDistPIDF.run();
    }

    public double longRangePIDF(double targetDegrees) {
        double p = Config.TurretConstants.TurretPIDFLarge.p;
        double i = Config.TurretConstants.TurretPIDFLarge.i;
        double d = Config.TurretConstants.TurretPIDFLarge.d;
        double f = Config.TurretConstants.TurretPIDFLarge.f;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        longDistPIDF.updatePosition(getCurrentPositionAsDegrees());
        longDistPIDF.setTargetPosition(targetDegrees);

        //TODO disabled for tuning
        longDistPIDF.setCoefficients(pidfCoefficients);

//        //Change ff input based on error direction
        double error = targetDegrees - getCurrentPositionAsDegrees();
        longDistPIDF.updateFeedForwardInput(Math.signum(error));

        return longDistPIDF.run();
    }
    public void setLongPIDFCoeffecients(PIDFCoefficients pidfCoefficients) {
        longDistPIDF.setCoefficients(pidfCoefficients);
    }

    public void setShortPIDFCoeffecients(PIDFCoefficients pidfCoefficients) {
        shortDistPIDF.setCoefficients(pidfCoefficients);
    }
    public double normalizeDegrees(double degreesTarget) {
        double posLimitAsDegs = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_POSITIVE_LIMIT_TICKS);
        double negLimitAsDegs = Utils.turretTicksToDegrees(Config.TurretConstants.TURRET_NEGATIVE_LIMIT_TICKS);

        if (degreesTarget > posLimitAsDegs) {
            degreesTarget -= 360;
        }
        if (degreesTarget < negLimitAsDegs) {
            degreesTarget += 360;
        }

        degreesTarget = MathFunctions.clamp(degreesTarget,negLimitAsDegs,posLimitAsDegs);

        return degreesTarget;
    }
    public double getCurrentPositionAsTicks() {
        return encoder.getCurrentPosition();
    }
    public PIDFCoefficients getLongPIDFCoefficients() {
        return longDistPIDF.getCoefficients();
    }

    public PIDFCoefficients getShortPIDFCoefficients() {
        return shortDistPIDF.getCoefficients();
    }

    public PIDFController getLongPIDFController() {
        return longDistPIDF;
    }
}
