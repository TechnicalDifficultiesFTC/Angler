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
    PIDFController pidfController;


    public Turret(HardwareMap hardwareMap) {
        double p = Config.TurretConstants.TurretPIDFLarge.p;
        double i = Config.TurretConstants.TurretPIDFLarge.i;
        double d = Config.TurretConstants.TurretPIDFLarge.d;
        double f = Config.TurretConstants.TurretPIDFLarge.f;
        encoder = new IsolatedThroughBoreEncoder(hardwareMap, DeviceRegistry.TURRET_ENCODER.str());
        pidfController = new PIDFController(new PIDFCoefficients(p,i,d,f));

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
        double targetPos = pidfController.getTargetPosition();
        return Utils.xDist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
    }

    public double getCurrentPositionAsDegrees() {
        return Utils.turretTicksToDegrees(getCurrentPositionAsTicks());
    }

    public void tuningSetTurretPositionAsDegrees(double targetDegrees) {
        //Add degrees wrapping here
        targetDegrees = normalizeDegrees(targetDegrees);
        double power = runPIDFController(targetDegrees);
        servoGroup.set(power);
    }
    public void tuningSetTurretPositionAsDegrees(double targetDegrees, Telemetry telemetry) {
        //Add degrees wrapping here
        targetDegrees = normalizeDegrees(targetDegrees);

        telemetry.addLine("Setpoint: " + targetDegrees);
        double power = runPIDFController(targetDegrees);
        servoGroup.set(power);
    }

    /**
     * For some reason, to make the function work in a real application thats not PIDF tuning,
     * I have to reverse the power output?? freaky stuff...
     */
    public void realSetTurretPositionAsDegrees(double targetDegrees) {
        //Add degrees wrapping here
        targetDegrees = normalizeDegrees(targetDegrees);
        double power = runPIDFController(targetDegrees);
        servoGroup.set(-power);
    }

    public double runPIDFController(double targetDegrees) {
        double absError = Utils.xDist(targetDegrees,getCurrentPositionAsDegrees());

        if (absError < 2) {
            return 0;
        }

        if (absError < 20) {

        }

        double power = longDistancePIDF(targetDegrees);

        // Use tanh to smoothly scale any value to (-1, 1)
        //smaller = more aggressive power
        //bigger = less aggressive
        power = Math.tanh(power / 125);
        return power;
    }

    public double longDistancePIDF(double targetDegrees) {
        double p = Config.TurretConstants.TurretPIDFLarge.p;
        double i = Config.TurretConstants.TurretPIDFLarge.i;
        double d = Config.TurretConstants.TurretPIDFLarge.d;
        double f = Config.TurretConstants.TurretPIDFLarge.f;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,i,d,f);

        pidfController.updatePosition(getCurrentPositionAsDegrees());
        pidfController.setTargetPosition(targetDegrees);


        pidfController.setCoefficients(pidfCoefficients);
        double power = pidfController.run();

//        //Change ff input based on error direction
        double error = targetDegrees - getCurrentPositionAsDegrees();
        pidfController.updateFeedForwardInput(Math.signum(error));
        return pidfController.run();
    }
    public void setPIDFCoeffiecients(PIDFCoefficients pidfCoefficients) {
        pidfController.setCoefficients(pidfCoefficients);
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
    public PIDFCoefficients getPIDFCoefficients() {
        return pidfController.getCoefficients();
    }

    public PIDFController getPIDFController() {
        return pidfController;
    }
}
