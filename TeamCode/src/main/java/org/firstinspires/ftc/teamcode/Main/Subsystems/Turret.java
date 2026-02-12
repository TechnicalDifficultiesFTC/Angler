package org.firstinspires.ftc.teamcode.Main.Subsystems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.DeviceRegistry;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;

public class Turret extends SubsystemBase {
    static class IsolatedThroughBoreEncoder {
        DcMotorEx encoderMotor;
        public IsolatedThroughBoreEncoder(HardwareMap hardwareMap, String id) {
            //Get motor that encoder is attached to
            encoderMotor = (DcMotorEx) hardwareMap.dcMotor.get(id);
        }
        public int getCurrentPosition() {
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
        double p = Config.TurretConstants.TurretPIDF.p;
        double f = Config.TurretConstants.TurretPIDF.f;
        encoder = new IsolatedThroughBoreEncoder(hardwareMap, DeviceRegistry.TURRET_ENCODER.str());
        pidfController = new PIDFController(new PIDFCoefficients(p,0,0,f));
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

    public void setTurretPositionAsTicks(int targetTicks) {
        pidfController.updatePosition(encoder.getCurrentPosition());
        pidfController.setTargetPosition(targetTicks);
        double power = pidfController.run();
        power = MathFunctions.clamp(power,-1,1);

        servoGroup.set(power);
    }


    public boolean getTurretReady() {
        double pos = encoder.getCurrentPosition();
        double targetPos = pidfController.getTargetPosition();
        return Utils.linearDist(pos, targetPos) <= Config.TurretConstants.TURRET_ERROR_MARGIN_TICKS;
    }

    public double getCurrentPositionAsDegrees() {
        return Utils.turretTicksToDegrees(getCurrentPositionAsTicks());
    }

    public void setCurrentPositionAsDegrees(double degrees) {
        //Add turret localization here
        setTurretPositionAsTicks(Utils.turretDegreesToTicks(degrees));
    }
    public double getCurrentPositionAsTicks() {
        return encoder.getCurrentPosition();
    }
}
