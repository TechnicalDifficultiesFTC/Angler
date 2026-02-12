package org.firstinspires.ftc.teamcode.Main.Helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class VelocityMotorGroup {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public VelocityMotorGroup(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    public void setVelocityPIDF(PIDFCoefficients pidfCoefficients) {
        leftMotor.setVelocityPIDFCoefficients(
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f
        );
        rightMotor.setVelocityPIDFCoefficients(
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f
        );
    }

    public void setVelocityTarget(double velocity, AngleUnit angleUnit) {
        leftMotor.setVelocity(velocity, angleUnit);
        rightMotor.setVelocity(velocity, angleUnit);
    }

    public void setVelocityTarget(double velocity) {
        leftMotor.setVelocity(velocity, AngleUnit.RADIANS);
        rightMotor.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public double getCurrentDualVelocityRadians() {
        double sumOfVelocities = leftMotor.getVelocity(AngleUnit.RADIANS) +
                rightMotor.getVelocity(AngleUnit.RADIANS);
        return sumOfVelocities/2;
    }
}
