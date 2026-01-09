package org.firstinspires.ftc.teamcode.Main.Teleop.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@TeleOp(name="Testing General", group="Tuning")
public class Testing extends LinearOpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    double amps;
    double maxVelPercent = 0;

    @Override
    public void runOpMode() {
        //Construct DT
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        turret = new Turret(hardwareMap);

        //Setup
        telemetry.setMsTransmissionInterval(10);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //Subsystem calls
            mecanumDrivetrain.processInputRC(gamepad1); //DT
            intake.processInput(gamepad1);
            indexer.processInput(gamepad1);
            turret.processInput(gamepad2);



           //Telemetry
            telemetry.addLine(MOTM);
            //Drivetrain
//            telemetry.addData(Config.DataCodes.lowPowerMode, mecanumDrivetrain.isLowPowerMode());
//            telemetry.addLine("Run Mode = " + mecanumDrivetrain.runmode + " Behavior: " +
//                    mecanumDrivetrain.currentZeroPowerBehavior);
//            telemetry.addLine("Bot Heading: " + Utils.roundAsDouble(
//                    Math.toDegrees(mecanumDrivetrain.botHeading), 3));

            //Intake
            telemetry.addLine("Intake Power: " + intake.intakeMotor.getPower());

            //Turret
            amps = turret.getFlywheelAmps();

            Pose2D pose2D = mecanumDrivetrain.getPinpoint().getPosition();

            if (turret.getFlywheelVelocityPercentage() > maxVelPercent) {
                maxVelPercent = turret.getFlywheelVelocityPercentage();
            }

            //Flywheel feedback
            telemetry.addLine("\nFlywheel %: " + turret.getFlywheelVelocityPercentage());
            telemetry.addLine("Max Velocity % Reached: " + maxVelPercent);
            telemetry.addLine("Flywheel Amps: " + amps);

            //Turret feedback
            telemetry.addLine(turret.getRotationStatus());
            telemetry.addLine("Turret position (TICKS): " +
                    turret.turretMotor.getCurrentPosition());

            //Pinpoint feedback
            //X
            telemetry.addLine("\nX coordinate (IN)" +
                    Utils.ras(pose2D.getX(DistanceUnit.INCH),2));
            //Y
            telemetry.addLine("Y coordinate (IN)" +
                    Utils.ras(pose2D.getY(DistanceUnit.INCH),2));
            //Heading
            telemetry.addLine("Heading Angle (Degrees) " +
                    Utils.ras(pose2D.getHeading(AngleUnit.DEGREES),2));

            telemetry.update();
        }
    }
}
