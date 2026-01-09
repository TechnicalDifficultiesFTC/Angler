package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Red Side | Solo", group="Solo")
public class RedSoloDrive extends OpMode {
    //Declarations
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    Follower follower;


    public void init() {
        /*Construction*/
        intake = new Intake(hardwareMap); //Construct Intake
        indexer = new Indexer(hardwareMap); //Construct Indexer
        turret = new Turret(hardwareMap); //Construct Turret

        //Setup
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(autoEndPose == null ? new Pose() : autoEndPose);
//        follower.update();

        telemetry.setMsTransmissionInterval(10);
        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.update();
    }

    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Subsystem calls

        intake.processInput(gamepad1); //Intake
        indexer.processInput(gamepad2); //Indexer
        turret.processInput(gamepad2); //Turret
        follower.update();

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Turret Pos: ", turret.getTurretPos());
        telemetry.addData("Intake Current: ", intake.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

        telemetry.update();
    }
}
