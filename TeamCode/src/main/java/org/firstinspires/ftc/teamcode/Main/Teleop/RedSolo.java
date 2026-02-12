package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="Red Solo Static Shooter", group="!Solo")
public class RedSolo extends OpMode {
    //Declarations
    double distance;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    String MOTM = Utils.generateMOTM();
    TelemetryManager panelsTelemetry;
    boolean shootCommandIncomplete = false;
    boolean isBlue = false;
    Follower follower;
    Pose initialFollowerPose = Config.AutoPoses.redAutoEndPose;

    boolean shooterIdle = true;
    @Override
    public void init() {
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, initialFollowerPose, isBlue); //Construct DT

        intake = new Intake(hardwareMap); //Construct Intake
        shooter = new Shooter(hardwareMap); //Construct Turret
        indexer = new Indexer(hardwareMap); //Construct Indexer

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initialFollowerPose);
        follower.update();
        //follower.startTeleOpDrive(true);

        Drawing.init();

        //Telemetry setup
        telemetry.setMsTransmissionInterval(5);

        telemetry.addLine(Config.dasshTag);
        telemetry.addLine(MOTM);
        telemetry.addLine();
        telemetry.addLine("INITIALIZED DRIVETRAIN, INTAKE, INDEXER, TURRET :)");
        telemetry.update();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        shooter.setup();
        indexer.setup();
        mecanumDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
            double curVelocityAsRadians = shooter.getFlywheelVelocityAsRadians();
            double curTargetVelocityAsRadians = shooter.flywheelTargetVelocityAsRadians;
            double error = curTargetVelocityAsRadians - curVelocityAsRadians;

            //Subsystem calls
            indexer.processInput(gamepad1, shooterIdle); //Indexer
            intake.processInput(gamepad1); //Intake
            mecanumDrivetrain.processInputRC(gamepad1);
            mecanumDrivetrain.periodic(); //update follower and pinpoint

            //Controls:
            if (gamepad1.dpadUpWasPressed()) {
                shooter.setFlywheelTargetVelocityAsPercentage(30);
                shooterIdle = true;
            }

            if (gamepad1.dpadDownWasPressed()) {
                shooter.setFlywheelTargetVelocityAsPercentage(0);
                shooterIdle = true;
            }

//            //Make the shooter command a thing here
//            if (gamepad1.yWasPressed() || shootCommandIncomplete) {
//                shootCommandIncomplete = shootCommand(isBlue);
//            }


            //Follower
            if (gamepad1.optionsWasPressed()) {
                follower.update();
            }
            follower.update();


//            follower.setTeleOpDrive(gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x,
//                    false);

            //turret.alignTurret(x,y,heading,false,telemetry);

            //Drawing
            Drawing.drawDebug(follower);
            Drawing.sendPacket();

            /* TELEMETRY!!!!! */
            telemetry.addLine(MOTM);

            //Drivetrain
            telemetry.addLine("\nDT LPM: " + mecanumDrivetrain.isLowPowerMode());
            //Indexer
            telemetry.addLine("Speed rec: " + shooter.getSpeedILUTValue(distance));
            telemetry.addLine("Hood rec: " + shooter.getHoodILUTValue(distance));
            telemetry.addLine();
            telemetry.addLine("Indexer Motor Power: " + indexer.indexerMotor.getPower());
            telemetry.addLine("Ready?: " + shooter.isFlywheelReady());
            telemetry.addLine("Shooter command incomplete?: " + shootCommandIncomplete);
            panelsTelemetry.addLine("Estimated distance: " + Utils.ras(mecanumDrivetrain.getEstimatedDistanceToGoal())
            );

            double x = Utils.rad(follower.getPose().getX(),2);
            double y = Utils.rad(follower.getPose().getY(),2);
            double deg = Utils.rad(Math.toDegrees(follower.getPose().getHeading()),2);
            panelsTelemetry.addLine("EST Pose: (x,y,deg) x: " + x + " y: " + y +  " deg: " + deg);

            //Telemetry update
            panelsTelemetry.update(telemetry);
    }
}
