package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="GP Static Shooter v1 | Solo", group="Solo")
public class General extends OpMode {
    //Declarations
    double percent = 50;
    double distance;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    TelemetryManager panelsTelemetry;
    boolean shootCommandIncomplete = false;

    Follower follower;


    @Override
    public void init() {
        /*
        CONSTRUCTION!!!!!!!!!!!!!!!!!!!!!!!!!!
         */

        //TODO test shooter NEW command (programmed)
        //TODO test follower's accuracy to goal (tune)
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap); //Construct DT
        intake = new Intake(hardwareMap); //Construct Intake
        indexer = new Indexer(hardwareMap); //Construct Indexer
        turret = new Turret(hardwareMap); //Construct Turret

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(134.714,9.158,Math.toRadians(90)));
        follower.update();
        follower.startTeleOpDrive(true);

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
    public void loop() {
            distance = turret.getEstimatedDistanceToGoal(follower.getPose(),false);
            double curVelocityAsRadians = turret.getFlywheelVelocityAsRadians();
            double curTargetVelocityAsRadians = turret.flywheelTargetVelocityAsRadians;
            double error = curTargetVelocityAsRadians - curVelocityAsRadians;

            //Subsystem calls
            indexer.processInput(gamepad1); //Indexer
            intake.processInput(gamepad1); //Intake

            //Controls:


            //Shooter testing
            if (gamepad1.dpadUpWasPressed()) {
                percent += 5;
            }

            if (gamepad1.dpadDownWasPressed()) {
                percent -= 5;
            }

            if (gamepad1.yWasPressed() || shootCommandIncomplete) {
                double targetPercentage = turret.getSpeedILUTValue(distance);
                double targetHoodAngle = turret.getHoodILUTValue(distance);
                turret.setFlywheelTargetVelocityAsPercentage(targetPercentage);
                turret.setHoodAngle(targetHoodAngle);

                //Shoot if ready
                if (turret.readyToFire()) {
                    indexer.moveArmOut();
                    indexer.indexerForward(1);
                    shootCommandIncomplete = false;
                }

                else {
                    //TODO test automatic ball restriction
                    indexer.moveArmIn(); //Indexer automatic restraints should kick in here!
                    shootCommandIncomplete = true; //This will cause a retry if the shooter wasn't ready initially
                }
            }

            //Follower
            follower.update();
            follower.setTeleOpDrive(gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    false);

            if (gamepad1.optionsWasPressed()) {
                follower.setPose(new Pose(0,0,0));
            }



            //Drawing
            Drawing.drawDebug(follower);
            Drawing.sendPacket();

            /* TELEMETRY!!!!! */
            telemetry.addLine(MOTM);

            //Drivetrain
            telemetry.addLine("\nDT LPM: " + mecanumDrivetrain.isLowPowerMode());

            //Indexer
            telemetry.addLine("Indexer Status: " + indexer.getIndexingStatus());
            telemetry.addLine("Ball Seen?: " + indexer.ballHeld());
            panelsTelemetry.addData("Error ", Utils.ras(Math.abs(error)));
            panelsTelemetry.addData("Setpoint ", Utils.ras(curTargetVelocityAsRadians));
            panelsTelemetry.addData("Velocity ", Utils.ras(curVelocityAsRadians));
            panelsTelemetry.addData("Speed rec: ", turret.getSpeedILUTValue(distance));
            panelsTelemetry.addData("Hood rec: ", turret.getHoodILUTValue(distance));
            panelsTelemetry.addData("Estimated distance: ", turret.getEstimatedDistanceToGoal(follower.getPose(),false));

            //Telemetry update
            telemetry.update();
            panelsTelemetry.update(telemetry);
    }
}
