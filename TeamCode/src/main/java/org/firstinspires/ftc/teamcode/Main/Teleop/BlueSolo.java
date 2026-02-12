package org.firstinspires.ftc.teamcode.Main.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Helpers.Drawing;
import org.firstinspires.ftc.teamcode.Main.Helpers.Utils;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Turret;

@Configurable
@TeleOp(name="Blue Solo Static Shooter", group="!Solo")
public class BlueSolo extends OpMode {
    public Pose autoEndPose;
    double distance;
    MecanumDrivetrain mecanumDrivetrain;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    Turret turret;
    String MOTM = Utils.generateMOTM();
    TelemetryManager panelsTelemetry;
    boolean shootCommandIncomplete = false;
    boolean isBlue = true;
    Pose initialFollowerPose = Config.AutoPoses.blueAutoEndPose;
    boolean shooterIdle = true;
    @Override
    public void init() {
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap, new Pose(initialFollowerPose.getX(),initialFollowerPose.getY(),0),Config.GlobalConstats.defaultIsBlueValue); //Construct DT

        intake = new Intake(hardwareMap); //Construct Intake
        shooter = new Shooter(hardwareMap); //Construct Turret
        indexer = new Indexer(hardwareMap); //Construct Indexer
        turret = new Turret(hardwareMap); //Construct turret

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
        turret.setTurretPositionAsTicks(0);
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

        //Controls:
        if (gamepad1.dpadUpWasPressed()) {
            shooter.setFlywheelTargetVelocityAsPercentage(30);
            shooterIdle = true;
        }

        if (gamepad1.dpadDownWasPressed()) {
            shooter.setFlywheelTargetVelocityAsPercentage(0);
            shooterIdle = true;
        }

        if (gamepad1.yWasPressed() || shootCommandIncomplete) {
            //shootCommandIncomplete = shootCommand(isBlue);
        }




//            follower.setTeleOpDrive(gamepad1.left_stick_x,
//                    -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x,
//                    false);

        //turret.alignTurret(x,y,heading,false,telemetry);

        /* TELEMETRY!!!!! */
        telemetry.addLine(MOTM);

        //Drivetrain
        telemetry.addLine("\nDT LPM: " + mecanumDrivetrain.isLowPowerMode());
        telemetry.addLine("Heading: " + mecanumDrivetrain.getFollower().getPose().getHeading());
        //Indexer
        telemetry.addLine("Speed rec: " + shooter.getSpeedILUTValue(distance));
        telemetry.addLine("Hood rec: " + shooter.getHoodILUTValue(distance));
        telemetry.addLine("Indexer Motor Power: " + indexer.indexerMotor.getPower());
        telemetry.addLine("Flywheel Ready?: " + shooter.isFlywheelReady());
        telemetry.addLine("Shooter command incomplete?: " + shootCommandIncomplete);
        panelsTelemetry.addLine("Estimated distance: " + Utils.ras(mecanumDrivetrain.getEstimatedDistanceToGoal()));

        //Telemetry update
        panelsTelemetry.update(telemetry);
    }
}
