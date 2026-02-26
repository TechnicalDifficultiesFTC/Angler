package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

public class UpdateIndexerState extends CommandBase {
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    TelemetryManager telemetry;

    public UpdateIndexerState(Indexer indexer, Intake intake, Shooter shooter, TelemetryManager telemetry) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.telemetry = telemetry;
        addRequirements(indexer);
    }

    public UpdateIndexerState(Indexer indexer, Intake intake, Shooter shooter) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(indexer);
    }

    public void initialize() {
    }

    public void execute() {

        /*
        state hierarchy:
        if ball is stalling against the arm -> 1/6 ish power
        if intake is running forward and ball isn't stalling AND shooter is ready -> full power
        if intake is running forward and ball isn't stalling BUT shooter isn't ready -> no power
        if intake isn't running and ball isn't stalling -> 3/4 power
         */

        boolean ballCollidingWithArm = indexer.ballDetected() && indexer.isArmInTheWay();
        boolean intakeMotorRunningForward = intake.intakeMotor.getPower() == 1;
        boolean telemetryDefined = (telemetry == null);

        if (ballCollidingWithArm) {
            indexer.setIndexerPower(.15); //set to slower speed

            if (!telemetryDefined) {
                telemetry.addData("istate = ", "ball colliding");
            }
        }
        else if (shooter.isFlywheelReady()) {
            indexer.setIndexerPower(1);

            if (!telemetryDefined) {
                telemetry.addData("istate = ", "ball push through");
            }
        }
        else if (!shooter.isFlywheelReady()) {
            indexer.setIndexerPower(0);

            if (!telemetryDefined) {
                telemetry.addData("istate = ", "ball stall");
            }
        }
        else{
            indexer.setIndexerPower(1);

            if (!telemetryDefined) {
                telemetry.addData("istate = ", "default state");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
