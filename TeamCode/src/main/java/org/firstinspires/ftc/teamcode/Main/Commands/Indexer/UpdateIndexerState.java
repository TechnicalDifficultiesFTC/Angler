package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Shooter;

public class UpdateIndexerState extends CommandBase {
    Indexer indexer;
    Intake intake;
    Shooter shooter;

    public UpdateIndexerState(Indexer indexer, Intake intake, Shooter shooter) {
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
    }

    public void initialize() {

        /*
        state hierarchy:
        if ball is stalling against the arm -> 1/6 ish power
        if intake is running forward and ball isn't stalling AND shooter is ready -> full power
        if intake is running forward and ball isn't stalling BUT shooter isn't ready -> no power
        if intake isn't running and ball isn't stalling -> 3/4 power
         */

        boolean ballCollidingWithArm = indexer.ballDetected() && indexer.isArmInTheWay();
        boolean intakeMotorRunningForward = intake.intakeMotor.getPower() == 1;
        boolean isReverseIndexerCommandPresent = CommandScheduler.getInstance().isScheduled(
                new ReverseIndexer(indexer)
        );


        if (ballCollidingWithArm) {
            indexer.setIndexerPower(.15); //set to slower speed
        }
        else if (intakeMotorRunningForward && shooter.isFlywheelReady()) {
            indexer.setIndexerPower(1);
        }
        else if (intakeMotorRunningForward && !shooter.isFlywheelReady()) {
            indexer.setIndexerPower(0);
        }
        else{
            indexer.setIndexerPower(.75); //set to default speed to reserve voltage
        }
    }

    public boolean isFinished() {
        return false;
    }
}
