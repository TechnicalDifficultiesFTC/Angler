package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Main.Helpers.Config;
import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;

import java.util.concurrent.TimeUnit;

/**
 * Practically the only difference from this to moveArmOut is the method change
 * from indexer.moveArmOut to indexer.moveArmIn
 */
public class MoveIndexerArmInCommand extends CommandBase {
    Indexer indexer;
    Timing.Timer timer = new Timing.Timer(Config.IndexerConstants.TRANSITION_TIME_MILLISECONDS,
            TimeUnit.MILLISECONDS);
    public MoveIndexerArmInCommand(Indexer indexer) {
        this.indexer = indexer;
    }

    public void initialize() {
        indexer.moveArmIn();
    }

    public boolean isFinished () {
        return timer.done();
    }

}
