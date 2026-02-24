package org.firstinspires.ftc.teamcode.Main.Commands.Indexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Indexer;

/**
 * Meant to override Update Indexer State and reverse the indexer to eject balls if
 * nessecary
 */
public class ReverseIndexer extends CommandBase {
    Indexer indexer;

    public ReverseIndexer(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.indexerReverse();
    }

    public boolean isFinished() {
        return true;
    }
}
