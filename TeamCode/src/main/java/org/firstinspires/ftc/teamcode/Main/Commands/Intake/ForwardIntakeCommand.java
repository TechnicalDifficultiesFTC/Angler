package org.firstinspires.ftc.teamcode.Main.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;

/**
 * Toggleable command
 * Yields if reverseIntake is scheduled
 */
public class ForwardIntakeCommand extends CommandBase {
    Intake intake;
    public ForwardIntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.intakeSpinup();
    }

    public boolean isFinished() {
        //yield to intake reversal
        if (CommandScheduler.getInstance().isScheduled(new ReverseIntakeCommand(intake))) {
            return true;
        } else { return false; }
    }
}
