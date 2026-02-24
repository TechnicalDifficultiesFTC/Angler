package org.firstinspires.ftc.teamcode.Main.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;

/**
 * Hops off the queue instantly after kicking off intake
 */
public class ReverseIntakeCommand extends CommandBase {
    Intake intake;
    public ReverseIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeReverse();
    }

    public boolean isFinished() {
        return true;
    }
}
