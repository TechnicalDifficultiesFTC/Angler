package org.firstinspires.ftc.teamcode.Main.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;

public class StopIntakeCommand extends CommandBase {
    Intake intake;
    public StopIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeStop();
    }

    public boolean isFinished() {
        return true;
    }
}
