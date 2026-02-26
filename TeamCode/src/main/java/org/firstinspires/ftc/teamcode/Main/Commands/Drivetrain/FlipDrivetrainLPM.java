package org.firstinspires.ftc.teamcode.Main.Commands.Drivetrain;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Main.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

/**
 * Flips low power mode state
 */
public class FlipDrivetrainLPM extends CommandBase {
    MecanumDrivetrain drivetrain;
    public FlipDrivetrainLPM(MecanumDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void execute() {
        drivetrain.setLowPowerMode(!drivetrain.isLowPowerMode());
    }

    public boolean isFinished() {
        return true;
    }
}
