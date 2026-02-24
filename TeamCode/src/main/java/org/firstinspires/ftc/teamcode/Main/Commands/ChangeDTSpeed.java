package org.firstinspires.ftc.teamcode.Main.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Main.Subsystems.MecanumDrivetrain;

public class ChangeDTSpeed extends CommandBase {
    MecanumDrivetrain mecanumDrivetrain;
    public ChangeDTSpeed(MecanumDrivetrain mecanumDrivetrain) {
        this.mecanumDrivetrain = mecanumDrivetrain;
    }

    public void initialize() {
        mecanumDrivetrain.setLowPowerMode(!mecanumDrivetrain.isLowPowerMode());
    }

    public boolean isFinished() {
        return true;
    }
}
