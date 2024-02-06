package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class ResetPose extends CommandBase {
    private MecanumDrivetrain m_drivetrain;
    public ResetPose(MecanumDrivetrain drivetrain) {
       m_drivetrain = drivetrain;
       addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.resetHeading();

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
