package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class DriveToPosition extends CommandBase {
    private MecanumDrivetrain m_drivetrain;
    private double m_translationTolerance;
    private Pose2d m_targetPose;

    public DriveToPosition(MecanumDrivetrain drivetrain, Pose2d target, double translationTolerance) {
        m_drivetrain = drivetrain;
        m_targetPose = target;
        m_translationTolerance = translationTolerance;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setTarget(m_targetPose);
        m_drivetrain.setTranslationTolerance(m_translationTolerance);
    }

    @Override
    public void execute() {
        m_drivetrain.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        m_drivetrain.state = "isFInished false";
        return m_drivetrain.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.resetPIDs();
        m_drivetrain.state = "DriveToPos done";
    }
}
