package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class DriveToPosition extends CommandBase {
    private MecanumDrivetrain m_mecanumDrive;
    public DriveToPosition(MecanumDrivetrain drivetrain, double x, double y, Rotation2d rotation) {
        m_mecanumDrive = drivetrain;

        addRequirements(m_mecanumDrive);
        m_mecanumDrive.setTarget(x, y, Rotation2d.fromDegrees(0));
    }

    @Override
    public void execute() {
        m_mecanumDrive.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return m_mecanumDrive.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        m_mecanumDrive.resetPIDs();
    }
}
