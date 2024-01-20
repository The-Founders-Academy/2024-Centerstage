package org.firstinspires.ftc.teamcode.commands.drivetrain.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

public class DriveForTime extends CommandBase {
    private MecanumDrivetrain m_mecanumDrive;
    private double m_xVelocity, m_yVelocity;
    public DriveForTime(MecanumDrivetrain drivetrain, double xVel, double yVel) {
        m_mecanumDrive = drivetrain;
        m_xVelocity = xVel;
        m_yVelocity = yVel;

        addRequirements(m_mecanumDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_mecanumDrive.moveFieldRelative(m_xVelocity, m_yVelocity, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_mecanumDrive.moveFieldRelative(0, 0, 0);
    }
}
