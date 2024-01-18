package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

import java.util.function.Supplier;

public class DriverRelativeDrive extends CommandBase {
    private GamepadSubsystem m_driver;
    private MecanumDrivetrain m_drivetrain;
    public DriverRelativeDrive(MecanumDrivetrain mecanumDrive, GamepadSubsystem driver) {
        m_driver = driver;
        m_drivetrain = mecanumDrive;
        addRequirements(m_drivetrain, m_driver); // This may be a point of failure and may need testing.
    }

    @Override
    public void execute() {
        double horizontalPercent = m_driver.getLeftX();
        double verticalPercent = m_driver.getLeftY();
        double omegaPercent = m_driver.getRightX() * 0.5;
        m_drivetrain.moveFieldRelative(horizontalPercent, verticalPercent, omegaPercent);
    }
}
