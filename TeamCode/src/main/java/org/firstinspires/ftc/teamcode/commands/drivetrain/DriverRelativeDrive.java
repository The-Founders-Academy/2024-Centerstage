package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

import java.util.function.Supplier;

public class DriverRelativeDrive extends CommandBase {

    private Supplier<Double> vX;
    private Supplier<Double> vY;
    private Supplier<Double> omegaRadiansPerSecond;

    private MecanumDrivetrain m_drivetrain;

    public DriverRelativeDrive(MecanumDrivetrain mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> omega) {
        vX = x;
        vY = y;
        omegaRadiansPerSecond = omega;
        m_drivetrain = mecanumDrive;
        addRequirements(m_drivetrain); // This may be a point of failure and may need testing.
    }

    @Override
    public void execute() {
        DriverStation.getInstance().telemetry.addData("Joystick right x", omegaRadiansPerSecond.get());
        m_drivetrain.moveFieldRelative(vX.get(), vY.get(), omegaRadiansPerSecond.get());
    }
}
