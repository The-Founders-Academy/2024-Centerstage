package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ResetPose;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@TeleOp(name="2024TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadSubsystem m_driver;
    private GamepadSubsystem m_operator;

    private MecanumDrivetrain m_drivetrain;

    private MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private void driverControls() {
        m_drivetrain.setDefaultCommand(new DriverRelativeDrive(m_drivetrain, m_driver));
        m_driver.buttonA().whenPressed(new ResetPose(m_drivetrain));
        // When the driver presses the A button, drive forward 1 meter. We can use this to test odometry.
        // Score
        // Shoot airplane
    }

    private void operatorControls() {
        // Move arm to pose
        // Intake
    }

    @Override
    public void initialize() {
        setupDriverStation();
        CommandScheduler.getInstance().cancelAll();

        m_driver = new GamepadSubsystem(new GamepadEx(gamepad1));
        m_operator = new GamepadSubsystem(new GamepadEx(gamepad2));

        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL", "fR", "bL", "bR");
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        multiTelemetry.update();
    }

    private void setupDriverStation() {
        DriverStation.getInstance().telemetry = telemetry;
        if(DriverStation.getInstance().alliance == DriverStation.Alliance.NONE) DriverStation.getInstance().alliance = DriverStation.Alliance.RED;
    }
}
