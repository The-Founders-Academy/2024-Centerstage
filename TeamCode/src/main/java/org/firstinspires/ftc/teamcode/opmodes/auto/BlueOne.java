package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.drivetrain.auto.DriveForTime;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@Autonomous(name="BlueOne", preselectTeleOp ="2024TeleOp")
public class BlueOne extends OpMode {
    private MecanumDrivetrain m_drivetrain;
    private Timer m_schoochtimer = new Timer(2);
    private Timer m_driveTimer = new Timer(6);
    private boolean debounce = false;
    private boolean debounceTwo = false;

    @Override
    public void init() {
        DriverStation.getInstance().telemetry = telemetry;
        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL",
                "fR", "bL", "bR");
        CommandScheduler.getInstance().schedule(new DriveForTime(m_drivetrain, 0, -0.3));
    }

    @Override
    public void loop() {
        if(debounce == false) {
            m_schoochtimer.start();
            debounce = true;
        }
        if(m_schoochtimer.done() && debounceTwo == false) {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().schedule(new DriveForTime(m_drivetrain, 0.7, 0));
            m_driveTimer.start();
            debounceTwo = true;
        }

        if(m_driveTimer.done()) {
            CommandScheduler.getInstance().cancelAll();
            terminateOpModeNow();
        }
        CommandScheduler.getInstance().run();
    }
}
