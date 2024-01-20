package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.drivetrain.auto.DriveForTime;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@Autonomous(name="RedOne", preselectTeleOp ="2024TeleOp")
public class RedOne extends OpMode {
    private MecanumDrivetrain m_drivetrain;
    private Timer m_timer = new Timer(2);
    private boolean debounce = false;

    @Override
    public void init() {
        DriverStation.getInstance().telemetry = telemetry;
        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL",
                "fR", "bL", "bR");
        CommandScheduler.getInstance().schedule(new DriveForTime(m_drivetrain, 1, -0.1));
    }

    @Override
    public void loop() {
        if(debounce == false) {
            m_timer.start();
            debounce = true;
        }
        if(m_timer.done()) {
            terminateOpModeNow();
        }
        CommandScheduler.getInstance().run();
    }
}
