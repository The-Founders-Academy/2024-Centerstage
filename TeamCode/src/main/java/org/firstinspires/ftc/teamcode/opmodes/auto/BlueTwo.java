package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.drivetrain.auto.DriveForTime;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@Autonomous(name="BlueTwo", preselectTeleOp ="2024TeleOp")
public class BlueTwo extends OpMode {
    boolean debounce = false;
    private MecanumDrivetrain m_drivetrain;
    Timer t = new Timer(3);
    @Override
    public void init() {
        // Init code
        DriverStation.getInstance().telemetry = telemetry;
        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL",
                "fR", "bL", "bR");
        CommandScheduler.getInstance().schedule(new DriveForTime(m_drivetrain,  0.5, 0.15));
        t.start();
    }

    @Override
    public void loop() {
        if(debounce == false) {
            t.start();
            debounce = true;
        }
        if(t.done()) {
            terminateOpModeNow();
        }
        CommandScheduler.getInstance().run();
    }
}