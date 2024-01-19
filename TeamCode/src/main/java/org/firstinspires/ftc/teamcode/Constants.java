package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Constants {

    /*
    Wheel center distances:
    ----------------------
    horizontal: 178 mm
    vertical: 168 mm
     */
    @Config
    public static class MecanumConstants {
        public static final double MaxRobotSpeedMetersPerSecond = 1.5; // Theoretical value from the strafer chassis product page
        public static final double MaxAngularVeloityRadiansPerSecond = 2*Math.PI;
        public static final Translation2d FrontLeftMotorLocation = new Translation2d(0.178, 0.168);
        public static final Translation2d FrontRightMotorLocation = new Translation2d(0.178, -0.168);
        public static final Translation2d BackLeftMotorLocation = new Translation2d(-0.178, 0.168);
        public static final Translation2d BackRightMotorLocation = new Translation2d(-0.178, -0.168);
        public static final double DistancePerEncoderTick = 0.00056; // 0.56 mm per pulse
    }
}
