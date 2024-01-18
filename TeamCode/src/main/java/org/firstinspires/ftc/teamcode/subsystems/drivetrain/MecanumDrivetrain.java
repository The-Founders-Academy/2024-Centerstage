package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

import java.sql.Driver;

// SMART DASHBOARD IP: 192.168.43.1:8080/dash
public class MecanumDrivetrain extends SubsystemBase {
    private MecanumMotor m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    private Pose2d m_pose;
    private IMU m_imu;
    private Timer m_elapsedTime;
    private MultipleTelemetry multiTelemetry = new MultipleTelemetry(DriverStation.getInstance().telemetry, FtcDashboard.getInstance().getTelemetry());


    // private final Rotation2d m_angleOffset = (DriverStation.getInstance().alliance == DriverStation.Alliance.BLUE) ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    public MecanumDrivetrain(Pose2d initialPose, HardwareMap hardwareMap, String frontLeftName, String frontRightName, String backLeftName, String backRightName) {
        // Initialize hardware

        m_frontLeft = new MecanumMotor(new MotorEx(hardwareMap, frontLeftName, Motor.GoBILDA.RPM_312));
        m_frontRight = new MecanumMotor(new MotorEx(hardwareMap, frontRightName, Motor.GoBILDA.RPM_312));
        m_backLeft = new MecanumMotor(new MotorEx(hardwareMap, backLeftName, Motor.GoBILDA.RPM_312));
        m_backRight = new MecanumMotor(new MotorEx(hardwareMap, backRightName, Motor.GoBILDA.RPM_312));

        m_backLeft.setInverted(true);
        m_frontLeft.setInverted(true);

        // m_vision = new Vision(hardwareMap);
        m_imu = hardwareMap.get(IMU.class, "imu");
        m_imu.initialize(
                new IMU.Parameters(
                        // TO-DO: When we actually mount the control hub, we will need to know the actual values for this
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        // Check if we have a non-null starting pose
        if(initialPose != null) {
            m_pose = initialPose;
        } else {
            m_pose = new Pose2d(); // Pose at (0,0,0)
        }

        // Initialize kinematics & odometry
        m_kinematics = new MecanumDriveKinematics(
                DrivetrainConstants.FrontLeftMotorLocation, DrivetrainConstants.FrontRightMotorLocation,
                DrivetrainConstants.BackLeftMotorLocation, DrivetrainConstants.BackRightMotorLocation
                );

        m_odometry = new MecanumDriveOdometry(
                m_kinematics, new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                m_pose
        );


        // We only need a timer object to call m_odometry.updateWithTime(), so the specific length doesn't matter, as long as it lasts longer than an FTC match.
        m_elapsedTime = new Timer(1200); // 20 minutes
        m_elapsedTime.start();
    }

    public Pose2d getPose() {
        return m_pose;
    }

    private void move(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
        m_frontLeft.setTargetVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setTargetVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setTargetVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setTargetVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }

    // positive x = away from you
    // positive y = to your left
    public void moveFieldRelative(double velocityXPercent, double velocityYPercent, double omegaPercent) {
        double velocityXMetersPerSecond = -velocityXPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double velocityYMetersPerSecond = velocityYPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double omegaRadiansPerSecond = -omegaPercent * DrivetrainConstants.MaxAngularVeloityRadiansPerSecond;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocityYMetersPerSecond, velocityXMetersPerSecond, omegaRadiansPerSecond, getHeading());
        move(speeds);
    }

    @Override
    public void periodic() {
        updatePose();

        multiTelemetry.addData("RobotPoseX", getPose().getX());
        multiTelemetry.addData("RobotPoseY", getPose().getY());
        multiTelemetry.addData("RobotAngleRad", getPose().getRotation().getRadians());
    }

    /**
     * This function does not return the absolute rotational position of the robot. It only returns how far the robot has rotated since it started or the last
     * resetHeading() call.
     * Boundaries: -360 deg or -PI rad to +360 deg or +PI rad
     * @return The current heading of the robot
     */
    public Rotation2d getHeading() {
        return new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    // This function is used by the mecanumDrivetrain class itself to update its position on the field. It works by
    // first asking if the vision object can read an april tag. If so, it will use the position data provided by that april tag. Otherwise,
    // It will use the odometry object to update the robot's postion.
    private void updatePose() {
        // Check to see if we saw and read an april tag
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                m_frontLeft.getVelocity(), m_frontRight.getVelocity(),
                m_backLeft.getVelocity(), m_backRight.getVelocity()
            );

        // Our coordinate system is flipped (see above)
        Pose2d m_falsePose = m_odometry.updateWithTime(DriverStation.getInstance().ElapsedTime.elapsedTime(), getHeading(), wheelSpeeds);
        m_pose = new Pose2d(-m_falsePose.getY(), m_falsePose.getX(), m_falsePose.getRotation());
    }

    public void resetHeading() {
        m_imu.resetYaw();
    }

    public void moveToTarget() {

    }

    public void setTarget(double x, double y, Rotation2d rotation) {

    }

    public void tunePIDs() {
    }

    public boolean atTarget() {
        return false;
    }
}