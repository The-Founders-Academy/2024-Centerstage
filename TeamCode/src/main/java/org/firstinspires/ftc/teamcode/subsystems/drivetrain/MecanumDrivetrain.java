package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
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
import org.firstinspires.ftc.teamcode.Constants.MecanumConstants;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

// SMART DASHBOARD IP: 192.168.43.1:8080/dash
public class MecanumDrivetrain extends SubsystemBase {
    private MecanumMotor m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    private Pose2d m_pose;
    private IMU m_imu;
    private Timer m_elapsedTime;
    private MultipleTelemetry multiTelemetry = new MultipleTelemetry(DriverStation.getInstance().telemetry, FtcDashboard.getInstance().getTelemetry());
    private PIDFController m_xPIDF;
    private PIDFController m_yPIDF;

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
                MecanumConstants.FrontLeftMotorLocation, MecanumConstants.FrontRightMotorLocation,
                MecanumConstants.BackLeftMotorLocation, MecanumConstants.BackRightMotorLocation
                );

        m_odometry = new MecanumDriveOdometry(
                m_kinematics, new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                m_pose
        );


        m_xPIDF = new PIDFController(MecanumConstants.XCoefficients.p, MecanumConstants.XCoefficients.i,
                                     MecanumConstants.XCoefficients.d, MecanumConstants.XCoefficients.f);
        m_yPIDF = new PIDFController(MecanumConstants.YCoefficients.p, MecanumConstants.YCoefficients.i,
                MecanumConstants.YCoefficients.d, MecanumConstants.YCoefficients.f);

        m_xPIDF.setTolerance(0.2);
        m_yPIDF.setTolerance(0.2);

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

    // positive y = away from yu
    // positive x = right
    public void moveFieldRelative(double velocityXMps, double velocityYMps, double omegaPercent) {
        double velocityXMetersPerSecond = -velocityXMps;
        double velocityYMetersPerSecond = velocityYMps;
        double omegaRadiansPerSecond = -omegaPercent * MecanumConstants.MaxAngularVeloityRadiansPerSecond;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocityYMetersPerSecond, velocityXMetersPerSecond, omegaRadiansPerSecond, getHeading());
        move(speeds);
    }

    @Override
    public void periodic() {
        updatePose();

        multiTelemetry.addData("RobotPoseX", getPose().getX());
        multiTelemetry.addData("RobotPoseY", getPose().getY());


        multiTelemetry.addData("yPIDF", m_xPIDF);
        multiTelemetry.addData("yPIDF", m_yPIDF);
        multiTelemetry.addData("ySetPoint", m_yPIDF.getSetPoint());
        multiTelemetry.addData("atTarget", Boolean.toString(atTarget()));
        tunePIDs();
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
        double xVelocityMps =  m_xPIDF.calculate(m_pose.getX());
        double yVelocityMps = m_yPIDF.calculate(m_pose.getY());
        moveFieldRelative(xVelocityMps, yVelocityMps, 0);
    }

    public void setTarget(double x, double y, Rotation2d rotation) {
        m_xPIDF.setSetPoint(x);
        m_yPIDF.setSetPoint(y);
    }

    public void tunePIDs() {
        m_xPIDF.setPIDF(MecanumConstants.XCoefficients.p, MecanumConstants.XCoefficients.i,
                MecanumConstants.XCoefficients.d, MecanumConstants.XCoefficients.f);
        m_yPIDF.setPIDF(MecanumConstants.YCoefficients.p, MecanumConstants.YCoefficients.i,
                MecanumConstants.YCoefficients.d, MecanumConstants.YCoefficients.f);
    }

    public boolean atTarget() {
        return m_yPIDF.atSetPoint();
    }

    public void resetPIDs() {
        m_xPIDF.reset();
        m_yPIDF.reset();
    }
}