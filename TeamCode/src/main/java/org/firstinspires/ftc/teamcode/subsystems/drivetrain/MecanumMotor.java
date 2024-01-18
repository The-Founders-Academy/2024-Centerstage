package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

public class MecanumMotor {
    private MotorEx m_motor;

    public MecanumMotor(MotorEx motor) {
        m_motor = motor;
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_motor.encoder.setDistancePerPulse(Constants.MecanumConstants.DistancePerEncoderTick);
        m_motor.setRunMode(Motor.RunMode.VelocityControl);
        m_motor.setVeloCoefficients(0.01, 0, 0);

    }

    public double getVelocity() {
        return m_motor.getVelocity() * Constants.MecanumConstants.DistancePerEncoderTick;
    }

    // Velocity in meters per second
    public void setTargetVelocity(double targetVelocity) {
        m_motor.setVelocity(targetVelocity / Constants.MecanumConstants.DistancePerEncoderTick);
    }

    public void setInverted(boolean val) {
        m_motor.setInverted(val);
    }

    public Encoder getEncoder() {
        return m_motor.encoder;
    }



}
