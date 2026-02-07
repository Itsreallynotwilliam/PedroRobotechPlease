package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RtLaunch {
    private Telemetry m_telemetry;
    private DcMotorEx m_launchMotor1;
    private DcMotorEx m_launchMotor2;

    private boolean m_launchToggle = false;
    private boolean m_lastLaunchToggle = false;
    final double LAUNCHER_TARGET_VELOCITY = 900;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    public RtLaunch(DcMotorEx parLaunchMotor1, DcMotorEx parLaunchMotor2, Telemetry parTelemetry) {
        m_launchMotor1 = parLaunchMotor1;
        m_launchMotor2 = parLaunchMotor2;
        m_telemetry = parTelemetry;

        if (hwExists()) {
            m_launchMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_launchMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_launchMotor1.setZeroPowerBehavior(BRAKE);
            m_launchMotor2.setZeroPowerBehavior(BRAKE);
            m_launchMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
            m_launchMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        }
    }

    public void launchArtifact() {
        m_telemetry.addLine("RtLaunch launch");
        if (hwExists()) {
            m_launchMotor1.setVelocity(LAUNCHER_TARGET_VELOCITY);
            m_launchMotor2.setVelocity(-1*LAUNCHER_TARGET_VELOCITY);
        }
    }

    public void stop() {
        m_telemetry.addLine("RtLaunch stop");
        if (hwExists()) {
            m_launchMotor1.setPower(0);
            m_launchMotor2.setPower(0);
        }
    }

    public void launch(boolean parLaunchToggle)
    {
        //apply toggle
        if (!m_lastLaunchToggle && parLaunchToggle) {
            m_launchToggle = !m_launchToggle;
        }
        m_lastLaunchToggle = parLaunchToggle;

        //take action
        if (m_launchToggle) {
            launchArtifact();
        } else {
            stop();
        }
    }
    private boolean hwExists() {
        boolean exists = true;

        if (m_launchMotor1 == null )
        {
            exists = false;
            m_telemetry.addLine("RtLaunch motor1 HW NOT CONNECTED");
            //m_telemetry.update();
        }
        if (m_launchMotor2 == null)
        {
            exists = false;
            m_telemetry.addLine("RtLaunch motor2 HW NOT CONNECTED");
            //m_telemetry.update();
        }
        return exists;
    }
}
