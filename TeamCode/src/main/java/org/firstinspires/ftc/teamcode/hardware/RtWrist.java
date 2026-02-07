package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RtWrist {

    private Telemetry m_telemetry;
    private Servo m_leftWrist;
    private Servo m_rightWrist;
    public RtWrist(Servo parLeftWrist, Servo parRightWrist, Telemetry parTelemetry) {
        m_leftWrist = parLeftWrist;
        m_rightWrist = parRightWrist;
        m_telemetry = parTelemetry;
    }

    public void wrist(boolean parLiftRightWrist, boolean parLiftLeftWrist) {
        if(hwExists()) {
            if (parLiftRightWrist) {
                m_leftWrist.setPosition(0);
                m_rightWrist.setPosition(1);
            } else if (parLiftLeftWrist) {
                m_leftWrist.setPosition(1);
                m_rightWrist.setPosition(0);
            }
        }
    }

    public void open() {
        if(hwExists()) {
            m_telemetry.addLine("RtWrist::open MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }

    public void close() {
        if(hwExists()) {
            m_telemetry.addLine("RtWrist::close MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }
    private boolean hwExists() {
        boolean exists = true;

        if (m_leftWrist == null || m_rightWrist == null)
        {
            exists = false;
            m_telemetry.addLine("RtWrist HW NOT CONNECTED");
            m_telemetry.update();
        }
        return exists;
    }
}