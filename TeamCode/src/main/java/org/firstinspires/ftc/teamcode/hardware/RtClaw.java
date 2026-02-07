package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RtClaw {
    private Telemetry m_telemetry;
    private Servo m_leftClaw;
    private Servo m_rightClaw;
    public RtClaw(Servo parLeftClaw, Servo parRightClaw, Telemetry parTelemetry) {
        m_leftClaw = parLeftClaw;
        m_rightClaw = parRightClaw;
        m_telemetry = parTelemetry;
    }

    public void claw(boolean parOpenLeftClaw, boolean parOpenRightClaw) {
        if (hwExists()) {
            if (parOpenLeftClaw) {
                m_leftClaw.setPosition(1);
                m_rightClaw.setPosition(0);
            }
            if (parOpenRightClaw) {
                m_leftClaw.setPosition(0);
                m_rightClaw.setPosition(1);
            }
        }
    }

    public void open() {
        if (hwExists()) {
            m_telemetry.addLine("RtClaw::open MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }

    public void close() {
        if(hwExists()) {
            m_telemetry.addLine("RtClaw::close MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }
    private boolean hwExists() {
        boolean exists = true;
        if (m_leftClaw == null || m_rightClaw == null)
        {
            exists = false;
            m_telemetry.addLine("RtClaw HW NOT CONNECTED");
            m_telemetry.update();
        }
        return exists;
    }
}