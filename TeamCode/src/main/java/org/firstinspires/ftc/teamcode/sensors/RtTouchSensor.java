package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RtTouchSensor {
    private Telemetry m_telemetry;
    private TouchSensor m_touchSensor;

    public RtTouchSensor(TouchSensor parTouchSensor, Telemetry parTelemetry){
        m_touchSensor = parTouchSensor;
        m_telemetry = parTelemetry;
    }

    public boolean isTouched()
    {
        boolean touched = false;
        if (hwExists()) {
            touched = m_touchSensor.isPressed();
        }
        return touched;
    }

    private boolean hwExists() {
        boolean exists = true;
        if (m_touchSensor == null)
        {
            exists = false;
            m_telemetry.addLine("RtTouchSensor HW NOT CONNECTED");
            m_telemetry.update();
        }
        return exists;
    }

}
