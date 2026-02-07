package org.firstinspires.ftc.teamcode.utilities;


import org.firstinspires.ftc.robotcore.external.Telemetry;


public class RtLog {
    private Telemetry m_telemetry;
    public RtLog(Telemetry parTelemetry) {
        m_telemetry = parTelemetry;
    }

    public void print(String parTelemetry){
        m_telemetry.addLine(parTelemetry);
        m_telemetry.update();
    }

    public void print(String caption, String format, Object... args) {
       m_telemetry.addData(caption, format, args);
       m_telemetry.update();
    }

}
