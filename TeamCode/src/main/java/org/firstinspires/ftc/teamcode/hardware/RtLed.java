package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.RtTypes;


public class RtLed {
    private Telemetry m_telemetry;
    private Servo m_ledServo;

    public RtLed(Servo parServo, Telemetry parTelemetry) {
        m_ledServo = parServo;
        m_telemetry = parTelemetry;
    }
    public void setColor(String parColor){
        if ( hwExists()) {
            String color = parColor.toLowerCase();
            double colorValue = 0;

            if (parColor.equals("red")) colorValue = 0.279;
            else if (parColor.equals("orange")) colorValue = 0.333;
            else if (parColor.equals("yellow")) colorValue = 0.388;
            else if (parColor.equals("sage")) colorValue = 0.444;
            else if (parColor.equals("green")) colorValue = 0.5;
            else if (parColor.equals("azure")) colorValue = 0.555;
            else if (parColor.equals("blue")) colorValue = 0.611;
            else if (parColor.equals("indigo")) colorValue = 0.666;
            else if (parColor.equals("violet")) colorValue = 0.722;
            else if (parColor.equals("white")) colorValue = 1.0;
            else colorValue = 0;

            m_ledServo.setPosition(colorValue);
        }
    }

    public void setColor(RtTypes.rtColor parColor)
    {
        switch(parColor)
        {
            case RED:
                setColor("red");
                break;
            case BLUE:
                setColor("blue");
                break;
            case YELLOW:
                setColor("yellow");
                break;
            case GREEN:
                setColor("green");
                break;
            case VIOLET:
                setColor("violet");
                break;
            case AZURE:
                setColor("azure");
                break;
            default:
                setColor("off");
                break;
        }
    }
    private boolean hwExists() {
        boolean exists = true;

        if (m_ledServo == null)
        {
            exists = false;
            m_telemetry.addLine("RtLed HW NOT CONNECTED");
            //m_telemetry.update();
        }
        return exists;
    }
}
