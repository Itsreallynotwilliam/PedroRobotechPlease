package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.RtTypes;

public class RtColorSensor {
    private Telemetry m_telemetry;
    private RevColorSensorV3 m_colorSensor;

    public RtColorSensor(RevColorSensorV3 parColorSensor, Telemetry parTelemetry)
    {
        m_colorSensor = parColorSensor;
        m_telemetry   = parTelemetry;
    }
    public double getDistance()
    {
        double distance = 0;
        if (hwExists()) {
            distance = m_colorSensor.getDistance(DistanceUnit.INCH);
        }
        return distance;
    }
    public RtTypes.rtColor getColor()
    {
        RtTypes.rtColor color = RtTypes.rtColor.UNKNOWN;

        if (hwExists()) {
            double distance = getDistance();

            //if object is within minimum distance in inches, read color
            if (distance < RtTypes.RT_MIN_OBJECT_DISTANCE) {
                int red = m_colorSensor.red();
                int blue = m_colorSensor.blue();
                int green = m_colorSensor.green();

                if (red > blue && red > green) {
                    color = RtTypes.rtColor.RED;
                } else if (blue > red && blue > green) {
                    color = RtTypes.rtColor.BLUE;
                } else if (green > red && green > blue) {
                    color = RtTypes.rtColor.YELLOW;
                } else {
                    color = RtTypes.rtColor.UNKNOWN;
                }
            }
        }
        return color;
    }
    public boolean isRed()
    {
        boolean objectIsRed = false;
        if ( getColor() == RtTypes.rtColor.RED ) {
            objectIsRed = true;
        }
        return objectIsRed;
    }

    public boolean isBlue()
    {
        boolean objectIsBlue = false;
        if ( getColor() == RtTypes.rtColor.BLUE ) {
            objectIsBlue = true;
        }
        return objectIsBlue;
    }

    public boolean isYellow()
    {
        boolean objectIsYellow = false;
        if ( getColor() == RtTypes.rtColor.YELLOW ) {
            objectIsYellow = true;
        }
        return objectIsYellow;
    }

    private boolean hwExists() {
        boolean exists = true;
        if (m_colorSensor == null)
        {
            exists = false;
            m_telemetry.addLine("RtColorSensor HW NOT CONNECTED");
            //m_telemetry.update();
        }
        return exists;
    }
}


