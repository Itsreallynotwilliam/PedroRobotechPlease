package org.firstinspires.ftc.teamcode.utilities;




public class RtTypes {
    public static final double RT_MIN_OBJECT_DISTANCE = 1.5;

    public static final float RT_SLOW_SPEED = 0.67f;
    public static final float RT_MAX_SPEED = 1.0f;

    public enum rtColor {
        RED,
        BLUE,
        YELLOW,
        GREEN,
        VIOLET,
        AZURE,
        OFF,
        UNKNOWN
    }
    public enum rtInitPosition {
        UNKNOWN,
        AT_GOAL,
        ACROSS_GOAL
    }
    static public String getColorText(rtColor parColorEnum)
    {
        switch(parColorEnum){
            case RED: return "RED";
            case BLUE: return "BLUE";
            default: return "UNKNOWN";
        }
    }
    static public String getInitPosText(rtInitPosition parInitPosEnum)
    {
        switch(parInitPosEnum)
        {
            case AT_GOAL: return "AT GOAL";
            case ACROSS_GOAL: return "ACROSS GOAL";
            default: return "UNKNOWN";
        }
    }
}
