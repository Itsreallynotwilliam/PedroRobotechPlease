package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RtSound {
    private HardwareMap m_hardwareMap;
    public RtSound(HardwareMap parHardwareMap) {
        m_hardwareMap = parHardwareMap;
    }
    public void play(String parSoundName){
        int sound = m_hardwareMap.appContext.getResources().getIdentifier(parSoundName, "raw", m_hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(m_hardwareMap.appContext, sound);
    }

}
