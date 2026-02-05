package org.firstinspires.ftc.teamcode.utilities;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RtTime {
    private ElapsedTime m_timer;
    public RtTime() {
        m_timer = new ElapsedTime();
    }

    public void delayTime(long parSeconds){
        long MSEC_IN_SEC = 1000;
        sleep(parSeconds * MSEC_IN_SEC);
    }

    public void delayTime(float parSeconds){
        m_timer.reset();
        while ( m_timer.time() < parSeconds ) {
            //  do nothing
            sleep(1000);
        }
    }
}
