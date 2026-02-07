package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RtLed;
import org.firstinspires.ftc.teamcode.utilities.RtTypes;


public class RtIntake {
    private Telemetry m_telemetry;
    private DcMotor m_intakeMotor;
    private CRServo m_midtakeServo1;
    private CRServo m_midtakeServo2;
    private RtLed m_led;

    private boolean m_intakeRetrieve = false;
    private boolean m_intakeExpel = false;
    private boolean m_midtakeOn = false;

    private boolean m_lastIntakeRetrieveToggle = false;
    private boolean m_lastIntakeExpelToggle = false;
    private boolean m_lastMidtakeOnToggle = false;
    public RtIntake(DcMotor parIntakeMotor, CRServo parMidtakeServo1, CRServo parMidtakeServo2, Telemetry parTelemetry, RtLed parLed) {
        m_intakeMotor   = parIntakeMotor;
        m_midtakeServo1 = parMidtakeServo1;
        m_midtakeServo2 = parMidtakeServo2;
        m_telemetry     = parTelemetry;
        m_led           = parLed;
    }

    public void retrieveArtifact(){
        m_telemetry.addLine("RtIntake retrieve");
        if ( hwExists()) {
            double power = 1.0;
            m_intakeMotor.setPower(power);
        }
    }

    public void expelArtifact(){
        m_telemetry.addLine("RtIntake expel");
        if ( hwExists()) {
            double power = -1.0;
            m_intakeMotor.setPower(power);
        }
    }

    public void stop(){
        m_telemetry.addLine("RtIntake stop");
        if ( hwExists()) {
            m_intakeMotor.setPower(0);
        }
    }

    public void runMidtake(boolean parOn){
        m_telemetry.addData("RtIntake run midtake On=", parOn);
        if (hwExists()) {
            double power = -1.0;
            if(!parOn)
            {
                power = 0;
                m_led.setColor(RtTypes.rtColor.OFF);
            }
            m_midtakeServo1.setPower(power);
            m_midtakeServo2.setPower(-power);
            m_led.setColor("orange");
        }

    }

    public void intake (boolean parRetrieveToggle, boolean parExpelToggle, boolean parMidtakeToggle )
    {
        //apply toggles
        if (!m_lastIntakeRetrieveToggle && parRetrieveToggle) {
            m_intakeRetrieve = !m_intakeRetrieve;
            if (m_intakeExpel)
            {
                m_intakeExpel = false; //can't have both on
            }
        }
        m_lastIntakeRetrieveToggle = parRetrieveToggle;

        if (!m_lastIntakeExpelToggle && parExpelToggle) {
            m_intakeExpel = !m_intakeExpel;
            if (m_intakeRetrieve)
            {
                m_intakeRetrieve = false; //can't have both on
            }
        }
        m_lastIntakeExpelToggle = parExpelToggle;

        if (!m_lastMidtakeOnToggle && parMidtakeToggle) {
            m_midtakeOn = !m_midtakeOn;
        }
        m_lastMidtakeOnToggle = parMidtakeToggle;

        //take action
        runMidtake(m_midtakeOn);

        if (!m_intakeRetrieve && !m_intakeExpel) {
            stop();
        }
        else {
            if (m_intakeRetrieve) {
                retrieveArtifact();
            }
            if (m_intakeExpel) {
                expelArtifact();
            }
        }
    }

    private boolean hwExists() {
        boolean exists = true;

        if (m_intakeMotor == null)
        {
            exists = false;
            m_telemetry.addLine("RtIntake Intake HW NOT CONNECTED");
            //m_telemetry.update();
        }

        if (m_midtakeServo1 == null)
        {
            exists = false;
            m_telemetry.addLine("RtIntake Midtake1 HW NOT CONNECTED");
            //m_telemetry.update();
        }

        if (m_midtakeServo2 == null)
        {
            exists = false;
            m_telemetry.addLine("RtIntake Midtake2 HW NOT CONNECTED");
            //m_telemetry.update();
        }

        if (m_led == null)
        {
            exists = false;
            m_telemetry.addLine("RtLed Midtake HW NOT CONNECTED");
            //m_telemetry.update();
        }
        return exists;
    }
}
