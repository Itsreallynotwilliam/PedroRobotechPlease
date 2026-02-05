package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RtLift {
    private Telemetry m_telemetry;
    private DcMotor m_leftLiftMotor;
    private DcMotor m_rightLiftMotor;
    private DcMotor m_top1Motor;
    private DcMotor m_top2Motor;

    public RtLift(DcMotor parLeftLift, DcMotor parRightLift, DcMotor parTop1, DcMotor parTop2, Telemetry parTelemetry) {
        m_leftLiftMotor = parLeftLift;
        m_rightLiftMotor = parRightLift;
        m_top1Motor = parTop1;
        m_top2Motor = parTop2;
        m_telemetry = parTelemetry;
    }
    public void lift(float parPower){
        if (hwExists()) {
            m_leftLiftMotor.setPower(parPower);
            m_rightLiftMotor.setPower(-parPower);

            m_top1Motor.setPower(parPower);
            m_top2Motor.setPower(-parPower);
        }
    }
    public void moveUp(){
        if (hwExists()) {
            m_telemetry.addLine("RtLift::moveUp MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }

    public void moveDown(){
        if (hwExists()) {
            m_telemetry.addLine("RtLift::moveDown MISSING IMPLEMENTATION");
            m_telemetry.update();
        }
    }
    private boolean hwExists() {
        boolean exists = true;

        if (m_leftLiftMotor == null || m_rightLiftMotor == null ||
            m_top1Motor     == null || m_top2Motor      == null)
        {
            exists = false;
            m_telemetry.addLine("RtLift HW NOT CONNECTED");
            m_telemetry.update();
        }
        return exists;
    }
}
