package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utilities.RtLog;




public class RtDrive {
    private Telemetry m_telemetry;
    private IMU m_imu;
    private DcMotor m_dtLeftBackMotor;
    private DcMotor m_dtRightBackMotor;
    private DcMotor m_dtRightFrontMotor;
    private DcMotor m_dtLeftFrontMotor;

    public RtDrive(DcMotor parLbMotor, DcMotor parRbMotor, DcMotor parRfMotor, DcMotor parLfMotor, Telemetry parTelemetry, IMU parIMU) {
        m_dtLeftBackMotor   = parLbMotor;
        m_dtRightBackMotor  = parRbMotor;
        m_dtRightFrontMotor = parRfMotor;
        m_dtLeftFrontMotor  = parLfMotor;
        m_telemetry         = parTelemetry;
        m_imu               = parIMU;
    }

    public void printDtPower()
    {
        if (hwExists()) {
            m_telemetry.addData("LFMotor Power Drive", "%.2f", m_dtLeftFrontMotor.getPower());
            m_telemetry.addData("RFMotor Power Drive", "%.2f", m_dtRightFrontMotor.getPower());
            m_telemetry.addData("LBMotor Power Drive", "%.2f", m_dtLeftBackMotor.getPower());
            m_telemetry.addData("RBMotor Power Drive", "%.2f", m_dtRightBackMotor.getPower());
            m_telemetry.update();
        }
    }

    //FC - field centric
    public void driveFC(double parX, double parY, double parRx, boolean parReset) {
        parY  = -parY;
        parRx = -parRx;

        // Retrieve the IMU from the hardware map

        // Adjust the orientation parameters to match your robot

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (parReset) {
            m_imu.resetYaw();
        }

        double botHeading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        m_telemetry.addData("Botheading", "double", botHeading);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = parX * Math.cos(-botHeading) - parY * Math.sin(-botHeading);
        double rotY = parX * Math.sin(-botHeading) + parY * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(parRx), 1);
//        double frontLeftPower = (rotY - rotX - parRx) / denominator;
//        double backLeftPower = (rotY + rotX - parRx) / denominator;
//        double frontRightPower = (rotY + rotX + parRx) / denominator;
//        double backRightPower = (rotY - rotX + parRx) / denominator;
        double frontLeftPower = (rotY + rotX + parRx) / denominator;
        double backLeftPower = (rotY - rotX + parRx) / denominator; 
        double frontRightPower = (rotY - rotX - parRx) / denominator;
        double backRightPower = (rotY + rotX - parRx) / denominator;
        // we changed the positive to negative. Wheel position is different than GM 0

        if (hwExists()) {
            m_dtLeftFrontMotor.setPower(frontLeftPower);
            m_dtLeftBackMotor.setPower(backLeftPower);
            m_dtRightFrontMotor.setPower(frontRightPower);
            m_dtRightBackMotor.setPower(backRightPower);
        }
    }
    //RC - robot centric
    public void driveRC(float parX, float parY, float parRotation, boolean parBoostSpeed){

        parY = -parY; //make negative y, forward

        final double QUARTER_PI = Math.PI/4;
        final double NOMINAL_SPEED = 1;
        final double TURBO_SPEED = 0.9;
        final double MAX_POWER = 1.0;

        double theta = Math.atan2(parY, parX);
        double power = Math.hypot(parX, parY);
        double sin = Math.sin(theta - QUARTER_PI), cos = Math.cos(theta - QUARTER_PI);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        if (hwExists()) {
            m_dtLeftFrontMotor.setPower((power * cos / max + parRotation) * NOMINAL_SPEED);
            m_dtRightFrontMotor.setPower((power * sin / max - parRotation) * NOMINAL_SPEED);
            m_dtLeftBackMotor.setPower((power * sin / max + parRotation) * NOMINAL_SPEED);
            m_dtRightBackMotor.setPower((power * cos / max - parRotation) * NOMINAL_SPEED);

            if ((power + Math.abs(parRotation)) > MAX_POWER) {
                m_dtLeftFrontMotor.setPower(m_dtLeftFrontMotor.getPower() / (power + parRotation));
                m_dtRightFrontMotor.setPower(m_dtRightFrontMotor.getPower() / (power + parRotation));
                m_dtRightBackMotor.setPower(m_dtRightBackMotor.getPower() / (power + parRotation));
                m_dtLeftBackMotor.setPower(m_dtLeftBackMotor.getPower() / (power + parRotation));
            }

            if (parBoostSpeed) {
                m_dtLeftFrontMotor.setPower((power * cos / max + parRotation) * TURBO_SPEED);
                m_dtRightFrontMotor.setPower((power * sin / max - parRotation) * TURBO_SPEED);
                m_dtLeftBackMotor.setPower((power * sin / max + parRotation) * TURBO_SPEED);
                m_dtRightBackMotor.setPower((power * cos / max - parRotation) * TURBO_SPEED);
            }
        }
    }

    public void stop(){
        if ( hwExists()) {
            m_dtLeftFrontMotor.setPower(0);
            m_dtRightFrontMotor.setPower(0);
            m_dtLeftBackMotor.setPower(0);
            m_dtRightBackMotor.setPower(0);
        }
    }
    public void moveRight(float parSpeed){
        if (hwExists()) {
            driveRC(parSpeed,0,0,false);
        }
    }

    public void moveLeft(float parSpeed){
        if (hwExists()) {
            driveRC(-parSpeed,0,0,false);
        }
    }

    public void moveForward(float parSpeed){
        if (hwExists()) {
            driveRC(0,parSpeed,0,false);
        }
    }

    public void moveBackward(float parSpeed){
        if (hwExists()) {
            driveRC(0,-parSpeed,0,false);
        }
    }

    public void turnClockwise(long parRotationInDegrees){
        if (hwExists()) {
            driveRC(0,0,parRotationInDegrees,false);
        }
    }

    public void turnCounterClockwise(long parRotationInDegrees){
        if (hwExists()) {
            driveRC(0,0,-parRotationInDegrees,false);
        }
    }
    private boolean hwExists() {
        boolean exists = true;

        if (m_dtLeftBackMotor   == null || m_dtRightBackMotor == null ||
            m_dtRightFrontMotor == null || m_dtLeftFrontMotor == null )
        {
            exists = false;
            m_telemetry.addLine("RtDrive HW NOT CONNECTED");
            //m_telemetry.update();
        }
        return exists;
    }
}
