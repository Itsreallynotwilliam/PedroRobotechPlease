package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.RtTypes;

@TeleOp
public class Robotech_Teleop_v0 extends LinearOpMode {

    //class attributes

    //robotech hw class, common to all opmodes
    Robotech m_robotech;

    @Override
    public void runOpMode(){
        boolean lastRobotCentricToggleValue = true;
        boolean isRobotCentric = false;

        //create all the robotech hardware instances and initialize
        m_robotech = new Robotech(hardwareMap, telemetry);

        //wait for operator to press start on driver hub
        waitForStart();

        while(!isStopRequested()) {

            m_robotech.rtLedLight.setColor(RtTypes.rtColor.AZURE);
            //toggle between robot centric and field centric when x button is pushed
            boolean fieldCentricToggle = gamepad1.x;
            if (!lastRobotCentricToggleValue && fieldCentricToggle){
                isRobotCentric = !isRobotCentric;
            }
            lastRobotCentricToggleValue = fieldCentricToggle;

            //drive train
            float xDirection = gamepad1.left_stick_x;
            float yDirection = gamepad1.left_stick_y;
            float rotation   = gamepad1.right_stick_x;
            if (isRobotCentric) {
                telemetry.addLine("DT Robot Centric");
                boolean boostSpeed = gamepad1.right_bumper;
                m_robotech.rtDriveTrain.driveRC(xDirection, yDirection, rotation, boostSpeed);
            }
            else //isFieldCentric
            {
                telemetry.addLine("DT Field Centric");
                boolean reset = gamepad1.options;
                m_robotech.rtDriveTrain.driveFC(xDirection, yDirection, rotation, reset);
            }

            //intake
            boolean retrieveToggle = gamepad2.a;
            boolean expelToggle    = gamepad2.b;
            boolean midtakeToggle  = gamepad2.y;
            m_robotech.rtIntake.intake(retrieveToggle, expelToggle, midtakeToggle);

            //launcher
            boolean launcherToggle = gamepad2.right_bumper;
            m_robotech.rtLaunch.launch(launcherToggle);

            //evaluate if stop button was pushed
            boolean stopAllButton = gamepad2.x;
            if (stopAllButton)
            {
                //stop everything
                m_robotech.rtIntake.stop();
                m_robotech.rtIntake.runMidtake(false);
                m_robotech.rtLaunch.stop();
            }

            //april tag detection
            /*if (m_robotech.rtCamera.detectedAprilTag())
            {
                m_robotech.rtLedLight.setColor(RtTypes.rtColor.GREEN);
                m_robotech.rtLog.print("AprilTag #", "%d %s",
                        m_robotech.rtCamera.m_aprilTagId,
                        m_robotech.rtCamera.m_aprilTagName);
            }
            else
            {
              m_robotech.rtLedLight.setColor(RtTypes.rtColor.OFF);
            }
            */

            telemetry.update();
        }
    }
}
