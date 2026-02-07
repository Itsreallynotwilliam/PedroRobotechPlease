package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.RtTypes;

@Autonomous
public class Robotech_Auton_v1 extends LinearOpMode {

    //robotech hw class, common to all opmodes
    Robotech m_robotech;

    @Override
    public void runOpMode(){

        //create all the robotech hardware instances and initialize
        m_robotech = new Robotech(hardwareMap, telemetry);

        //wait for operator to press start on driver hub
        waitForStart();

        //run auton...
        while(!isStopRequested() && opModeIsActive()) {

            if (m_robotech.m_initialPosition == RtTypes.rtInitPosition.AT_GOAL) {
                m_robotech.rtDriveTrain.moveBackward(RtTypes.RT_SLOW_SPEED);
                m_robotech.searchUntilAprilTag();
                m_robotech.rtDriveTrain.stop();
                m_robotech.rtLaunch.launchArtifact();
                m_robotech.rtTime.delayTime(5);
                m_robotech.rtLaunch.stop();
                // after launching go retrieve more artifacts
            }
            else //if (m_robotech.m_initialPosition == RtTypes.rtInitPosition.ACROSS_GOAL)
            {
                m_robotech.rtDriveTrain.moveForward(RtTypes.RT_SLOW_SPEED);
                m_robotech.searchUntilColor(m_robotech.m_allianceColor);
                m_robotech.searchUntilAprilTag();
                m_robotech.rtDriveTrain.stop();
                m_robotech.rtLaunch.launchArtifact();
                m_robotech.rtTime.delayTime(5);
                m_robotech.rtLaunch.stop();
                // after launching go retrieve more artifacts
            }

            telemetry.update();
        }
    }
}

