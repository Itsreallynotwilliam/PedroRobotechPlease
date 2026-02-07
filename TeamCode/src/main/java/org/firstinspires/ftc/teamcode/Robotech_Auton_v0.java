package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.RtTypes;

@Autonomous
public class Robotech_Auton_v0 extends LinearOpMode {

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
            m_robotech.rtDriveTrain.moveForward(0.67f);  //go to observation zone
        }
    }
}

