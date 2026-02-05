package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.RtTypes;

@TeleOp
public class Robotech_Auton_V2 extends OpMode {
    Robotech m_robotech;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private boolean shotstriggered = false;

    public enum PathState{
        // START POSITION_ENDPOSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startPose = new Pose(123,123,Math.toRadians(36));
    private final Pose shootPose = new Pose(94,95,Math.toRadians(36));

    private PathChain driveStartPosShootPos;

    public void buildpaths(){
        //put ini coordinates for starting pos > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer & make new state
            break;
            case SHOOT_PRELOAD:
                //check is follower done it's path?
                if (!follower.isBusy()){

                    //TODO add logic to flywheel shooter
                    m_robotech.rtLaunch.launchArtifact();
                    m_robotech.rtIntake.runMidtake();
                    telemetry.addLine("Done Path 1");
                    //Transition next state
                }
                break;
            default:
                telemetry.addLine("No state Commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotstriggered = false;
    }


    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // TODO add in any other init mechanisms
        m_robotech = new Robotech(hardwareMap, telemetry);

        buildpaths();
        follower.setPose(startPose);
    }
    public void start(){
        m_robotech.rtLedLight.setColor(RtTypes.rtColor.AZURE);
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop() {

        follower.update();
        
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        

    }
}
