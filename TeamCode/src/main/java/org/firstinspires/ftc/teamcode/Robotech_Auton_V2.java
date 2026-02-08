package org.firstinspires.ftc.teamcode;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilities.RtTypes;

@Autonomous
public class Robotech_Auton_V2 extends OpMode {
    Robotech m_robotech;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int m_allianceIdx;

//    private boolean shotstriggered = false;

    public enum PathState{
        // START POSITION_ENDPOSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        TRANSITION,
        PICKUP,
        SECOND_SHOOT_MOVE,
        SHOOT_PRELOAD2

    }

    PathState pathState;

    private final Pose[] startPose = new Pose[]{new Pose(24, 126, Math.toRadians(-37)), new Pose(24, 126, Math.toRadians(-37))};
    private final Pose[] shootPose = new Pose[]{new Pose(52,115,Math.toRadians(-31)), new Pose(52,115,Math.toRadians(-31))};
    private final Pose[] transitionPose = new Pose[]{new Pose(73,84,Math.toRadians(180)),new Pose(73,84,Math.toRadians(180))};
    private final Pose[] pickupPose = new Pose[]{new Pose(17,84,Math.toRadians(180)),new Pose(17,84,Math.toRadians(180))};
    private final Pose[] secondShootPose = new Pose[]{new Pose(52,115,Math.toRadians(-31)),new Pose(52,115,Math.toRadians(-31))};



    private PathChain driveStartPosShootPos,TransitionPos,PickUpPos,SecondShootPos;


    public void buildpaths(){
        m_allianceIdx = 0; //BLUE
        if ( m_robotech.m_allianceColor == RtTypes.rtColor.RED) {
            m_allianceIdx = 1; //RED
        }

        //put ini coordinates for starting pos > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose[m_allianceIdx], shootPose[m_allianceIdx]))
                .setLinearHeadingInterpolation(startPose[m_allianceIdx].getHeading(), shootPose[m_allianceIdx].getHeading())
                .build();
        TransitionPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose[m_allianceIdx], transitionPose[m_allianceIdx]))
                .setLinearHeadingInterpolation(shootPose[m_allianceIdx].getHeading(), transitionPose[m_allianceIdx].getHeading())
                .build();

       PickUpPos = follower.pathBuilder()
               .addPath(new BezierLine(transitionPose[m_allianceIdx],pickupPose[m_allianceIdx]))
               .setLinearHeadingInterpolation(transitionPose[m_allianceIdx].getHeading(), pickupPose[m_allianceIdx].getHeading())
               .build();
       SecondShootPos = follower.pathBuilder()
               .addPath(new BezierLine(pickupPose[m_allianceIdx],secondShootPose[m_allianceIdx]))
               .setLinearHeadingInterpolation(pickupPose[m_allianceIdx].getHeading(), shootPose[m_allianceIdx].getHeading())
               .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // Flywheel + intake logic
                    m_robotech.rtLaunch.launchArtifactAuton();
                    m_robotech.rtIntake.retrieveArtifact();
                    m_robotech.rtIntake.runMidtake(true);

                    telemetry.addLine("Done Path 1 - Shooting preload complete");

                    // Move to next path
                    setPathState(PathState.TRANSITION);
                }
                break;

            case TRANSITION:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>12) {
                    m_robotech.rtLaunch.stop();
//                    m_robotech.rtIntake.stop();
                    m_robotech.rtIntake.runMidtake(false);
                    follower.followPath(TransitionPos,0.8, true);

                    telemetry.addLine("Transitioning to pickup zone");
                    setPathState(PathState.PICKUP);
                }
                break;

            case PICKUP:
                if (!follower.isBusy()) {
                    m_robotech.rtIntake.retrieveArtifact();
                    m_robotech.rtIntake.runMidtake(true);
                    follower.followPath(PickUpPos,0.8, true);

                    telemetry.addLine("Following pickup path");
                    setPathState(PathState.SECOND_SHOOT_MOVE);
                    // Add next state here if you continue the cycle
                    // setPathState(PathState.DRIVE_BACK_TO_SCORE);
                }
                break;
            case SECOND_SHOOT_MOVE:
                if(!follower.isBusy()){
                    m_robotech.rtLaunch.launchArtifactAuton();
                    m_robotech.rtIntake.stop();
                    m_robotech.rtIntake.runMidtake(false);
                    follower.followPath(SecondShootPos,true);
                    telemetry.addLine("Transitioning to shoot");
                    setPathState(PathState.SHOOT_PRELOAD2);
                }
                break;

            case SHOOT_PRELOAD2:
                if (!follower.isBusy()) {
                    // Flywheel + intake logic
                    m_robotech.rtLaunch.launchArtifactAuton();
                    m_robotech.rtIntake.retrieveArtifact();
                    m_robotech.rtIntake.runMidtake(true);

                    telemetry.addLine("Second Time Shooting");

                    // Move to next path
//                    setPathState(PathState.Second_PICKUP);
                }
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

//        shotstriggered = false;
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
        follower.setPose(startPose[m_allianceIdx]);
    }
    public void start(){
        m_robotech.rtLedLight.setColor(RtTypes.rtColor.AZURE);
        m_robotech.rtLaunch.launchArtifactAuton();
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
