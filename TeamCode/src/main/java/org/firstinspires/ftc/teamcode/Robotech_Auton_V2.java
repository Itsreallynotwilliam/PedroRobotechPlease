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
    
//    private boolean shotstriggered = false;

    public enum PathState{
        // START POSITION_ENDPOSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        TRANSITION,
        PICKUP
    }

    PathState pathState;

    private final Pose startPose = new Pose(24,126,Math.toRadians(-37));
    private final Pose shootPose = new Pose(52,115,Math.toRadians(-31));
    private final Pose transitionPose = new Pose(52,84,Math.toRadians(180));
    private final Pose pickupPose = new Pose(13,84,Math.toRadians(180));



    private PathChain driveStartPosShootPos,TransitionPos,PickUpPos;


    public void buildpaths(){
        //put ini coordinates for starting pos > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        TransitionPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, transitionPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), transitionPose.getHeading())
                .build();
       PickUpPos = follower.pathBuilder()
               .addPath(new BezierLine(transitionPose,pickupPose))
               .setLinearHeadingInterpolation(transitionPose.getHeading(), pickupPose.getHeading())
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
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>14) {

                    m_robotech.rtIntake.stop();
                    m_robotech.rtIntake.runMidtake(false);
                    follower.followPath(TransitionPos,0.8, true);

                    telemetry.addLine("Transitioning to pickup zone");
                    setPathState(PathState.PICKUP);
                }
                break;

            case PICKUP:
                if (!follower.isBusy()) {
                    m_robotech.rtIntake.retrieveArtifact();
                    m_robotech.rtIntake.runMidtake(false);
                    follower.followPath(PickUpPos,08, true);

                    telemetry.addLine("Following pickup path");
                    // Add next state here if you continue the cycle
                    // setPathState(PathState.DRIVE_BACK_TO_SCORE);
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
        follower.setPose(startPose);
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
