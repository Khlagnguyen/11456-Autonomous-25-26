package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "XavionTest")
public class XavionTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private CRServo LFeed;
    private CRServo RFeed;
    private DcMotor LLaunch;
    private DcMotor RLaunch;
    private DcMotor BR;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BL;

    private int pathState;
    private java.util.List<Pose> allPoses;

    private Pose startPose, endPose;

    private Path movement;
    private PathChain goToFinish1, goToFinish2, goToFinish3, grabPickup1, grabPickup2, grabPickup3;

    private int blueOrRedX = 0;
    private int blueOrRedHeading = 0;

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        LFeed = hardwareMap.get(CRServo.class, "L Feed");
        RFeed = hardwareMap.get(CRServo.class, "R Feed");
        LLaunch = hardwareMap.get(DcMotor.class, "L Launch");
        RLaunch = hardwareMap.get(DcMotor.class, "R Launch");

        LLaunch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RLaunch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {

        startPose = new Pose(Math.abs(blueOrRedX-72), 9, Math.toRadians(Math.abs(blueOrRedHeading-90)));
        endPose = new Pose(Math.abs(blueOrRedX-72), 72, Math.toRadians(Math.abs(blueOrRedHeading-0)));


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
    }


    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        movement = new Path(new BezierLine(startPose, endPose));
//        movement.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, new Pose(Math.abs(blueOrRedX-80), 40, Math.toRadians(Math.abs(blueOrRedHeading-0))), pickup1Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .build();



    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


    }
    public void setPowers(double power) {
        ((DcMotorEx) LLaunch).setVelocity(1200 * power);
        ((DcMotorEx) RLaunch).setVelocity(-1200 * power);
//        LLaunch.setPower(0.56 * power);
//        RLaunch.setPower(-0.56 * power);
        LFeed.setPower(-1 * power);
        RFeed.setPower(1 * power);
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPowers(1.0);
                follower.followPath(movement);
                setPathState(1);
                break;

        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
