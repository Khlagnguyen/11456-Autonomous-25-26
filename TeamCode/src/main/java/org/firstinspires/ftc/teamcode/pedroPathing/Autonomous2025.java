package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Autonomous2025")
public class Autonomous2025 extends OpMode {

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

    private Pose startPose, scorePose, pickup1Pose, pickup2Pose, pickup3Pose, finishPose;

    private Path scorePreload;
    private PathChain goToFinish1, goToFinish2, goToFinish3, grabPickup1, grabPickup2, grabPickup3;

    private int blueOrRedX = 0;
    private int blueOrRedHeading = 0;
    private boolean alliance = false;
    private String blueOrRed = "blue";

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        if (!alliance) {
            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, new Pose(Math.abs(blueOrRedX - 70), 40, Math.toRadians(Math.abs(blueOrRedHeading - 0))), pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();

        } else {
            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, new Pose(Math.abs(blueOrRedX - 80), 40, Math.toRadians(Math.abs(blueOrRedHeading - 0))), pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();
        }
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(finishPose, new Pose(Math.abs(blueOrRedX - 70), 72, Math.toRadians(Math.abs(blueOrRedHeading - 0))), pickup2Pose))
                .setLinearHeadingInterpolation(finishPose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(finishPose, new Pose(Math.abs(blueOrRedX - 70), 98, Math.toRadians(Math.abs(blueOrRedHeading - 0))), pickup3Pose))
                .setLinearHeadingInterpolation(finishPose.getHeading(), pickup3Pose.getHeading())
                .build();

        goToFinish1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, finishPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), finishPose.getHeading())
                .build();

        goToFinish2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, finishPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), finishPose.getHeading())
                .build();
        goToFinish3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, finishPose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), finishPose.getHeading())
                .build();


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


    }

    public void setPowers(double power) {
        if (alliance) {
            ((DcMotorEx) LLaunch).setVelocity(1100 * power);
            ((DcMotorEx) RLaunch).setVelocity(-1100 * power);
        } else {
            ((DcMotorEx) LLaunch).setVelocity(1200 * power);
            ((DcMotorEx) RLaunch).setVelocity(-1200 * power);
        }
        LFeed.setPower(-1 * power);
        RFeed.setPower(1 * power);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!alliance) {
                    setPowers(1.0);
                }
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (alliance) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        setPowers(1.0);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 8) {
                        setPowers(0.0);
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 6) {
                        setPowers(0.0);
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(goToFinish1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(goToFinish2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(goToFinish3, true);
                    setPathState(-1);
                }
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

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {


        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

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
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            if (blueOrRed.equals("red")) {
                blueOrRed = "blue";
                blueOrRedX = 0;
                blueOrRedHeading = 0;
            } else if (blueOrRed.equals("blue")) {
                blueOrRed = "red";
                blueOrRedX = 144;
                blueOrRedHeading = 180;

            }
        }

        if (gamepad1.right_bumper) {
            alliance = !alliance;
        }

        telemetry.addData("Alliance (press right bumper)", alliance);
        telemetry.addData("Current Selection (press left bumper)", blueOrRed);
        telemetry.addData("Current X offset", blueOrRedX);
        telemetry.addData("Current heading offset", blueOrRedHeading);
        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        if (alliance) {
            startPose = new Pose(Math.abs(blueOrRedX - 23), 124, Math.toRadians(Math.abs(blueOrRedHeading - 315)));
            scorePose = new Pose(Math.abs(blueOrRedX - 110), 101, Math.toRadians(Math.abs(blueOrRedHeading - 158))); // Scoring Pose of our robot. It is facing the goal at a 158 degree angle.
        } else {
            startPose = new Pose(Math.abs(blueOrRedX - 53), 9, Math.toRadians(Math.abs(blueOrRedHeading - 90)));
            scorePose = new Pose(Math.abs(blueOrRedX - 89), 10, Math.toRadians(Math.abs(blueOrRedHeading - 120))); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        }
        pickup1Pose = new Pose(Math.abs(blueOrRedX - 120), 48, Math.toRadians(Math.abs(blueOrRedHeading - 0))); // Lowest (Third Set) of Artifacts from the Spike Mark.

        pickup2Pose = new Pose(Math.abs(blueOrRedX - 120), 72, Math.toRadians(Math.abs(blueOrRedHeading - 0))); // Middle (Second Set) of Artifacts from the Spike Mark.
        pickup3Pose = new Pose(Math.abs(blueOrRedX - 120), 96, Math.toRadians(Math.abs(blueOrRedHeading - 0))); // Highest (First Set) of Artifacts from the Spike Mark.
        finishPose = new Pose(Math.abs(blueOrRedX - 125), 14, Math.toRadians(Math.abs(blueOrRedHeading - 0)));


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        if (LLaunch != null) LLaunch.setPower(0.0);
        if (RLaunch != null) RLaunch.setPower(0.0);
        if (LFeed != null) LFeed.setPower(0.0);
        if (RFeed != null) RFeed.setPower(0.0);
    }
}