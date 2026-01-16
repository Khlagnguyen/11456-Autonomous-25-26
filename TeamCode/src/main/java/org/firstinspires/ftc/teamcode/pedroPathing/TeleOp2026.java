package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * --- DRIVER HUB CONFIGURATION INSTRUCTIONS ---
 * Use the following exact names (case-sensitive) in your Robot Configuration.
 *
 * MOTORS (Control Hub & Expansion Hub Motor Ports):
 * - "FL"            : Front Left Drive Motor
 * - "FR"            : Front Right Drive Motor  (Also Strafe/Aux Odometry Pod)
 * - "BL"            : Back Left Drive Motor
 * - "BR"            : Back Right Drive Motor   (Also Right Odometry Pod)
 * - "L Launch"      : Left Launcher Motor
 * - "R Launch"      : Right Launcher Motor
 * - "frontLeftOrdo" : Left Odometry Pod (Plugged into a Motor Port, likely no motor attached)
 *
 * SERVOS (Servo Ports):
 * - "L Feed"        : Left Feeder Servo (Continuous Rotation)
 * - "R Feed"        : Right Feeder Servo (Continuous Rotation)
 *
 * SENSORS & CAMERAS:
 * - "Webcam 1"      : USB Camera (Huskylens / GoBilda / Logitech)
 *
 * NOTE ON DIRECTIONS:
 * Leave all devices as "Forward" in the configuration. The code handles reversing.
 * - TeleOp reverses Right motors.
 * - Auto (Constants.java) reverses Left motors by default -> CHECK THIS if Auto drives backwards!
 */

@TeleOp(name = "TeleOp 2026")
public class TeleOp2026 extends LinearOpMode {

    // --- HARDWARE ---
    private CRServo LFeed, RFeed;
    private DcMotorEx LLaunch, RLaunch; // Changed to DcMotorEx for velocity control
    private DcMotor BR, FR, FL, BL;

    // --- CAMERA ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- HOMING CONFIG ---
    // IDs to home onto. Edit this list to match your "Pillar" tags.
    private static final int[] TARGET_IDS = {20, 24};
    
    // Turning Gain: Increase if robot turns too slowly, decrease if it oscillates.
    private static final double HOMING_TURN_GAIN = 0.03; 
    private static final double MAX_AUTO_TURN = 0.5;

    // --- ADAPTIVE LAUNCHER CONFIG ---
    // Tune these values based on your robot's performance!
    // Example: At 10 inches away, the motor needs to spin at 1100 ticks/sec.
    //          At 40 inches away, it needs 1300 ticks/sec.
    private static final double DIST_NEAR = 72; // inches
    private static final double VELO_NEAR = 900; // velocity ticks/sec
    
    private static final double DIST_FAR  = 144.0; // inches
    private static final double VELO_FAR  = 1200; // velocity ticks/sec

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        initHardware();

        // 2. Initialize Camera
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Waiting for Start...");
        telemetry.addData("DS preview on/off", "3 dots -> Camera Stream");
        telemetry.update();

        waitForStart();

        // Variables
        int velocityTarget = 1200;
        boolean useAutoVelocity = false;

        while (opModeIsActive()) {

            // --- 0. SENSORS (Look for Tag) ---
            AprilTagDetection targetTag = getClosestTargetTag();
            
            // --- 1. DRIVER CONTROLS (Gamepad 1) ---
            
            // Raw inputs
            double y = gamepad1.left_stick_y; // Keeping raw Y as per your original code logic
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Slow Mode
            if (gamepad1.left_trigger > 0) {
                y /= 4;
                x /= 4;
                rx /= 4;
            }

            // --- 2. HOMING LOGIC (Gamepad 1 'B') ---
            boolean isHoming = gamepad1.b;
            if (isHoming) {
                if (targetTag != null) {
                    // Turn towards tag
                    double bearing = targetTag.ftcPose.bearing;
                    double autoTurn = -bearing * HOMING_TURN_GAIN;
                    rx = Range.clip(autoTurn, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    telemetry.addData("Homing", "Locked on (ID %d)", targetTag.id);
                } else {
                    rx = 0; // Stop rotation if searching
                    telemetry.addData("Homing", "Searching...");
                }
            }

            // --- 3. MECANUM DRIVE MIXING ---
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frPower = (y + x + rx) / denominator;
            double brPower = (y - x + rx) / denominator;
            double flPower = (y - x - rx) / denominator;
            double blPower = (y + x - rx) / denominator;

            FR.setPower(frPower);
            BR.setPower(brPower);
            FL.setPower(flPower);
            BL.setPower(blPower);

            // --- 4. OPERATOR CONTROLS (Gamepad 2) ---

            // Feeder Logic
            if (gamepad2.left_bumper) {
                LFeed.setPower(1);
                RFeed.setPower(-1);
            } else if (gamepad2.right_bumper) {
                LFeed.setPower(-1);
                RFeed.setPower(1);
            } else {
                LFeed.setPower(0);
                RFeed.setPower(0);
            }

            // Launcher Velocity Selection
            if (gamepad2.x) {
                velocityTarget = 1200;
                useAutoVelocity = false; // Manual override disables auto
            }
            if (gamepad2.y) {
                velocityTarget = 1120;
                useAutoVelocity = false; // Manual override disables auto
            }
            if (gamepad2.a) {
                useAutoVelocity = true; // 'A' enables Auto Velocity
            }

            // Adaptive Velocity Logic
            if (useAutoVelocity) {
                if (targetTag != null) {
                    double range = targetTag.ftcPose.range;
                    velocityTarget = calculateAdaptiveVelocity(range);
                    telemetry.addData("Auto Velo", "Active (Range: %.1f, Velo: %d)", range, velocityTarget);
                } else {
                    telemetry.addData("Auto Velo", "Waiting for Tag...");
                    // Keeps last known valid velocityTarget if no tag seen
                }
            } else {
                telemetry.addData("Auto Velo", "OFF (Press 'A' to Enable)");
            }

            // Launcher Control
            if (gamepad2.right_trigger > 0) {
                LLaunch.setVelocity(velocityTarget);
                RLaunch.setVelocity(-velocityTarget);
            } else {
                LLaunch.setVelocity(0);
                RLaunch.setVelocity(0);
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("Launcher Target", velocityTarget);
            telemetryAprilTag(); // Detailed tag info
            telemetry.update();
        }

        // Cleanup
        visionPortal.close();
    }

    private void initHardware() {
        // Motors
        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        
        LLaunch = hardwareMap.get(DcMotorEx.class, "L Launch");
        RLaunch = hardwareMap.get(DcMotorEx.class, "R Launch");
        
        // Servos
        LFeed = hardwareMap.get(CRServo.class, "L Feed");
        RFeed = hardwareMap.get(CRServo.class, "R Feed");

        // Directions
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);

        // Zero Power Behavior
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1); // Lowest decimation for best range

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Finds the closest AprilTag that matches our target IDs.
     */
    private AprilTagDetection getClosestTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection closestTag = null;
        double minRange = 1000;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && isTargetPillar(detection.id)) {
                if (detection.ftcPose.range < minRange) {
                    minRange = detection.ftcPose.range;
                    closestTag = detection;
                }
            }
        }
        return closestTag;
    }

    /**
     * Linear interpolation to calculate velocity based on range.
     */
    private int calculateAdaptiveVelocity(double range) {
        // y = mx + c formula
        // Slope m = (y2 - y1) / (x2 - x1)
        if(range>120) {
            double slope = (VELO_FAR - VELO_NEAR) / (DIST_FAR - DIST_NEAR);

            // y = m * (x - x1) + y1
            double velocity = slope * (range - DIST_NEAR) + VELO_NEAR;

            return (int) Range.clip(velocity, 0, 3000); // Safety clip
        } else{
            return(1100);
        }
    }

    private boolean isTargetPillar(int id) {
        for (int target : TARGET_IDS) {
            if (id == target) return true;
        }
        return false;
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            }
        }
    }
}

/**
 * =================================================================================================
 *                                   CAMERA LOGIC EXPLAINED
 * =================================================================================================
 *
 * 1. HOW IT WORKS:
 *    - The robot uses a USB Camera plugged into the Control Hub.
 *    - The "VisionPortal" is the software manager that talks to the camera.
 *    - The "AprilTagProcessor" is a piece of software that analyzes the image frame-by-frame.
 *    - When it sees a special QR-code-like pattern (AprilTag), it calculates where that tag is
 *      relative to the camera lens.
 *
 * 2. COORDINATE SYSTEMS (XYZ vs RBE):
 *    The camera gives us two ways to understand where the tag is:
 *
 *    A) XYZ (Translation / Cartesian Coordinates)
 *       - X: Horizontal distance. Positive = RIGHT of the camera. Negative = LEFT.
 *       - Y: Forward distance. Positive = FORWARD (away from camera).
 *       - Z: Vertical distance. Positive = UP. Negative = DOWN.
 *       Useful for: "Strafe X inches to the right to line up."
 *
 *    B) RBE (Spherical Coordinates) - *USED FOR HOMING*
 *       - R (Range): Direct straight-line distance to the tag (in inches).
 *         > Used for ADAPTIVE LAUNCHER VELOCITY. The farther away, the harder we shoot.
 *
 *       - B (Bearing): The angle (in degrees) the robot needs to turn to face the tag.
 *         > Negative Bearing = Target is to the RIGHT.
 *         > Positive Bearing = Target is to the LEFT.
 *         > Used for HOMING. If Bearing is +10 deg, we turn Left to make it 0.
 *
 *       - E (Elevation): The vertical angle (looking up or down) to the tag.
 *
 * 3. PRY (Rotation / Orientation):
 *    - Pitch: Tilting up/down.
 *    - Roll: Tilting head left/right.
 *    - Yaw: Turning head left/right.
 *    Useful for: Knowing if the robot is approaching the tag straight-on or at an angle.
 *
 * 4. DECIMATION:
 *    - We set "aprilTag.setDecimation(1)".
 *    - Decimation is "downsampling". High decimation (3) = Fast but low resolution (short range).
 *    - Low decimation (1) = Full resolution processing. Slower fps, but detects tags FAR away.
 *
 * 5. ADAPTIVE VELOCITY MATH:
 *    - We use a Linear Interpolation (y = mx + b).
 *    - We define two known points:
 *         Point 1: Near (10 inches away -> 1100 velocity)
 *         Point 2: Far  (40 inches away -> 1300 velocity)
 *    - If the robot is at 25 inches (middle), the code calculates the velocity halfway between (1200).
 *    - This allows precise shooting from anywhere on the field!
 */
