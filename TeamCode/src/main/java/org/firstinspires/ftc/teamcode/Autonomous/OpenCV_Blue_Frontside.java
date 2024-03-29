package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "OpenCV_BlueAutonomous_FrontSideTBC")

public class OpenCV_Blue_Frontside extends LinearOpMode {
    private DcMotor rightDrive;
    private DcMotor leftDrive;
    private DcMotor backDrive;
    private DcMotor skyLift;
    private CRServo droneLaunch;

    private CRServo pixelWheel;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 552;  // Replace with the focal length of the camera in pixels


    @Override
    public void runOpMode() {

        double vertical;
        double horizontal;
        double pivot;
        double rightMotor;
        double leftMotor;
        double backMotor;
        double lift;

        //change drive train for 3 wheel drive.
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        backDrive = hardwareMap.get(DcMotor.class, "backDrive");
        skyLift = hardwareMap.get(DcMotor.class, "skyLift");
        droneLaunch = hardwareMap.get(CRServo.class, "droneLaunch");
        pixelWheel = hardwareMap.get(CRServo.class, "pixelWheel");

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        skyLift.setDirection(DcMotor.Direction.REVERSE);
        droneLaunch.setDirection(CRServo.Direction.FORWARD);

        if (opModeIsActive()) {


            int reference = 3;
            skyLift.setPower(0.5);
            sleep(500);
            pixelWheel.setPower(-0.5);
            sleep(800);
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            backDrive.setPower(0);
            //frontright.setPower(0);
            // To be change, Needs calibration for Kiwi
            if (cY > 267 && cY < 288){
                if (cX > 50 && cX < 139){
                    telemetry.addData("Left","(" + (int) cX + ", " + (int) cY + ")" );
                    //Red  spike mark is Left
                    reference = 0;
                }
            }
            if (cY > 245 && cY < 264){
                if (cX > 177 && cX < 437){
                    telemetry.addData("Middle","(" + (int) cX + ", " + (int) cY + ")" );
                    // Red spike mark is middle
                    reference = 1;
                }
            }
            if (cY > 247 && cY < 99) {
                if (cX > 495 && cX < 631) {
                    telemetry.addData("Right", "(" + (int) cX + ", " + (int) cY + ")");
                    //Red spike mark is Right
                   reference = 2;
                }
            }
            //Values are based on Botlobster, please CAREFULLY change setpower and sleep till you get intended result.
            //Randomized towards the left
            // First Bot will turn 360 to set

            // if spike is left (done)
            if (reference == 0) {
                //Move towards spike mark.
                moveY(0.5,800);
                //Turn 90 degrees counterclockwise to face spike mark.
                turn(-0.5,875);
                //whip out claw
                AutoskyLift(0.5, 250);
                //Open claw, release pixels.
                pixelWheel.setPower(-0.5);
                sleep(800);
                //claw goes back into position
                AutoskyLift(-0.5, 250);
                //strafe right
                moveX(-0.5, 800);
                //Move forward to park.
                moveY(0.5, 800);

            }
            //Randomized towards the middle spike (done)
            if (reference == 1) {
                //Move towards spike
                moveY(0.5, 1000);



                //whip out Claw arm out
                AutoskyLift(0.5, 250);

                //open claw to release pixels (20 Points)
                pixelWheel.setPower(-0.5);
                sleep(800);

                //claw goes back in  position.
                AutoskyLift(-0.5, 250);

                // Move Backwards, Park
                moveY(-0.5,700);

                //strafe left
                moveX(0.5, 800);

                /* //backDrive.setPower(-0.5);
               // backDrive.setPower(0.5);
                sleep(1190);
                rightDrive.setPower(0.5);
                leftDrive.setPower(0.5);
                backDrive.setPower(0.5);
                //frontright.setPower(0.5);
                sleep(2500); */

            }
            //Randomized towards the right
            if (reference == 2) {
                //Move towards spike mark.
                moveY(0.5,800);
                //Turn 90 degrees clockwise to face spike mark.
                turn(0.5,875);
                //whip out claw
                AutoskyLift(0.5, 250);
                //Open claw, release pixels.
                pixelWheel.setPower(-0.5);
                sleep(800);
                //claw goes back into position
                AutoskyLift(-0.5, 250);
                //strafe right
                moveX(0.5, 800);
                //Move backwards to park or back? (claw miight be in the way)
                moveY(-0.5, 3000);

            }
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            //What color your detecting.
            Scalar lowerYellow = new Scalar(100, 50, 50);
            Scalar upperYellow = new Scalar(140, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    // This function will have the robot move forward for specified power for specified milliseconds
    // @arg power - direction and power for the robot. negative power will move the robot backwards
    // @arg waitTime - the length of time to move
    private void moveY(double power, int waitTime) {
        rightDrive.setPower(power * Math.sqrt(3)/2 );
        leftDrive.setPower(power * Math.sqrt(3)/2 );
        sleep(waitTime);
        zeroMotors();
    }
    // This function will have the robot strafeat specified power for specified milliseconds
    private void moveX(double power, int waitTime) {
        rightDrive.setPower (-0.5 * power);
        leftDrive.setPower (0.5 * power);
        backDrive.setPower (0.8 * power);
        sleep(waitTime);
        zeroMotors();
    }

    // This function will move the robot at specified power for specified milliseconds
    private void turn(double power, int waitTime) {
        rightDrive.setPower (-power);
        leftDrive.setPower (power);
        backDrive.setPower (-power);
        sleep(waitTime);
        zeroMotors();

    }

    private void AutoskyLift(double power, double waitTime){
        skyLift.setPower (power);
        zeroMotors();
    }
    // This turns of all moving motors
    private void zeroMotors() {
        rightDrive.setPower (0);
        leftDrive.setPower (0);
        backDrive.setPower (0);


        }
    }
