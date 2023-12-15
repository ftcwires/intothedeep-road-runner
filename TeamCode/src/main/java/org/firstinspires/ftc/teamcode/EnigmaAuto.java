/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//do he auto and get a 2+2
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevTouchSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Locale;
import java.util.List;

/**
 * ENIGMA Autonomous Example for only vision detection using openCv and park
 */
@Autonomous(name = "ENIGMA Autonomous Mode", group = "00-Autonomous", preselectTeleOp = "Mutation")
public class EnigmaAuto extends LinearOpMode {

    public static String TEAM_NAME = "ENIGMA"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 16265; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

    private RevTouchSensor rightUpper;
    private RevTouchSensor leftUpper;
    private RevTouchSensor rightLower;
    private RevTouchSensor leftLower;
    private Servo rightLift;
    private Servo leftLift;
    private Servo shoulder;
    private Servo wrist;
    private Servo elbow;
    private Servo leftFinger;
    private Servo rightFinger;

    double LiftLeftOffset = .04;
    double LiftHeight;

    private static final double ELBOW_DRIVE= Mutation.ELBOW_DRIVE;
    private static final double ELBOW_INTAKE = Mutation.ELBOW_INTAKE;
    private static final double WRIST_INTAKE = Mutation.WRIST_INTAKE;
    //private static final double SHOULDER_DRIVE = 0.425; // 0.425
    private static final double SCORE_ONE_SHOULDER = Mutation.SCORE_ONE_SHOULDER;
    private static final double SCORE_ONE_WRIST = Mutation.SCORE_ONE_WRIST;
    private static final double SCORE_ONE_ELBOW = Mutation.SCORE_ONE_ELBOW;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    /*
    OpenCV / Color Detection
     */
    OpenCvCamera webcam1 = null;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
    public static double leftavgfinoutput = 0;
    public static double centeravgfinoutput = 0;
    public static double rightavgfinoutput = 0;

    private double servoposition = 0.0;
    private double servodelta = 0.02;
    private double servodelaytime = 0.03;

    private void moveServoGradually(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();

        // Check if enough time has passed since the last update
        if (servoTimer.seconds() > servodelaytime) {
            // Determine the direction of movement
            double direction = targetPosition > currentPosition ? servodelta : -servodelta;

            // Calculate the new position
            servoposition = currentPosition + direction;
            servoposition = Range.clip(servoposition, 0, 1); // Ensure the position is within valid range

            // Update the servo position
            servo.setPosition(servoposition);

            // Reset the timer
            servoTimer.reset();
        }
    }

    private Servo setLiftPosition(double targetPosition) {
        // Ensure the target position is within the valid range
        targetPosition = Math.max(0.0, Math.min(targetPosition, 1.0));

        // Set the servo positions
        leftLift.setPosition(targetPosition);
        rightLift.setPosition(targetPosition);
        return null;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        wrist = hardwareMap.get(Servo.class, "wrist");

        shoulder = hardwareMap.get(Servo.class, "shoulder");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        elbow = hardwareMap.get(Servo.class, "elbow");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);


        //init pos
        setLiftPosition(Mutation.LIFT_DRIVE);
        shoulder.setPosition(Mutation.SHOULDER_DRIVE);
        wrist.setPosition(Mutation.WRIST_INTAKE);
        elbow.setPosition(Mutation.ELBOW_DRIVE);
        sleep(2500);
        leftFinger.setPosition(Mutation.LEFT_FINGER_GRIP);
        rightFinger.setPosition(Mutation.RIGHT_FINGER_GRIP);

        // Vision OpenCV / Color Detection
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new teamElementPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.setMsTransmissionInterval(50);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses TensorFlow for pixel detection
        //initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
           // runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.addData("leftavfin", leftavgfinoutput);
            telemetry.addData("centeravfin", centeravgfinoutput);
            telemetry.addData("rightavfin", rightavgfinoutput);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()
    private void setLiftHeight(double inputLiftHeight) {
        if (inputLiftHeight < 0.42) {
            inputLiftHeight = 0.42;
        }
        if (inputLiftHeight > 1) {
            inputLiftHeight = 1;
        }
        LiftHeight = inputLiftHeight;
        leftLift.setPosition(LiftLeftOffset + LiftHeight);
        rightLift.setPosition(LiftHeight);
    }
    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPosePush = new Pose2d(0, 0, 0);
        //Pose2d dropPurplePixelPosePos = new Pose2d(0, 0, 0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        //TODO: edit crap here
        switch (startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPosePush = new Pose2d(27, 8, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(19, 36, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPosePush = new Pose2d(32, 0, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(26, 36,  Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPosePush = new Pose2d(27, -9, Math.toRadians(-45));
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(31, 36, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPosePush = new Pose2d(27, 8, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(34, -36, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPosePush = new Pose2d(32, 0, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(27, -36,  Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPosePush = new Pose2d(27, -9, Math.toRadians(-45));
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(20, -36, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPosePush = new Pose2d(27, 8, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(19, 86, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPosePush = new Pose2d(32, 0, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(26.25, 86, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPosePush = new Pose2d(27, -9, Math.toRadians(-45));
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(33.5, 86, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, -21, Math.toRadians(-90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPosePush = new Pose2d(27, 8, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(34, -86, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPosePush = new Pose2d(32, 0, Math.toRadians(0)); // change up
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -86, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPosePush = new Pose2d(27, -9, Math.toRadians(-45));
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(23, -86, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 21, Math.toRadians(90));
                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, -84, Math.toRadians(90));
                break;
        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPosePush.position, dropPurplePixelPosePush.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        //.strafeToLinearHeading(dropPurplePixelPosePos.position, dropPurplePixelPosePos.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(1);
        shoulder.setPosition(Mutation.SHOULDER_DRIVE);
        wrist.setPosition(Mutation.WRIST_INTAKE);
        for(int c = 0; c<40; c++) {
            moveServoGradually(elbow, ELBOW_INTAKE);
            sleep(10);
        }
        sleep(200);
        rightFinger.setPosition(Mutation.RIGHT_FINGER_DROP);
        sleep(200);
        elbow.setPosition(ELBOW_DRIVE);
        wrist.setPosition(Mutation.WRIST_TUCK);

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
                startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(1);

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
        }

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);
        for(int w = 0; w<40; w++) {
            moveServoGradually(wrist, SCORE_ONE_WRIST);
            sleep(10);
        }
        for(int e = 0; e<20; e++) {
            moveServoGradually(elbow, SCORE_ONE_ELBOW);
            sleep(10);
        }
        for(int s = 0; s<200; s++) {
            moveServoGradually(shoulder, SCORE_ONE_SHOULDER);
            sleep(7);
        }
        sleep(200);
        leftFinger.setPosition(Mutation.LEFT_FINGER_DROP);
        sleep(100);
        for(int s = 0; s<200; s++) {
            moveServoGradually(shoulder, Mutation.SHOULDER_DRIVE);
            sleep(7);
        }
        for(int w = 0; w<40; w++) {
            moveServoGradually(wrist, WRIST_INTAKE);
            sleep(10);
        }
        for(int e = 0; e<20; e++) {
            moveServoGradually(elbow, ELBOW_DRIVE);
            sleep(10);
        }



        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing ENIGMA Autonomous Team# a",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
class teamElementPipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat centerCrop;
        Mat rightCrop;
        double leftavgfin;
        double centeravgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");
            /* OG Config
            Rect leftRect = new Rect(1,1,212, 479);
            Rect centerRect = new Rect(213,1,212, 479);
            Rect rightRect = new Rect(426,1,213, 479);

            Rect leftRect = new Rect(1,300,200, 400);
            Rect centerRect = new Rect(400,300,500, 200);
            Rect rightRect = new Rect(1000,300,260, 400);
*/
            // testing tighter frames

            Rect leftRect = new Rect(1,380,125, 200);
            Rect centerRect = new Rect(500,300,200, 200);
            Rect rightRect = new Rect(1100,380,150, 200);




            input.copyTo(outPut);


            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            centerCrop = YCbCr.submat(centerRect);
            rightCrop = YCbCr.submat(rightRect);

            if (startPosition == START_POSITION.BLUE_LEFT || startPosition == START_POSITION.BLUE_RIGHT) {
                Core.extractChannel(leftCrop, leftCrop, 0);
                Core.extractChannel(centerCrop, centerCrop, 0);
                Core.extractChannel(rightCrop, rightCrop, 0);  // blue is 0, green is 1, red is 2
            } else if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.RED_RIGHT) {
                Core.extractChannel(leftCrop, leftCrop, 2);
                Core.extractChannel(centerCrop, centerCrop, 2);
                Core.extractChannel(rightCrop, rightCrop, 2);  // blue is 0, green is 1, red is 2
            } else {
                Core.extractChannel(leftCrop, leftCrop, 0);
                Core.extractChannel(centerCrop, centerCrop, 0);
                Core.extractChannel(rightCrop, rightCrop, 0);  // blue is 0, green is 1, red is 2
            }

            Scalar leftavg = Core.mean(leftCrop);
            Scalar centeravg = Core.mean(centerCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            centeravgfin = centeravg.val[0];
            rightavgfin = rightavg.val[0];

            leftavgfinoutput = leftavgfin;
            centeravgfinoutput = centeravgfin;
            rightavgfinoutput = rightavgfin;

            if (leftavgfin < centeravgfin && leftavgfin < rightavgfin) {
                //telemetry.addLine("Left");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            } else if (centeravgfin < leftavgfin && centeravgfin < rightavgfin) {
                //telemetry.addLine("Center");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            } else if (rightavgfin < leftavgfin && rightavgfin < centeravgfin) {
                //telemetry.addLine("Right");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            } else {
                //telemetry.addLine("Failed to detect position");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            }
            return (outPut);
        }
}
}   // end class
