/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

//@Autonomous(name= "Left Auto")
public class BlueLeftE extends LinearOpMode
{
    // Declare vars
    private RevTouchSensor rightUpper;
    private RevTouchSensor leftUpper;
    private RevTouchSensor rightLower;
    private RevTouchSensor leftLower;
    private Servo launcher;
    private Servo rightLift;
    private Servo leftLift;
    private Servo shoulder;
    private Servo wrist;
    private Servo hopper;
    //MecanumDrive drive;
    //OgDrive ogDrive;
    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1
    private DcMotor leftBack; //rear left 3
    private DcMotor leftHang;
    private DcMotor rightHang;

    DcMotor frontIntake;
    DcMotor rearIntake;
    @Override
    public void runOpMode() {
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
    }
}