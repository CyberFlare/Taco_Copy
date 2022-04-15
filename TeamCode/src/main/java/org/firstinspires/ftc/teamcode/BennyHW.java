/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Motor channel:  Manipulator drive motot:  "right_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class BennyHW {
    DcMotor rightDrive; //chassis
    DcMotor leftDrive;
    DcMotor srightDrive;
    DcMotor sleftDrive;
    //BatteryChecker.BatteryStatus Battery1;

    DcMotor Linearout0; //linear pivots out
    DcMotor Linearup1; //linear move up or down
    DcMotor Intake2; //intake
    DcMotor Intake3; //intake

    CRServo Intake4; //for intake up and down
    Servo arm3; //for claw realignment
    Servo Claw; //close and opens stone grab thing
    Servo foundation; //moves foundation
    Servo Stone; //pushes stone in
    Servo Pivot; //pivots stone
    CRServo rightArm;
    Servo leftArm;

    DigitalChannel Touch;
    DigitalChannel Toucharm;
    ColorSensor Color1;
    ColorSensor Color2;
    Boolean elevatorCheck = true;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to HaHardwareMap ahwMaprdware map
        hwMap = ahwMap;
        rightDrive = hwMap.get(DcMotor.class, "Motor0");
        leftDrive = hwMap.get(DcMotor.class, "Motor1");
        srightDrive = hwMap.get(DcMotor.class, "Motor2");
        sleftDrive = hwMap.get(DcMotor.class, "Motor3");

        Linearout0 = hwMap.get(DcMotor.class, "Linear0"); //Big Boi
        Linearup1 = hwMap.get(DcMotor.class, "Linear1");
        Intake2 = hwMap.get(DcMotor.class, "Intake2");
        Intake3 = hwMap.get(DcMotor.class, "Intake3");

        Intake4 = hwMap.get(CRServo.class, "Intakes2"); //big boi
        Stone = hwMap.get(Servo.class, "Stone4");
        Pivot = hwMap.get(Servo.class, "Pivot0");

        Claw = hwMap.get(Servo.class, "Claw5");
        arm3 = hwMap.get(Servo.class, "arm3");
        foundation = hwMap.get(Servo.class, "Foundation2");
        leftArm = hwMap.get(Servo.class, "leftArm");
        rightArm = hwMap.get(CRServo.class, "rightArm");

        Touch = hwMap.get(DigitalChannel.class, "touchboi"); //Chassis
        //Toucharm = hardwareMap.get (DigitalChannel.class, "touchFoundation");
        Color1 = hwMap.colorSensor.get ("bcolor");
        Color2 = hwMap.colorSensor.get ("scolor");

        Linearout0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linearup1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}