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
import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="TankTaco: Teleop Tank", group="Pushbot")
public class PushbotTeleopTank_IterativeA extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot_A robot = new HardwarePushbot_A(); // use the class created to define a Pushbot's hardware
    double          armPosi = robot.ARM_HOME;
    double          armPos = robot.ARM_HOME;
    double       armPosition = robot.ARM_HOME;
    double       armPositi = robot.ARM_HOME;
    double       armPosii = robot.ARM_HOME;
    double          armPosit  = robot.ARM_HOME;                  // Servo mid position
    final double    ARM_SPEED  = 0.75 ;                 // sets rate to move servo

    public double deadBand(double power){

        if(power<0.1 || power>-0.1){
            return 0.0;
        }
        return power;
    }

    /*
     * Code to run ONCE when thse driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    int Harris;
    @Override
    public void loop() {

        double in;
        double out;


        //Set power of intake to make it move
        in   = -gamepad2.left_stick_x;
        out  =  gamepad2.left_stick_x;
       //Driving
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.leftDrive.setPower(v1);
        robot.rightDrive.setPower(v2);
        robot.sleftDrive.setPower(-v3);
        robot.srightDrive.setPower(v4);

        if (gamepad1.dpad_up) {
            robot.leftDrive.setPower(-0.75);
            robot.rightDrive.setPower(-0.75);
            robot.sleftDrive.setPower(0.75);
            robot.srightDrive.setPower(0.75);
        }
        else if (gamepad1.dpad_down) {
            robot.leftDrive.setPower(0.75);
            robot.rightDrive.setPower(0.75);
            robot.sleftDrive.setPower(-0.75);
            robot.srightDrive.setPower(-0.75);
        }
        else if (gamepad1.dpad_right) {
            robot.leftDrive.setPower(0.75);
            robot.rightDrive.setPower(-0.75);
            robot.sleftDrive.setPower(0.75);
            robot.srightDrive.setPower(-0.75);
        }
        else if (gamepad1.dpad_left) {
            robot.leftDrive.setPower(-0.75);
            robot.rightDrive.setPower(0.75);
            robot.sleftDrive.setPower(-0.75);
            robot.srightDrive.setPower(0.75);
        }
        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.sleftDrive.setPower(0);
            robot.srightDrive.setPower(0);
        }

        // Use Y & A to move the intake system  up and down

         if (gamepad1.y) {
             robot.leftServ.setPower(0.75);
         }
         else if (gamepad1.a) {
             robot.leftServ.setPower(-0.75);
         }
         else {
             robot.leftServ.setPower(0);
         }

         //Used to move the stick to hit the block up the ramp in case it does not make it up
        if (gamepad1.right_bumper) {
            armPos += ARM_SPEED;
        }
        else if (gamepad1.left_bumper) {
            armPos -= -ARM_SPEED;
        }
        else {
            armPos = 0.0;
        }

        //used to move the claw up and down
        if (gamepad2.dpad_up) {
            armPosi += ARM_SPEED;
        }
        else if (gamepad2.dpad_down) {
            armPosi -= ARM_SPEED;
        }

        if (gamepad2.dpad_right) {
            armPosi += ARM_SPEED;
            telemetry.addLine("before");
        }
        else if (gamepad2.dpad_left) {
            armPosi -= ARM_SPEED;
        }
        //Used to close and open the claw
        if (gamepad2.right_bumper){
            armPositi += ARM_SPEED;
        }
        else if (gamepad2.left_bumper) {
            armPositi = 0.30;
            telemetry.addLine("up to");
        }

        //Used to move the servo to latch onto the foundation.
        if (gamepad1.x) {
            armPosit += -ARM_SPEED;
        }
        else {
            armPosit = 0.0;
        }

        if (gamepad1.a) {
            armPosii += ARM_SPEED;
        }
        else {
            armPosii = 0.0;
        }

        //other servo stuff
        robot.SP2.setPosition(armPos);
        robot.SP1.setPosition(armPosii);
        //robot.SP4.setPosition(armPosi);
        //robot.leftServ.setPosition(armPosition);
        robot.Foundation.setPosition(armPosit);
        robot.SP5.setPosition(armPositi);

        armPos = Range.clip(armPos, 0.0, 0.5);
        armPosii = Range.clip(armPosii, -0.5, 0.5);
        armPosi = Range.clip(armPosi, -0.5, 0.5);
        armPosition = Range.clip(armPosition, -0.5, 0.5);
        armPosit = Range.clip(armPosit, -0.5, 0.5);
        armPositi = Range.clip(armPositi, -0.5, 0.5);




        // Use gamepad buttons to turn on & off intake system

        robot.leftSucc.setPower(in);
        robot.rightSucc.setPower(out);

        //Use Y & A to move arm up and down
        /*boolean goingToGreaterLevel = true;
        if (gamepad2.y) {
            Harris = 1;
            robot.liftArm.setPower(-0.70);
            goingToGreaterLevel = true;
        } else if (gamepad2.a) {
            Harris = 0;
            robot.liftArm.setPower(0.70);
            goingToGreaterLevel = false;
        }
        int valueHarris;
        if (Harris == 0) {
            valueHarris = 0;
        }
        else if(Harris ==1) {
            valueHarris = -2789;
        }
        else {
            valueHarris = Harris*2689;

        }

        /*double position = 0;

        if (gamepad2.right_stick_y > 0) {
            robot.liftArm.setPower(1);
            telemetry.addData("", robot.liftArm.getCurrentPosition());
            telemetry.update();

        } else if (gamepad2.right_stick_y == 0) {
            robot.liftArm.setPower(0.05);
        } else {
            telemetry.addData("", robot.liftArm.getCurrentPosition());
            telemetry.update();
            robot.liftArm.setPower(-1);
        }

        robot.liftArm.setTargetPosition(valueHarris);       // Send telemetry message to signify robot running;
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(goingToGreaterLevel) {
            robot.liftArm.setPower(-0.70);
        } else if (goingToGreaterLevel == false) { robot.liftArm.setPower(0.70); //set back to 0.70 when done testing
       }*/

        if (gamepad2.y) {
            robot.liftArm.setPower(1);
        }
        else if (gamepad2.a) {
            robot.liftArm.setPower(-1);
        }
        else {
            robot.liftArm.setPower(0);
        }

        if (robot.liftArm.getCurrentPosition() >= 4987) {
            robot.liftArm.setPower(-0.75);
        }

        //Moves the arms sideways
        if (gamepad2.x) {
            robot.lifty.setPower(0.35);
        }
        else if (gamepad2.b) {
            robot.lifty.setPower(-0.35);
        }
        else {
            robot.lifty.setPower(0);
        }
        telemetry.update();
        telemetry.addData("in", "%.2f", in);
        telemetry.addData("out", "&.2f", out);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
