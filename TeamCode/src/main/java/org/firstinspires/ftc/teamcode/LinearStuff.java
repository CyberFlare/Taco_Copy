package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.security.KeyStore;

@TeleOp
public class LinearStuff extends OpMode {

    DcMotor Motor0; //chassis
    DcMotor Motor1;
    DcMotor Motor2;
    DcMotor Motor3;
    //BatteryChecker.BatteryStatus Battery1;

    DcMotor Linearout0; //linear pivots out
    DcMotor Linearup1; //linear move up or down
    DcMotor Intake2; //intake
    DcMotor Intake3; //intake

    CRServo Intake4; //for intake up and down
    Servo arm3; //for claw realignment
    Servo Claw; //close and opens stone grab thing
    Servo Foundation; //moves foundation
    Servo Stone; //pushes stone in
    Servo Pivot; //pivots stone
    CRServo Tape; //uses tape measure to part

    DigitalChannel Touch;
    DigitalChannel Toucharm;
    ColorSensor Color1;
    ColorSensor Color2;
    Boolean elevatorCheck = true;



    ElapsedTime Wait = new ElapsedTime();

    public void loop(){


        //if (gamepad1.x) {
          //  Foundation.setPower(0.5);
        //} else Foundation.setPower(0);

       // if (gamepad1.y) {
         //  Foundation.setPower(-0.5);
       //} //else if (Touch.getState() == true){
          //  Foundation.setPower(0);
        //}

        //Motor0.setPower(gamepad1.right_stick_y);
        //Motor1.setPower(gamepad1.right_stick_y);

        //Motor2.setPower(-gamepad1.left_stick_y);
        //Motor3.setPower(-gamepad1.left_stick_y);

       // Motor0.setPower(-gamepad1.left_stick_x); //strafing
        //Motor1.setPower(gamepad1.left_stick_x);
        //Motor2.setPower(-gamepad1.left_stick_x);
        //Motor3.setPower(gamepad1.left_stick_x);


        Motor0.setPower(gamepad1.left_stick_y);
        Motor1.setPower(gamepad1.left_stick_y);
        Motor2.setPower(-gamepad1.left_stick_y);
        Motor3.setPower(-gamepad1.left_stick_y);

        Motor0.setPower(gamepad1.right_stick_x);
        Motor1.setPower(gamepad1.right_stick_x);
        Motor2.setPower(gamepad1.right_stick_x);
        Motor3.setPower(gamepad1.right_stick_x);

        telemetry.update();

        if (gamepad1.right_bumper) { //Straf right
            Motor0.setPower(1);
            Motor1.setPower(-1);
            Motor2.setPower(1);
            Motor3.setPower(-1);
        } //strafe left
        if (gamepad1.left_bumper) { //Straf Left
            Motor0.setPower(-1);
            Motor1.setPower(1);
            Motor2.setPower(-1);
            Motor3.setPower(1);
        } //strafe left

        if (gamepad1.dpad_left) { //Straf Left
            Motor0.setPower(-0.5);
            Motor1.setPower(0.5);
            Motor2.setPower(-0.5);
            Motor3.setPower(0.5);
        } //strafe left
        if (gamepad1.dpad_right) { //Straf Right
            Motor0.setPower(0.5);
            Motor1.setPower(-0.5);
            Motor2.setPower(0.5);
            Motor3.setPower(-0.5);
        } //strafe right
        if (gamepad1.dpad_up) { //Forward
            Motor0.setPower(-1);
            Motor1.setPower(-1);
            Motor2.setPower(1);
            Motor3.setPower(1);
        } //move forward
        if (gamepad1.dpad_down) { //Backward
            Motor0.setPower(1);
            Motor1.setPower(1);
            Motor2.setPower(-1);
            Motor3.setPower(-1);

        } //move backward
        telemetry.update();

        Intake2.setPower(-gamepad1.left_trigger); //intake in & out
        Intake3.setPower(gamepad1.left_trigger);

        Intake2.setPower(gamepad1.right_trigger);
        Intake3.setPower(-gamepad1.right_trigger);


        Intake4.setPower(gamepad2.left_stick_y);

        if (gamepad2.a) {
            Claw.setPosition(0.07);
        }
        if (gamepad2.b) {
            Claw.setPosition(0);
        }




        if (gamepad2.right_bumper && Linearup1.getCurrentPosition() < 750){

            Linearup1.setPower(0.6);

        } else if(gamepad2.left_bumper && Linearup1.getCurrentPosition() > 0) {

            Linearup1.setPower(-0.4);
        }

        else {
            Linearup1.setPower(0);

        }

        if (gamepad2.left_bumper) {
            arm3.setPosition(-0.57);
        } if (gamepad2.right_bumper) {
            arm3.setPosition(0.57);
        }


        if (gamepad2.right_stick_y > 0 && Linearout0.getCurrentPosition() < 5000) {

            Linearout0.setPower(gamepad2.right_stick_y);
            telemetry.addData("Linearout0:", Linearout0.getCurrentPosition());
            telemetry.update();
        }

        else if (gamepad2.right_stick_y < 0 && Linearout0.getCurrentPosition() > 0) {

           Linearout0.setPower(gamepad2.right_stick_y);
            telemetry.addData("Linearout0:", Linearout0.getCurrentPosition());
            telemetry.update();
        }

        else {

            Linearout0.setPower(0);
            telemetry.addData("Linearout0:", Linearout0.getCurrentPosition());
            telemetry.update();
        }
        

        if (gamepad2.x) {
            Pivot.setPosition(-0.5);
        } if (gamepad2.y) {
            Pivot.setPosition(0.5);
        }


        if (gamepad1.a) {
            Stone.setPosition(1);
        } else Stone.setPosition(0);

        if (gamepad1.b) {
            Foundation.setPosition(0.4);
        } else Foundation.setPosition(0);

        Tape.setPower(gamepad2.right_trigger);
        Tape.setPower(-gamepad2.left_trigger);


        //telemetry.addData("Arm Pos.:", Linearout0.getCurrentPosition());
        //telemetry.update();
    }

    public void init() {


        Motor0 = hardwareMap.dcMotor.get ("Motor0"); //Chassis
        Motor1 = hardwareMap.dcMotor.get ("Motor1");
        Motor2 = hardwareMap.dcMotor.get ("Motor2");
        Motor3 = hardwareMap.dcMotor.get ("Motor3");

        Linearout0 = hardwareMap.dcMotor.get ("Linear0"); //Big Boi
        Linearup1 = hardwareMap.dcMotor.get ("Linear1");
        Intake2 = hardwareMap.dcMotor.get ("Intake2");
        Intake3 = hardwareMap.dcMotor.get ("Intake3");

        Intake4 = hardwareMap.crservo.get ("Intakes2"); //big boi
        Stone = hardwareMap.servo.get ("Stone4");
        Pivot = hardwareMap.servo.get("Pivot0");

        Claw = hardwareMap.servo.get ("Claw5"); //chassis
        arm3 = hardwareMap.servo.get ("arm3");
        Foundation = hardwareMap.servo.get ("Foundation2");

        Touch = hardwareMap.get (DigitalChannel.class, "touchboi"); //Chassis
        //Toucharm = hardwareMap.get (DigitalChannel.class, "touchFoundation");
    //Color1 = hardwareMap.colorSensor.get ("bcolor");
        //Color2 = hardwareMap.colorSensor.get ("scolor");

        Linearout0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linearup1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Battery1 = hardwareMap.battery ("g");
    }

}
