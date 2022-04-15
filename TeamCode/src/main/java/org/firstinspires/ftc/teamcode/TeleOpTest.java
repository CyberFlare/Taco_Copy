package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpTest  extends OpMode {

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

    DigitalChannel Touch;
    ColorSensor Color1;
    ColorSensor Color2;
    Boolean elevatorCheck = true;

    public void loop(){

        

    }

    public void init(){

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
        //Color1 = hardwareMap.colorSensor.get ("bcolor");
        //Color2 = hardwareMap.colorSensor.get ("scolor");

        Linearout0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linearup1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}
