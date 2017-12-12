package org.firstinspires.ftc.teamcode.RelicRecovery.Autonomous.Auto_40;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by Jorge on 23/11/2017.
 */
@Autonomous(name="Blue_Back_40")
public class Blue_Back_40 extends LinearOpMode {
    //WHEELS
    public DcMotor Left_motors = null;
    public DcMotor Right_motors = null;
    public DcMotor Lifter = null;

    //SERVOS
    public Servo RightJServo = null;
    public Servo LeftJServo = null;
    public Servo RightCServo = null;
    public Servo LeftCServo = null;

    public String Multi;
    public String JewelColor;
    public String blue = "blue";
    public String red = "red";

    //SENSORS
    public ColorSensor LineSensor;
    public ColorSensor JewelSensor;
    ModernRoboticsI2cGyro GyroSensor;
    public ModernRoboticsI2cGyro GyroS;



    //VUFORIA
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE MAPPING
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");
        Lifter = hardwareMap.dcMotor.get("Lifter");

        Left_motors.setDirection(DcMotor.Direction.REVERSE);

        RightJServo = hardwareMap.servo.get("RightJServo");
        LeftJServo = hardwareMap.servo.get("LeftJServo");
        RightCServo = hardwareMap.servo.get("RightCServo");
        LeftCServo = hardwareMap.servo.get("LeftCServo");

        JewelSensor = hardwareMap.colorSensor.get("JewelSensor");
        //ColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c 0x26
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        //ColorSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro)GyroSensor;
        LineSensor.enableLed(true);
        JewelSensor.enableLed(true);




        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        GrabCubeandLift(0,.7,1, 200);
        sleep(1500);
        HitJewel(0, .7);
        sleep(3500);
        //RunToCryptoBoxNatural(.5, 1000, 1, 850);
    }
    //FUNCTIONS
    public void Forward (double power){
        Right_motors.setPower(power);
        Left_motors.setPower(power);
    }
    public void Reverse(double power){
        Right_motors.setPower(-power);
        Left_motors.setPower(-power);
    }
    public void Stop(){
        Forward(0);
    }
    public void LeftTurn(double power){
        Right_motors.setPower(power);
        Left_motors.setPower(-power);
    }
    public void RightTurn(double power){
        Right_motors.setPower(-power);
        Left_motors.setPower(power);
    }
    public void GrabCubeandLift(double posR, double posL, double power, int time){
        RightCServo.setPosition(posR);
        LeftCServo.setPosition(posL);
        Lifter.setPower(power);
        sleep(time);
        Lifter.setPower(0);
        telemetry.addLine(">  GrabCubeandLift phase complete");
        telemetry.update();
    }
    public void HitJewel(double posD,double posU) {
        LeftJServo.setPosition(posD);
        sleep(3000);
        CheckJewelColor();
        sleep(2000);
        if (JewelColor == "blue") {
            Left_motors.setPower(-.3);
            sleep(1000);
            Stop();
            LeftJServo.setPosition(posU);
            sleep(1000);
            Right_motors.setPower(6);
            sleep(1200);
            Stop();
            Forward(.4);
            sleep(2000);
            Stop();
        } else if (JewelColor == "red") {
            Left_motors.setPower(.3);
            sleep(1000);
            Stop();
            LeftJServo.setPosition(posU);
            sleep(1000);
            Left_motors.setPower(.3);
            Right_motors.setPower(1);
            sleep(600);
            Stop();
            Forward(.4);
            sleep(800);
            Stop();
        } else {
            telemetry.addLine("ERROR ON HIT JEWEL");
        }
        telemetry.addLine(">  HitJewel phase complete");
        telemetry.update();
    }
    public void CheckJewelColor(){
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(JewelSensor.red() * 8, JewelSensor.green() * 8, JewelSensor.blue() * 8, hsvValues);
        if (JewelSensor.blue() > JewelSensor.red() && JewelSensor.blue() > JewelSensor.green()) {
            telemetry.addLine("Color: blue");
            telemetry.update();
            JewelColor = blue;
        } else if (JewelSensor.red() > JewelSensor.blue() && JewelSensor.red() > JewelSensor.green()) {
            telemetry.addLine("Color: red");
            telemetry.update();
            JewelColor = red;
        }
        else {
            telemetry.addLine(">  NO DETECTED");
        }
        telemetry.addData(">  Jewel Color: ",JewelColor);
        telemetry.addLine(">  CheckJewelColor phase complete");
        telemetry.update();
    }
    public void RunToCryptoBoxNatural(double RghtTrnPwr, int TurnTime, double FwrdPwr, int FwrdTime){
        if (JewelColor == "blue") {
            RightTurn(RghtTrnPwr);
            sleep(TurnTime);
            Forward(FwrdPwr);
            sleep(FwrdTime);
            Stop();
            telemetry.addLine(">  RunToCryptoBoxNatural phase complete");
            telemetry.update();
        } else if (JewelColor == "red") {
            Reverse(1);
            sleep(3000);
            Stop();
            RightTurn(RghtTrnPwr);
            sleep(TurnTime);
            Forward(FwrdPwr);
            sleep(FwrdTime);
            Stop();
            telemetry.addLine(">  RunToCryptoBoxNatural phase complete");
            telemetry.update();
        } else {
            telemetry.addLine("ERROR ON HIT JEWEL");
        }

    }
}
