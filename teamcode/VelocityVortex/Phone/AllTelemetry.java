package org.firstinspires.ftc.teamcode.VelocityVortex.Phone;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
/*
1cm = 1022  6cm = 60    11cm = 21
2cm = 440   7cm = 45    12cm = 17
3cm = 225   8cm = 35    13cm = 15
4cm = 130   9cm = 30    14cm = 13
5cm = 85    10cm =24    15cm = 12
 */

/**
 * Created by Jorge on 08/12/2016.
 */
//@TeleOp(name = "AllTelemetry")
public class AllTelemetry extends LinearOpMode {
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    private DcMotor LauncherR;
    private DcMotor LauncherL;
    private DcMotor banda;
    private DcMotor aspas;
    Servo ServoR;
    Servo ServoL;
    private ColorSensor LineSensor;
    private ColorSensor BeaconSensor;
    private OpticalDistanceSensor WallSensor;
    private GyroSensor GyroSensor;
    public ModernRoboticsI2cGyro GyroS;

    String white = "white";
    String blue = "blue";
    String red = "red";
    String black = "black";
    String sColor = "";
    String sColorB = "";
    String Multi;

    static double odsReadngRaw;
    static double odsReadingLinear;
    static int odsEstimatedDistance;
    static double m = 56;
    static double b = -0.356;
    double currentHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        LauncherL = hardwareMap.dcMotor.get("LauncherL");
        LauncherR = hardwareMap.dcMotor.get("LauncherR");
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        banda = hardwareMap.dcMotor.get("banda");
        aspas = hardwareMap.dcMotor.get("aspas");
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        BeaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        LineSensor.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        WallSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
        GyroSensor = hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro)GyroSensor;
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        LauncherL.setDirection(DcMotor.Direction.REVERSE);
        LineSensor.enableLed(false);
        BeaconSensor.enableLed(false);

        //BEFORE WE PRESS START GYRO SENSOR NEEDS TO BE CALIBRATED
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        GyroSensor.calibrate();

        //WHILE START ISNT PRESSED GIVE SOME TIME TO CALIBRATE
        while (!isStopRequested() && GyroSensor.isCalibrating())  {
            sleep(500);
            idle();
        }

        //SEND CALIBRATION BY TELEMETRY
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        sleep(200);
        while (!opModeIsActive()){
            telemetry.addData("Grados: ", GyroS.getIntegratedZValue());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()){
            ColorTelemtry();
            ODSTelemetry();
            GyroTelemetry();
            telemetry.update();
        }

    }

   public void ColorTelemtry (){
       BeaconSensor.enableLed(false);

       float hsvValues[] = {0,0,0};


       Color.RGBToHSV(BeaconSensor.red()*8, BeaconSensor.green()*8,BeaconSensor.blue()*8,hsvValues);
       telemetry.addData("Azul", BeaconSensor.blue());
       telemetry.addData("Rojo", BeaconSensor.red());
       telemetry.addData("Verde", BeaconSensor.green());
   }

   public void ODSTelemetry (){
       odsReadngRaw = WallSensor.getRawLightDetected();                       //update raw value
       odsReadingLinear = Math.pow(odsReadngRaw, -0.5);                //calculate linear value
       odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);

       telemetry.addData("Distance", odsEstimatedDistance);

   }

   public void GyroTelemetry(){

       telemetry.addData("Heading: ", GyroSensor.getHeading());

   }
}