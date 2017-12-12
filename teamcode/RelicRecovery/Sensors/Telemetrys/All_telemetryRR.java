package org.firstinspires.ftc.teamcode.RelicRecovery.Sensors.Telemetrys;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 12/09/2017.
 */

@TeleOp(name ="All_telemetryRR",group = "TeleOp")
public class All_telemetryRR extends OpMode {
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
    public ColorSensor JewelColor;
    public String blue = "blue";
    public String red = "red";

    //SENSORS
    public ColorSensor LineSensor;
    public ColorSensor JewelSensor;
    public DistanceSensor RevDistanceSensor;
    public ColorSensor RevColorSensor;
    ModernRoboticsI2cGyro GyroSensor;
    public ModernRoboticsI2cGyro GyroS;


    @Override
    public void init() {

        //HARDWARE MAPPING
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");
        Lifter = hardwareMap.dcMotor.get("Lifter");

        Right_motors.setDirection(DcMotor.Direction.REVERSE);

        RightJServo = hardwareMap.servo.get("RightJServo");
        LeftJServo = hardwareMap.servo.get("LeftJServo");
        RightCServo = hardwareMap.servo.get("RightCServo");
        LeftCServo = hardwareMap.servo.get("LeftCServo");

        JewelColor = hardwareMap.colorSensor.get("JewelSensor");
        //ColorSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c 0x26
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        //ColorSensor2.setI2cAddress(I2cAddr.create7bit(0x1e));//3c
        GyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro)GyroSensor;
        LineSensor.enableLed(false);
        JewelSensor.enableLed(false);




        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    public void loop() {
        telemetry.addLine("LineSensor:");
        ColorTelemtry();
        telemetry.addLine("LineSensor:");
        Color2Telemtry();
        telemetry.addLine("GyroSensor:");
        GyroTelemetry();
        telemetry.update();
            
    }


    public void ColorTelemtry (){
        LineSensor.enableLed(true);

        float hsvValues[] = {0,0,0};


        Color.RGBToHSV(LineSensor.red()*8, LineSensor.green()*8,LineSensor.blue()*8,hsvValues);
        telemetry.addData("Azul", LineSensor.blue());
        telemetry.addData("Rojo", LineSensor.red());
        telemetry.addData("Verde", LineSensor.green());
    }
    public void Color2Telemtry (){
        LineSensor.enableLed(true);

        float hsvValues[] = {0,0,0};


        Color.RGBToHSV(LineSensor.red()*8, LineSensor.green()*8,LineSensor.blue()*8,hsvValues);
        telemetry.addData("Azul", LineSensor.blue());
        telemetry.addData("Rojo", LineSensor.red());
        telemetry.addData("Verde", LineSensor.green());
    }


    public void GyroTelemetry(){

        telemetry.addData("Heading: ", GyroSensor.getHeading());

    }
}
