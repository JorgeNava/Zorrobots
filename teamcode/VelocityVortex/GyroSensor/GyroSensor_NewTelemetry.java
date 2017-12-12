package org.firstinspires.ftc.teamcode.VelocityVortex.GyroSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 16/12/2016.
 */

//@TeleOp(name ="GyroSensor_NewTelemetry",group = "TeleOp")
public class GyroSensor_NewTelemetry extends LinearOpMode {
    public DcMotor motorLeftF;
    public DcMotor motorRightF;
    public DcMotor motorLeftB;
    public DcMotor motorRightB;
    public DcMotor LauncherR;
    public DcMotor LauncherL;
    public DcMotor aspas;
    public DcMotor banda;
    public Servo ServoR;
    public Servo ServoL;
    public ColorSensor LineSensor;
    public ColorSensor BeaconSensor;
    public OpticalDistanceSensor WallSensor;
    public GyroSensor GyroSensor;
    public ModernRoboticsI2cGyro GyroS;
    double Z;
    double heading;
    double xVal;
    double yVal;
    double zVal;


    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        LauncherL = hardwareMap.dcMotor.get("LauncherL");
        LauncherR = hardwareMap.dcMotor.get("LauncherR");
        aspas = hardwareMap.dcMotor.get("aspas");
        banda = hardwareMap.dcMotor.get("banda");
        LineSensor = hardwareMap.colorSensor.get("LineSensor");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        WallSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        GyroSensor = hardwareMap.gyroSensor.get("GyroSensor");
        GyroS = (ModernRoboticsI2cGyro)GyroSensor;
        motorRightF.setDirection(DcMotor.Direction.REVERSE);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);
        LauncherL.setDirection(DcMotor.Direction.REVERSE);
        LineSensor.enableLed(false);
        BeaconSensor.enableLed(false);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Z: ", GyroS.getIntegratedZValue());
            telemetry.update();
        }
    }
}



