package org.firstinspires.ftc.teamcode.VelocityVortex.ODS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
/*
1cm = 1022  6cm = 60    11cm = 21
2cm = 440   7cm = 45    12cm = 17
3cm = 225   8cm = 35    13cm = 15
4cm = 130   9cm = 30    14cm = 13
5cm = 85    10cm =24    15cm = 12
 */


//@Autonomous(name = "OdsExample_2", group = "Sensor")
//@Disabled
public class OdsExample_2 extends LinearOpMode {

    OpticalDistanceSensor odsSensor;
    DcMotor motorLeftF;
    DcMotor motorLeftB;
    DcMotor motorRightF;
    DcMotor motorRightB;

    static double reading2cm = 800;
    static double reading10cm = 20;
    static double odsReadngRaw;
    static double odsReadingLinear;
    static int odsEstimatedDistance;
    static double m = 56;
    static double b = -0.356;
    double DistanciaInicial = odsEstimatedDistance;
    double DistanciaFinal;


    @Override
    public void runOpMode() {

        odsSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            odsReadngRaw = odsSensor.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);
            odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);

            /*Girar(.5);
            sleep(2000);
*/
            Avanzar(-.2);
            while (odsEstimatedDistance < 145) {
                motorLeftB.setPower(0);
                motorLeftF.setPower(0);
                motorRightB.setPower(0);
                motorRightF.setPower(0);
            }
        }
    }


    public void Avanzar (double power) {
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
    }
    public void Girar (double power){
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
    }
    public void Frenar (){
        Girar(0);
        Avanzar(0);
    }
}

