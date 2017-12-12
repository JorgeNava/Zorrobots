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


//@TeleOp(name = "Example1", group = "Sensor")
//@Disabled
public class Example1 extends LinearOpMode {

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

        waitForStart();
        while (opModeIsActive()) {
            odsReadngRaw = odsSensor.getRawLightDetected();
            odsReadingLinear = Math.pow(odsReadngRaw, -0.5);
            odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);

            //Llega a Tape
            Avanzar(1);
            sleep(1000);
            DistanciaInicial = odsEstimatedDistance;
            GirarDerecha(0.3);
            DistanciaFinal = odsEstimatedDistance;
            if (DistanciaInicial - DistanciaFinal > 10) {
                Frenar();
                break;
            } else {
                DistanciaInicial = DistanciaFinal;
            }
            AvanzarHastaDistanciaX();

        }
    }
    public void AvanzarHastaDistanciaX(){
        if (odsEstimatedDistance < 100){
            Avanzar(1);
        }
        else {
            Frenar();
        }
    }
    public void Avanzar (double power){
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
        motorRightB.setPower(power);
        motorRightF.setPower(power);
    }
    public void Frenar (){
        Avanzar(0);
    }
    public void GirarDerecha (double power){
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
    }
}
