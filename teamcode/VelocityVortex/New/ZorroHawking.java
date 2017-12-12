package org.firstinspires.ftc.teamcode.VelocityVortex.New;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 21/01/2017.
 */
//@TeleOp(name ="ZorroHawking",group = "TeleOp")
public class ZorroHawking extends OpMode {
  /*  DcMotor MotorLeftF;
    DcMotor MotorLeftB;
    DcMotor MotorRightF;
    DcMotor MotorRightB;*/
    DcMotor Catapulta;
    DcMotor Banda;
    Servo DafaCadaDePitoDosPuntosUveNinoData;

    @Override
    public void init() {
        //MotorLeftF = hardwareMap.dcMotor.get("MotorLeftF");
        //MotorLeftB = hardwareMap.dcMotor.get("MotorLeftB");
        //MotorRightF = hardwareMap.dcMotor.get("MotorRightF");
        //MotorRightB = hardwareMap.dcMotor.get("MotorRightB");
        Banda = hardwareMap.dcMotor.get("Banda");
        Catapulta = hardwareMap.dcMotor.get("Catapulta");
        DafaCadaDePitoDosPuntosUveNinoData = hardwareMap.servo.get("DafaCadaDePitoDosPuntosUveNinoData");


        //MotorLeftF.setDirection(DcMotor.Direction.REVERSE);
        //MotorLeftB.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        /*MotorRightB.setPower(gamepad1.right_stick_y);
        MotorRightF.setPower(gamepad1.right_stick_y);
        MotorLeftB.setPower(gamepad1.left_stick_y);
        MotorLeftF.setPower(gamepad1.left_stick_y);*/
        if (gamepad1.right_bumper){
            Catapulta.setPower(.7);
        }
        else {
            Catapulta.setPower(0);
        }
        if (gamepad1.y){
            Banda.setPower(1);
        }
        else if(gamepad1.x){
            Banda.setPower(-1);
        }
        else{
            Banda.setPower(0);
        }

        if (gamepad1.a){
            DafaCadaDePitoDosPuntosUveNinoData.setPosition(1);
        }
        else if (gamepad1.b){
            DafaCadaDePitoDosPuntosUveNinoData.setPosition(.5);
        }
        else{
            DafaCadaDePitoDosPuntosUveNinoData.setPosition(0);
        }
    }
}
