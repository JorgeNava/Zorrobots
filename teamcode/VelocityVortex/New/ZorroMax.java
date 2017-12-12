package org.firstinspires.ftc.teamcode.VelocityVortex.New;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 21/01/2017.
 */
//@TeleOp(name ="ZorroMax",group = "TeleOp")
public class ZorroMax extends OpMode {
    DcMotor Motor1;
    DcMotor Motor2;
    DcMotor Motor3;
    DcMotor Motor4;

    @Override
    public void init() {
        Motor1 = hardwareMap.dcMotor.get("Motor1");
        Motor2 = hardwareMap.dcMotor.get("Motor2");
        Motor3 = hardwareMap.dcMotor.get("Motor3");
        Motor4 = hardwareMap.dcMotor.get("Motor4");

        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        Motor1.setPower(gamepad1.right_stick_y);
        Motor2.setPower(gamepad1.right_stick_y);
        Motor3.setPower(gamepad1.left_stick_y);
        Motor4.setPower(gamepad1.left_stick_y);


    }
}
