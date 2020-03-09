package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Gamepad V2", group="Control")
@Disabled
public class gamepadv1 extends OpMode
{
    // Declare OpMode members.
    // initializare componente
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_drive = null;
    private DcMotor right_drive = null;
    private DcMotor left_drive1 = null;
    private DcMotor right_drive1 = null;
    private DistanceSensor sensorRange;
    private DcMotor brat = null;
    private Servo   servo;
    private Servo   servo2;
    private Servo   servo3;
    private Servo   servo1;
    private double position1 = 0.5;
    private double  position = 0.5;
    private double  position2 = 0.5;
    private double  position3 = 0.5;




    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        right_drive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        brat = hardwareMap.get(DcMotor.class, "brat");
        servo1 = hardwareMap.get(Servo.class, "servo0");
        servo = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
        brat = hardwareMap.get(DcMotor.class, "brat");

        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive1.setDirection(DcMotor.Direction.FORWARD);
        right_drive1.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }



    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double Power;
        double Direction;
        double axysy;
        double axysx;
        double spin_left;
        double spin_right;
        double power_rotate = 0.4;
        double schimb;

        schimb=1;
        if(gamepad1.b)
            schimb= 1;
        if (gamepad1.x)
            schimb=0;


        Power = -gamepad1.left_stick_y;
        Direction = -gamepad1.left_stick_x;
        axysy = -gamepad1.right_stick_y;
        axysx = -gamepad1.right_stick_x;
        spin_left = gamepad1.left_trigger;
        spin_right = gamepad1.right_trigger;


        // inceput joystyck dreapta
        // FATA/SPATE
        schimb=1;
        if (gamepad1.b)
            schimb=0;
        if (schimb == 1) {
            if (Power != 0) {
                left_drive.setPower(Power);
                right_drive.setPower(Power);
                left_drive1.setPower(Power);
                right_drive1.setPower(Power);
            } else if (Direction != 0) {
                left_drive.setPower(-Direction);
                right_drive.setPower(Direction);
                left_drive1.setPower(Direction);
                right_drive1.setPower(-Direction);
            } else
                //fata-spate stanga
                if (axysy != 0) {
                    left_drive.setPower(axysy);
                    right_drive.setPower(0);
                    left_drive1.setPower(0);
                    right_drive1.setPower(axysy);
                } else
                    //stanga-spate
                    //fata-spate dreapta
                    if (axysx != 0) {
                        left_drive.setPower(0);
                        right_drive.setPower(axysx);
                        left_drive1.setPower(axysx);
                        right_drive1.setPower(0);
                    } else if (spin_left != 0) {
                        left_drive.setPower(-spin_left);
                        right_drive.setPower(spin_left);
                        left_drive1.setPower(-spin_left);
                        right_drive1.setPower(spin_left);
                    } else if (spin_right != 0) {
                        left_drive.setPower(spin_right);
                        right_drive.setPower(-spin_right);
                        left_drive1.setPower(spin_right);
                        right_drive1.setPower(-spin_right);
                    } else {
                        left_drive.setPower(0);
                        right_drive.setPower(0);
                        left_drive1.setPower(0);
                        right_drive1.setPower(0);
                        telemetry.addData("Status", "Nu apesi nimic!");
                    }
        }
        else
        {
            if (gamepad1.right_stick_y!=0)
                if(gamepad1.right_stick_y>0)
                    if(position<1.00001 && position>0.00001) {
                        position += 0.0001;
                        position1 += -0.0001;
                    }
                else
                if(gamepad1.right_stick_y<0)
                    if(position<1.00001 && position>0.00001) {
                        position -= 0.0001;
                        position1 -= -0.0001;
                    }
            //brat-servo2-stanga-dreapta
            if(gamepad1.right_stick_x!=0)
                if(position2<1.00001 && position2>0.00001)
                    position2+=0.0001;
                else
                if(gamepad1.right_stick_x<0)
                    if(position2<1.00001 && position2>0.00001)
                        position2-=0.0001;

            //brat-servo3-prindere
            if(gamepad1.left_stick_x!=0)
                if(position3<1.00001 && position3>0.00001)
                    position3+=0.0001;

                else
                if(gamepad1.left_stick_x<0)
                    if(position3<1.00001 && position3>0.00001)
                        position3-=0.0001;

            //lift-sus-jos
            if (gamepad1.a) {
                brat.setPower(-power_rotate);
            } else if (gamepad1.y) {
                brat.setPower(power_rotate);
            } else
                brat.setPower(0);

            //se afiseaza pozitia servoului
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData("Servo Position", "%5.2f", position2);
            telemetry.addData("Servo Position", "%5.2f", position3);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();
            servo.setPosition(position);
            servo2.setPosition(position2);
            servo3.setPosition(position3);
        }
        // se afiseaza timpul scurs si puterea, distanta
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motoare", "FATA/SPATE (%.2f), STANGA/DREAPTA (%.2f)", Power, Direction);
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));

        telemetry.update();
    }

}