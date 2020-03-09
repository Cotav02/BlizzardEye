package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="DEMO IASI", group="Control")
//@Disabled
public class gamepads2 extends OpMode
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
    CRServo   servo;
    CRServo servo1;
    CRServo   servo2;
    Servo servo3;
    Servo tava1;
    Servo tava0;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        right_drive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        brat = hardwareMap.get(DcMotor.class, "brat");
        servo = hardwareMap.crservo.get( "servo0");
        servo1 = hardwareMap.crservo.get( "servo1");
        servo2 = hardwareMap.crservo.get( "servo2");
        servo3 = hardwareMap.servo.get( "servo3");
        tava0 = hardwareMap.servo.get( "tava1");
        tava1 = hardwareMap.servo.get( "tava2");

       // sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive1.setDirection(DcMotor.Direction.FORWARD);
        right_drive1.setDirection(DcMotor.Direction.REVERSE);
      /*  float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues); */
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
        double position=0.5;
        double power_rotate = 1;
        if(gamepad2.x)
        {
            tava0.setPosition(1);
            tava1.setPosition(0.5);
        }
        if(gamepad2.b)
        {
            tava0.setPosition(0);
            tava1.setPosition(1);
        }

// inceput joystyck dreapta
        // FATA/SPATE

            if (gamepad1.left_stick_y != 0) {
                left_drive.setPower(-gamepad1.left_stick_y );
                right_drive.setPower(-gamepad1.left_stick_y );
                left_drive1.setPower(-gamepad1.left_stick_y );
                right_drive1.setPower(-gamepad1.left_stick_y );

            }

            else
                //dreapta stanga
                if (gamepad1.right_stick_x != 0) {
                left_drive.setPower(gamepad1.right_stick_x);
                right_drive.setPower(-gamepad1.right_stick_x);
                left_drive1.setPower(-gamepad1.right_stick_x);
                right_drive1.setPower(gamepad1.right_stick_x);
            } else
                //rotire stanga
                if (gamepad1.left_trigger != 0) {
                        left_drive.setPower(-gamepad1.left_trigger);
                        right_drive.setPower(gamepad1.left_trigger);
                        left_drive1.setPower(-gamepad1.left_trigger);
                        right_drive1.setPower(gamepad1.left_trigger);
                    } else
                        //rotire dreapta
                        if (gamepad1.right_trigger != 0) {
                        left_drive.setPower(gamepad1.right_trigger );
                        right_drive.setPower(-gamepad1.right_trigger );
                        left_drive1.setPower(gamepad1.right_trigger );
                        right_drive1.setPower(-gamepad1.right_trigger );}
                         else
                             //gamepad2 fata
                            if (gamepad2.dpad_up)
                            {
                                left_drive.setPower(0.4);
                                right_drive.setPower(0.4 );
                                left_drive1.setPower(0.4 );
                                right_drive1.setPower(0.4 );
                            }
                            else
                                //gamepad2 spate
                                if (gamepad2.dpad_down)
                            {
                                left_drive.setPower(-0.4);
                                right_drive.setPower(-0.4 );
                                left_drive1.setPower(-0.4 );
                                right_drive1.setPower(-0.4 );
                            }
                            else
                                //gamepad2 dreapta
                                if (gamepad2.dpad_right)
                            {
                                left_drive.setPower(0.4);
                                right_drive.setPower(-0.4 );
                                left_drive1.setPower(-0.4 );
                                right_drive1.setPower(0.4 );
                            }
                            else
                                //gamepad2 stanga
                                if (gamepad2.dpad_left)
                            {
                                left_drive.setPower(-0.4);
                                right_drive.setPower(0.4 );
                                left_drive1.setPower(0.4 );
                                right_drive1.setPower(-0.4 );
                            }
                                else
                                    if (gamepad2.left_trigger!= 0){
                                        left_drive.setPower(-gamepad2.left_trigger);
                                        right_drive.setPower(gamepad2.left_trigger);
                                        left_drive1.setPower(-gamepad2.left_trigger);
                                        right_drive1.setPower(gamepad2.left_trigger);
                                    }
                                        else
                                            if(gamepad2.right_trigger!=0)
                                            {
                                                left_drive.setPower(gamepad2.right_trigger );
                                                right_drive.setPower(-gamepad2.right_trigger );
                                                left_drive1.setPower(gamepad2.right_trigger );
                                                right_drive1.setPower(-gamepad2.right_trigger );
                                            }
                    else
                        //oprire cand nu se apasa nimic
                        {
                        left_drive.setPower(0);
                        right_drive.setPower(0);
                        left_drive1.setPower(0);
                        right_drive1.setPower(0);
                        telemetry.addData("Status", "Nu apesi nimic!");
                    }




            if (gamepad2.right_stick_y!=0)
                     {
                        servo.setPower(-gamepad2.right_stick_y/2);
                        servo1.setPower(gamepad2.right_stick_y/2);

                    }
            else
            {
                servo.setPower(0);
                servo1.setPower(0);
            }

            //brat-servo2-stanga-dreapta
            if(gamepad2.right_stick_x!=0)
                servo2.setPower(gamepad2.right_stick_x/2);
            else
                servo2.setPower(0);

            //brat-servo3-prindere
            if(gamepad2.left_stick_y==1)
                {
                    position=0;
                }
            if(gamepad2.left_stick_y==-1)
                position=1;

            servo3.setPosition(position);

            //lift-sus-jos
            if (gamepad2.a) {
                brat.setPower(-power_rotate);
            } else if (gamepad2.y) {
                brat.setPower(power_rotate);
            } else
                brat.setPower(0);


        // se afiseaza timpul scurs si puterea, distanta
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motoare", "FATA/SPATE (%.2f), STANGA/DREAPTA (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.addData("joystick2 batstanga", "FATA/SPATE (%.2f), STANGA/DREAPTA (%.2f)", gamepad2.left_stick_y, gamepad1.right_stick_x);
       telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));/*
        telemetry.addData("SENZORI", "CULORI");
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue()); */


        telemetry.update();
    }

}