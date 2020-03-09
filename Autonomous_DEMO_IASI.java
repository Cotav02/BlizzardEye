package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static android.os.SystemClock.sleep;


@Autonomous(name="DEMO IASI", group="Control")
//@Disabled
public class Autonomous_DEMO_IASI extends OpMode
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
    CRServo   servo2;
    CRServo servo1;
    CRServo servo3;
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
       /* servo = hardwareMap.crservo.get( "servo0");
        servo1 = hardwareMap.crservo.get( "servo1");
        servo2 = hardwareMap.crservo.get( "servo2");
        servo3 = hardwareMap.crservo.get( "servo3");*/

        brat = hardwareMap.get(DcMotor.class, "brat");
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
        double Power;
        double Direction;
        double axysy;
        double axysx;
        double spin_left;
        double spin_right;
        double power_rotate = 0.4;
        Power = -gamepad1.left_stick_y;
        Direction = -gamepad1.left_stick_x;
        axysy = -gamepad1.right_stick_y;
        axysx = -gamepad1.right_stick_x;
        spin_left = gamepad1.left_trigger;
        spin_right = gamepad1.right_trigger;
// inceput joystyck dreapta
        // FATA/SPATE


            left_drive.setPower(0.4);
            right_drive.setPower(-0.4);
            left_drive1.setPower(-0.4);
            right_drive1.setPower(0.4);
            /*sleep(1000);
        left_drive.setPower(0.4);
        right_drive.setPower(-0.4);
        left_drive1.setPower(-0.4);
        right_drive1.setPower(0.4);      */
            sleep(1000);
        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive1.setPower(0);
        right_drive1.setPower(0);




           /* if (Power != 0) {
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



            if (gamepad2.right_stick_y!=0)
                     {
                        servo.setPower(-gamepad2.right_stick_y);
                        servo1.setPower(-gamepad2.right_stick_y);
                    }
            else
            {
                servo.setPower(0);
                servo1.setPower(0);
            }

            //brat-servo2-stanga-dreapta
            if(gamepad2.right_stick_x!=0)
                servo2.setPower(gamepad2.right_stick_x);
            else
                servo2.setPower(0);

            //brat-servo3-prindere
            if(gamepad2.left_stick_x!=0)
                servo3.setPower(gamepad2.left_stick_x);
            else
                servo3.setPower(0);

            //lift-sus-jos
            if (gamepad2.a) {
                brat.setPower(-power_rotate);
            } else if (gamepad2.y) {
                brat.setPower(power_rotate);
            } else
                brat.setPower(0);

        */
        // se afiseaza timpul scurs si puterea, distanta
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motoare", "FATA/SPATE (%.2f), STANGA/DREAPTA (%.2f)", Power, Direction);
      /*  telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("SENZORI", "CULORI");
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue()); */


        telemetry.update();
    }

}