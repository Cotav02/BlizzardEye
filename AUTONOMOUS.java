package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static android.os.SystemClock.sleep;

@Autonomous(name="DEMO AUTONOMOUS", group="Control")
//@Disabled
public class AUTONOMOUS extends OpMode
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
        if(sensorRange.getDistance(DistanceUnit.CM)<10)
        {
            servo.setPower(0.5);
            servo1.setPower(0.5);
            sleep(500);
            servo3.setPosition(1);
            left_drive.setPower(0);
            right_drive.setPower(0);
            left_drive1.setPower(0);
            right_drive1.setPower(0);
        }
        else
            while (sensorRange.getDistance(DistanceUnit.CM)>10) {
                left_drive.setPower(1);
                right_drive.setPower(1);
                left_drive1.setPower(1);
                right_drive1.setPower(1);
            }

        runtime.reset();
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry



        telemetry.update();
    }

}