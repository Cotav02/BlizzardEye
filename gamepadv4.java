/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static android.os.SystemClock.sleep;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="TESTARE", group="Linear Opmode")
@Disabled
public class gamepadv4 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_drive = null;
    private DcMotor right_drive = null;
    private DcMotor left_drive1 = null;
    private DcMotor right_drive1 = null;
    private DistanceSensor sensorRange;
    private DcMotor brat = null;
    CRServo servo;
    CRServo servo1;
    CRServo   servo2;
    Servo servo3;
    Servo tava1;
    Servo tava0;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
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

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive1.setDirection(DcMotor.Direction.FORWARD);
        right_drive1.setDirection(DcMotor.Direction.REVERSE);
        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 1;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        {
            telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();
       int    CYCLE_MS    =   50;
        sleep(CYCLE_MS);}
            while (sensorRange.getDistance(DistanceUnit.CM)>25)
            {
                //mers fata
                left_drive.setPower(0.5);
                right_drive.setPower(0.5);
                left_drive1.setPower(0.5);
                right_drive1.setPower(0.5);
            }
            //STOP
            left_drive.setPower(0);
            right_drive.setPower(0);
            left_drive1.setPower(0);
            right_drive1.setPower(0);

            //DETECTIA SKYSTONE-ULUI PROGRES 0%




            //STOP
            left_drive.setPower(0);
            right_drive.setPower(0);
            left_drive1.setPower(0);
            right_drive1.setPower(0);
            //COBOARA BRATUL
            servo.setPower(0.35);
            servo1.setPower(0.35);
            sleep(2000);
            //STRANGE BRATUL
            servo3.setPosition(0);
            sleep(1000);
            //RIDICA BRATUL
            servo.setPower(0.25);
            servo1.setPower(-0.25);
            servo3.setPosition(0);
            sleep(1000);
            //MERGI PUTIN IN SPATE SA NE SE AGATE DE MIJLOC ATUNCI CAND MERGE IN DREAPTA
            left_drive.setPower(-0.5);
            right_drive.setPower(-0.5);
            left_drive1.setPower(-0.5);
            right_drive1.setPower(-0.5);
            servo3.setPosition(0);
            servo.setPower(0);
            servo1.setPower(0);
            sleep(200);
            servo3.setPosition(0);
            //MERGI IN DREAPTA
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_drive.setTargetPosition(2430);
            left_drive.setPower(-1);
            right_drive.setPower(1 );
            left_drive1.setPower(-1 );
            right_drive1.setPower(1 );
            //TRECE DE MIJLOC DACA PRIMUL CUB E SKYSTONE
            //sleep(1350);
            left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_drive.setTargetPosition(1800);
            left_drive.setPower(1);
            right_drive.setPower(1 );
            left_drive1.setPower(1 );
            right_drive1.setPower(1 );
            sleep(1000);
            //STOP
            left_drive.setPower(0);
            right_drive.setPower(0 );
            left_drive1.setPower(0 );
            right_drive1.setPower(0 );
            //DESFACE BRATUL
            servo3.setPosition(1);
            servo.setPower(0.1);
            servo1.setPower(-0.1);
            sleep(500);
            //MERGI INAPOI DUPA AL DOILE CUB
            left_drive.setPower(-1);
            right_drive.setPower(1 );
            left_drive1.setPower(1 );
            right_drive1.setPower(-1 );
            //DUPA CE TRECE DE MIJLOCUL TERENULUI RIDICA BRATUL CA SA PRINDA AL DOILEA CUB, FIIND AL DOILEA SKYSTONE SI PE TEREN ESTE AL 3-LEA CUB
            sleep(1500);
            left_drive.setPower(-1);
            right_drive.setPower(1 );
            left_drive1.setPower(1 );
            right_drive1.setPower(-1 );
            servo.setPower(0.25);
            servo1.setPower(-0.25);

            sleep(1600);
            servo.setPower(0);
            servo1.setPower(0);

            //MERGE IN FATA SI SE OPRESTE LA FIX CA SA COBOARE BRATUL SI SA PRINDA CUBUL
            while (sensorRange.getDistance(DistanceUnit.CM)>25) {
                left_drive.setPower(0.5);
                right_drive.setPower(0.5);
                left_drive1.setPower(0.5);
                right_drive1.setPower(0.5);
            }
            //STOP
            left_drive.setPower(0);
            right_drive.setPower(0);
            left_drive1.setPower(0);
            right_drive1.setPower(0);
            //COBOARA BRATUL
            servo.setPower(-0.35);
            servo1.setPower(0.35);
            sleep(2000);
            //STRANGE BRATUL
            servo3.setPosition(0);
            sleep(1000);
            //RIDICA BRATUL PUTIN, NU TOTAL
            servo.setPower(0.25);
            servo1.setPower(-0.25);
            servo3.setPosition(0);
            sleep(1000);
            //MERGI OLEACA IN FATA SA NU SI AGATI BRATU
            left_drive.setPower(-0.5);
            right_drive.setPower(-0.5);
            left_drive1.setPower(-0.5);
            right_drive1.setPower(-0.5);
            servo3.setPosition(0);
            servo.setPower(0);
            servo1.setPower(0);
            sleep(10);
            //MERGI IN DREAPTA
            servo3.setPosition(0);
            left_drive.setPower(1);
            right_drive.setPower(-1 );
            left_drive1.setPower(-1 );
            right_drive1.setPower(1 );
            sleep(3500);
            //STOP
            left_drive.setPower(0);
            right_drive.setPower(0 );
            left_drive1.setPower(0 );
            right_drive1.setPower(0 );
            //DESFACE BRATUL
            servo3.setPosition(1);
            sleep(500);
            //RIDICA PUTIN BRATUL CA SA NU AGATE AL DOILEA CUB CAND MERGE SPRE MIJLOC
            servo.setPower(0.15);
            servo1.setPower(-0.15);
            //MERGE STANGA CAT SA SE OPREASCA LA MIJLOC
            left_drive.setPower(-1);
            right_drive.setPower(1 );
            left_drive1.setPower(1 );
            right_drive1.setPower(-1 );
            sleep(1250);
            //STOP -S-A PARCAT-
            servo.setPower(0);
            servo1.setPower(0);
            left_drive.setPower(0);
            right_drive.setPower(0 );
            left_drive1.setPower(0 );
            right_drive1.setPower(0 );
            sleep(20000);







        }
    }
}