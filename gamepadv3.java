package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Gamepad V1.1", group="Control")
@Disabled
public class gamepadv3 extends OpMode
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



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        right_drive1 = hardwareMap.get(DcMotor.class, "right_drive1");
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
        double up;
        double power_rotate = 0.5;
        boolean a;
        boolean y;
        Power = -gamepad1.left_stick_y;
        Direction = -gamepad1.left_stick_x;
        axysy = -gamepad1.right_stick_y;
        axysx = -gamepad1.right_stick_x;
        spin_left = gamepad1.left_trigger;
        spin_right = gamepad1.right_trigger;
        a = gamepad1.a;
        y = gamepad1.y;

        // inceput joystyck dreapta
        // FATA/SPATE

        int optiune = 0;
        if (Power != 0) {
            left_drive.setPower(Power);
            right_drive.setPower(Power);
            left_drive1.setPower(Power);
            right_drive1.setPower(Power);
        }
        else if (Direction != 0) {
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
        if (a) {
            brat.setPower(-power_rotate);
        } else if (y) {
            brat.setPower(power_rotate);
        } else
            brat.setPower(0);

        // se afiseaza timpul scurs si puterea
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motoare", "FATA/SPATE (%.2f), STANGA/DREAPTA (%.2f)", Power, Direction);
        telemetry.addData("deviceName",sensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

        telemetry.update();
    }

}