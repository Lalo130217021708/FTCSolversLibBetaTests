package org.firstinspires.ftc.teamcode.Initializers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerInitializer {
    public static double LSx1, LSy1, RSx1, RSy1;
    public static double LT1, RT1, LB1, RB1;
    public static double A1, B1, X1, Y1;
    public static boolean dPadUp1, dPadDown1, dPadRight1, dPadLeft1;

    public static double LSx2, LSy2, RSx2, RSy2;
    public static double LT2, RT2, LB2, RB2;
    public static double A2, X2, Y2, B2;
    public static boolean dPadUp2, dPadDown2, dPadRight2, dPadLeft2;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public ControllerInitializer(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public double applyDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {return 0.0;}
        return input;
    }
    public void actualizeGamepad(){
        actualizeGamepad1();
        actualizeGamepad2();
    }
    
    public void actualizeGamepad1(){
        LSx1 = applyDeadband(gamepad1.left_stick_x , .05);
        LSy1 = applyDeadband(gamepad1.left_stick_y , .05);
        RSx1 = applyDeadband(gamepad1.right_stick_x , .05);
        RSy1 = applyDeadband(-gamepad1.right_stick_y , .05);

        LT1 = gamepad1.left_trigger;
        RT1 = gamepad1.right_trigger;
        LB1 = gamepad1.left_bumper ? 1 : 0;
        RB1 = gamepad1.right_bumper ? 1 : 0;

        A1 = gamepad1.a ? 1.0 : 0;
        X1 = gamepad1.x ? 1.0 : 0;
        Y1 = gamepad1.y ? 1.0 : 0;
        B1 = gamepad1.b ? 1.0 : 0;

        dPadUp1 = gamepad1.dpad_up;
        dPadDown1 = gamepad1.dpad_down;
        dPadRight1 = gamepad1.dpad_right;
        dPadLeft1 = gamepad1.dpad_left;
    }
    public void actualizeGamepad2(){
        LSx2 = applyDeadband(gamepad2.left_stick_x , .5);
        LSy2 = applyDeadband(gamepad2.left_stick_y , .5);
        RSx2 = applyDeadband(gamepad2.right_stick_x , .5);
        RSy2 = applyDeadband(gamepad2.right_stick_y , .5);

        LT2 = gamepad2.left_trigger;
        RT2 = gamepad2.right_trigger;
        LB2 = gamepad2.left_bumper ? 1 : 0;
        RB2 = gamepad2.right_bumper ? 1 : 0;

        A2 = gamepad2.a ? 1.0 : 0;
        X2 = gamepad2.x ? 1.0 : 0;
        Y2 = gamepad2.y ? 1.0 : 0;
        B2 = gamepad2.b ? 1.0 : 0;

        dPadUp2 = gamepad2.dpad_up;
        dPadDown2 = gamepad2.dpad_down;
        dPadRight2 = gamepad2.dpad_right;
        dPadLeft2 = gamepad2.dpad_left;
    }

}
