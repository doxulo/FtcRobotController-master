package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.HashMap;

public class Sequence {

    DcMotorEx RF;
    DcMotorEx RB;
    DcMotorEx LF;
    DcMotorEx LB;

    public Sequence(DcMotorEx RF, DcMotorEx RB, DcMotorEx LF, DcMotorEx LB) {
        this.RF = RF;
        this.RB = RB;
        this.LF = LF;
        this.LB = LB;
    }

    public HashMap<String, Integer> getCurrentPositions() {
        HashMap<String, Integer> encoderPositions = new HashMap<>();
        encoderPositions.put("RF", RF.getCurrentPosition());
        encoderPositions.put("LF", LF.getCurrentPosition());
        encoderPositions.put("RB", RB.getCurrentPosition());
        encoderPositions.put("LB", LB.getCurrentPosition());

        return encoderPositions;
    }

    public HashMap<String, Integer> forward(double inches) {
        int ticks = (int) PIDCommands.inchesToTicks(inches);
        HashMap<String, Integer> encoderPositions = this.getCurrentPositions();
        encoderPositions.put("RF", encoderPositions.get("RF")+ticks);
        encoderPositions.put("LF", encoderPositions.get("LF")-ticks);
        encoderPositions.put("RB", encoderPositions.get("RB")-ticks);
        encoderPositions.put("LB", encoderPositions.get("LB")-ticks);
        return encoderPositions;
    }

    public HashMap<String, Integer> strafeRight(double inches) {
        int ticks = (int) PIDCommands.inchesToTicks(inches);
        HashMap<String, Integer> encoderPositions = this.getCurrentPositions();
        encoderPositions.put("RF", encoderPositions.get("RF")+ticks);
        encoderPositions.put("LF", encoderPositions.get("LF")+ticks);
        encoderPositions.put("RB", encoderPositions.get("RB")-ticks);
        encoderPositions.put("LB", encoderPositions.get("LB")+ticks);

        return encoderPositions;
    }



}
