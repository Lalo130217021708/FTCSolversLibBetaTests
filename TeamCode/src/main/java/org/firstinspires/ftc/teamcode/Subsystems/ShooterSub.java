package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.distance;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.dShooter;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.fShooter;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.iShooter;
import static org.firstinspires.ftc.teamcode.Configurations.PIDValues.shooterConfigurableVariables.pShooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;

public class ShooterSub {
    private final MotorEx shooterMotor, shooterMotor2;
    public static double shooterVel, shooterVel2, rpmsError;
    private final MotorGroup shooterMotors;
    public static InterpLUT interpLUT;
    private final VoltageCompensator voltageCompensator;
    public double desiredRPMs;
    public ShooterSub(HardwareMap hardwareMap, VoltageCompensator voltageCompensator) {
        shooterMotor = new MotorEx(hardwareMap, "shooterMotor");
        shooterMotor.setInverted(false);
        shooterMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotor2 = new MotorEx(hardwareMap, "shooterMotor2");
        shooterMotor2.setInverted(true);
        shooterMotor2.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        shooterMotors = new MotorGroup(shooterMotor, shooterMotor2);

        interpLUT = new InterpLUT();

        this.voltageCompensator = voltageCompensator;
        setInterpLUTValues();
    }
    /// Shooter Functions ///
    public void shootRPMs(){
        PIDFController pid = new PIDFController(pShooter, iShooter, dShooter, voltageCompensator.compensateVoltage(fShooter));
        shooterMotors.set(pid.calculate(shooterVel, desiredRPMs));
    }

    public void shootManually(double power){shooterMotors.set(power);}
    public void stop(){shooterMotors.set(0);}

    /// Shooter Getters
    public void getRPMsError(){
        rpmsError = desiredRPMs - ((shooterVel + shooterVel2)/2);
    }
    public void getShooterVel(){
        shooterVel = shooterMotor.getVelocity()/28 * 60;
    }
    public void getInterpLUT(){
        desiredRPMs = interpLUT.get(distance);
    }
    public void getShooterVel2(){
        shooterVel2 = shooterMotor2.getVelocity()/28 * 60;
    }
    public void getGetters(){
        getRPMsError();
        getShooterVel();
        getShooterVel2();
    }

    /// Setters ///
    public void setInterpLUTValues(){
        interpLUT.add(0, 3400);
        interpLUT.add(25, 3400);
        interpLUT.add(35, 3445);
        interpLUT.add(45, 3520);
        interpLUT.add(55, 3800);
        interpLUT.add(65, 3950);
        interpLUT.add(75, 4100);
        interpLUT.add(85, 4200);
        interpLUT.add(95, 4230);
        interpLUT.add(105, 4600);
        interpLUT.add(120, 4725);
        interpLUT.add(130, 4875);

        interpLUT.createLUT();
    }
}
