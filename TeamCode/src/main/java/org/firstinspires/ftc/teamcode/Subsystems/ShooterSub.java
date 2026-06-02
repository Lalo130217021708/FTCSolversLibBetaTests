package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Camera.Limelight.distance;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.configurableRPMs;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.d;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.f;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.i;
import static org.firstinspires.ftc.teamcode.Configurations.ConfigurableVariables.shooterConfigurableVariables.p;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.ControlSystems.VoltageCompensator;

public class ShooterSub {
    private final MotorEx shooterMotor, shooterMotor2;
    public static double shooterCPR, shooterPos, shooterRate, shooterVel, shooterVel2, rpmsError;
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
        PIDFController pid = new PIDFController(p, i, d, voltageCompensator.compensateVoltage(f));
        shooterMotors.set(pid.calculate(shooterVel, desiredRPMs));
    }
    public void shootManually(double power){shooterMotors.set(power);}
    public void stop(){shooterMotors.set(.4);}

    /// Shooter Getters
    public void getRPMsError(){
        rpmsError = desiredRPMs - ((shooterVel + shooterVel2)/2);
    }
    public void getShooterMotorCPR(){shooterCPR = shooterMotor.getCPR();}
    public void getShooterPos(){shooterPos = shooterMotor.getCurrentPosition() / shooterMotor.getCPR();}
    public void getShooterRate(){shooterRate = shooterMotor.getRate();}
    public double getShooterVel(){
        shooterVel = shooterMotor.getVelocity()/28 * 60;
        return shooterVel;
    }
    public void getInterpLUT(){
        desiredRPMs = interpLUT.get(distance);
    }
    public void getShooterVel2(){
        shooterVel2 = shooterMotor2.getVelocity()/28 * 60;
    }
    public void getGetters(){
        getRPMsError();
        getShooterMotorCPR();
        getShooterPos();
        getShooterRate();
        getShooterVel();
        getShooterVel2();
    }

    /// Setters ///
    public void setInterpLUTValues(){
        interpLUT.add(0, 3085);
        interpLUT.add(25, 3085);
        interpLUT.add(30, 3170);
        interpLUT.add(35, 3270);
        interpLUT.add(40, 3370);
        interpLUT.add(45, 3470);
        interpLUT.add(50, 3550);
        interpLUT.add(55, 3650);
        interpLUT.add(60, 3750);
        interpLUT.add(65, 3820);
        interpLUT.add(70, 3920);
        interpLUT.add(75, 3930);
        interpLUT.add(80, 3945);
        interpLUT.add(85, 3985);
        interpLUT.add(90, 4040);
        interpLUT.add(95, 4120);
        interpLUT.add(100, 4250);
        interpLUT.add(105, 4300);
        interpLUT.add(110, 4370);

        interpLUT.createLUT();
    }
}
