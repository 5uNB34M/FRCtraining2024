package frc.robot.subsystems;

import javax.swing.text.Position;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.ModuleConstants;

public class MAXSwereveModule {
    private final CANSparkMax kTurningMAX;
    private final CANSparkMax kDrivingMAX;

    private final RelativeEncoder kDriveEncoder;
    private final AbsoluteEncoder kTurningEncoder;

    private final SparkPIDController kTurningPID;
    private final SparkPIDController kDrivingPID;

    private double chassisoffset = 0.0;
    private SwerveModuleState targetstate = new SwerveModuleState();

    public MAXSwereveModule(int turningCANid, int drivingCANid, double angleOffset) {
        kTurningMAX = new CANSparkMax(turningCANid, MotorType.kBrushless);
        kDrivingMAX = new CANSparkMax(drivingCANid, MotorType.kBrushless);

        kDrivingMAX.restoreFactoryDefaults();
        kTurningMAX.restoreFactoryDefaults();
        
        kDriveEncoder = kDrivingMAX.getEncoder();
        kTurningEncoder = kTurningMAX.getAbsoluteEncoder(Type.kDutyCycle);
        kTurningPID = kTurningMAX.getPIDController();
        kDrivingPID = kDrivingMAX.getPIDController();
        kTurningPID.setFeedbackDevice(kTurningEncoder);
        kTurningPID.setFeedbackDevice(kDriveEncoder);

        kDriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        kDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        kTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        kTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        kTurningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        kTurningPID.setPositionPIDWrappingEnabled(true);
        kTurningPID.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        kTurningPID.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        kDrivingPID.setP(ModuleConstants.kDrivingP);
        kDrivingPID.setI(ModuleConstants.kDrivingI);
        kDrivingPID.setD(ModuleConstants.kDrivingD);
        kDrivingPID.setFF(ModuleConstants.kDrivingFF);
        kDrivingPID.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        kTurningPID.setP(ModuleConstants.kTurningP);
        kTurningPID.setI(ModuleConstants.kTurningI);
        kTurningPID.setD(ModuleConstants.kTurningD);
        kTurningPID.setFF(ModuleConstants.kTurningFF);
        kTurningPID.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        kDrivingMAX.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        kTurningMAX.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        kDrivingMAX.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        kTurningMAX.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        kDrivingMAX.burnFlash();
        kTurningMAX.burnFlash();

        chassisoffset = angleOffset;
        targetstate.angle = new Rotation2d(kTurningEncoder.getPosition());
        kDriveEncoder.setPosition(0);
        
    }
}
