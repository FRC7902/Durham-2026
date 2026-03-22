// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.LimelightWrapper;
import limelight.Limelight;
import swervelib.SwerveDrive;

public class LimeLightChooser extends SubsystemBase {
  LimelightWrapper limelightWrapperA;
  LimelightWrapper limelightWrapperB;

  /** Creates a new LimeLightChooser. */
  public LimeLightChooser(LimelightWrapper A, LimelightWrapper B) {
    this.limelightWrapperA = A;
    this.limelightWrapperB = B;
  }

  public int prioritizeLimelight() {
    Matrix matrixA = limelightWrapperA.getEstimationStdDevsLimelightMT2(limelightWrapperA.poseEstimateMt2, true);
    Matrix matrixB = limelightWrapperB.getEstimationStdDevsLimelightMT2(limelightWrapperB.poseEstimateMt2, true);

    double[] DeltaStd = {
      matrixA.get(0, 0) - matrixB.get(0,0), 
      matrixA.get(0,2) - matrixB.get(0,2), 
      matrixA.get(0,2) - matrixB.get(0,2)
    };

    int diff = 0;
    for (int i = 0; i < DeltaStd.length; i++){
      if (DeltaStd[i] > 0){
        diff--;
      } else if (DeltaStd[i] < 0){
        diff++;
      }
    }
    //returns 1 if use A
    if (diff > 1){
      return 1;
    }
    //returns -1 if use B
    else if (diff < -1){
      return -1;
    }
    //returns 0 if use both
    else{
      return 0;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
