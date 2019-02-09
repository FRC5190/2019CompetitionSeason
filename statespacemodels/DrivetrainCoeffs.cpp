#include "subsystems/\DrivetrainCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 2, 2> MakeDrivetrainPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 0.9888722165795495;
  A(0, 1) = -0.0012093683454447283;
  A(1, 0) = -0.0012093683454447283;
  A(1, 1) = 0.9888722165795494;
  Eigen::Matrix<double, 2, 2> B;
  B(0, 0) = 0.005501552044126173;
  B(0, 1) = 0.0005979090930863556;
  B(1, 0) = 0.0005979090930863556;
  B(1, 1) = 0.005501552044126172;
  Eigen::Matrix<double, 2, 2> C;
  C(0, 0) = 1;
  C(0, 1) = 0;
  C(1, 0) = 0;
  C(1, 1) = 1;
  Eigen::Matrix<double, 2, 2> D;
  D(0, 0) = 0;
  D(0, 1) = 0;
  D(1, 0) = 0;
  D(1, 1) = 0;
  return frc::StateSpacePlantCoeffs<2, 2, 2>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 2, 2> MakeDrivetrainControllerCoeffs() {
  Eigen::Matrix<double, 2, 2> K;
  K(0, 0) = 10.394480594597672;
  K(0, 1) = -0.04039971724685352;
  K(1, 0) = -0.04039971724686938;
  K(1, 1) = 10.394480594597814;
  Eigen::Matrix<double, 2, 2> Kff;
  Kff(0, 0) = 179.5752466586855;
  Kff(0, 1) = -18.630554009000274;
  Kff(1, 0) = -18.630554009000274;
  Kff(1, 1) = 179.57524665868553;
  Eigen::Matrix<double, 2, 1> Umin;
  Umin(0, 0) = -12.0;
  Umin(1, 0) = -12.0;
  Eigen::Matrix<double, 2, 1> Umax;
  Umax(0, 0) = 12.0;
  Umax(1, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 2, 2>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 2, 2> MakeDrivetrainObserverCoeffs() {
  Eigen::Matrix<double, 2, 2> K;
  K(0, 0) = 0.9999000197738087;
  K(0, 1) = -2.390636678447462e-11;
  K(1, 0) = -2.3906366784499987e-11;
  K(1, 1) = 0.9999000197738088;
  return frc::StateSpaceObserverCoeffs<2, 2, 2>(K);
}

frc::StateSpaceLoop<2, 2, 2> MakeDrivetrainLoop() {
  return frc::StateSpaceLoop<2, 2, 2>(MakeDrivetrainPlantCoeffs(),
                                      MakeDrivetrainControllerCoeffs(),
                                      MakeDrivetrainObserverCoeffs());
}
