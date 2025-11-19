#include "Motors.h"

void Motors::commandMotor(std::array<float,4>& motorSpinRateSquare) {


};


std::array<float,4> Motors::getThrustTorques(std::array<float,4>& motorSpinRateSquare) {
  // U2TM = [-kT, -kT, -kT, -kT;
  //          L*kT, -L*kT, -L*kT, L*kT;
  //          L*kT, L*kT, -L*kT, -L*kT;
  //          kM, -kM, kM, -kM];
  float kT = SETUP::kT;
  float kM = SETUP::kM;
  float L = SETUP::L;

  // [T,M] = U2TM * motorSpinRateSquare
  // U2TM is signed, but since this is only used to check deadbands and such, we would be taking magnitude of thrust anyways so make it positive.
  // In LQR formulation, the U2TM matrix keeps the signs as above
  std::array<float,4> ThrustTorques {kT * motorSpinRateSquare[0] + kT * motorSpinRateSquare[1] + kT * motorSpinRateSquare[2] + kT * motorSpinRateSquare[3],
                                     L*kT * motorSpinRateSquare[0] - L*kT * motorSpinRateSquare[1] - L*kT * motorSpinRateSquare[2] + L*kT * motorSpinRateSquare[3],
                                     L*kT * motorSpinRateSquare[0] + L*kT * motorSpinRateSquare[1] - L*kT * motorSpinRateSquare[2] - L*kT * motorSpinRateSquare[3],
                                     kM * motorSpinRateSquare[0] - kM * motorSpinRateSquare[1] + kM * motorSpinRateSquare[2] - kM * motorSpinRateSquare[3]};

  return ThrustTorques
};
