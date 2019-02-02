#include "Vision.hpp"

PixyManager::PixyManager () {
  driveManager = new DriveManager();

  I2CPixy = new frc::I2C(frc::I2C::Port::kOnboard, I2C_ADDRESS);
  stick = new frc::Joystick{ 0 };

  target = 170;    

    //Gyro
    try {
        ahrs = new AHRS(SPI::Port::kMXP);
    }
    catch(std::exception ex) {
        std::string err_string = "Error initalizing naxX-MXP"; 
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }

  revoultions = 0;
}

void PixyManager::pixy() {
  byte buff[31];
  I2CPixy->Read(I2C_ADDRESS, 31, buff);


  if (!(buff[1] == buff[2])) {
    if (buff[0] == 0) {
      for(int i = 0; i < 30; i++) {
        buff[i] = buff[i + 1];
      }
    }
  }

  for(int i = 0; i < 15; i++) {
    translate[i] = buff[(2 * i) + 1] * 256 + buff[2 * i];
  }

  if (translate[0] == 43605 && translate[1] == 43605 && translate[8] == 43605) {
    Pixyx1 = translate[4];
    Pixyy1 = translate[5];
    Pixyw1 = translate[6];
    Pixyh1 = translate[7];

    Pixyx2 = translate[11];
    Pixyy2 = translate[12];
    Pixyw2 = translate[13];
    Pixyh2 = translate[14];

    goodTargets = true; 
    frc::SmartDashboard::PutString("pixyTargets", "two targets");
  }
  else if (translate[0] == 0 && translate[1] == 0 && translate[8] == 0) { 
    frc::SmartDashboard::PutString("pixyTargets", "no target");
    goodTargets = false;

/*
    Pixyx1 = 0;
    Pixyy1 = 0;
    Pixyw1 = 0;
    Pixyh1 = 0;

    Pixyx2 = 0;
    Pixyy2 = 0;
    Pixyw2 = 0;
    Pixyh2 = 0;
    */
  }
  else if (translate[0] == 43605 && translate[1] == 43605 && translate[8] == 0) {
    frc::SmartDashboard::PutString("pixyTargets", "one target");
    goodTargets = false; 

/*
    Pixyx1 = translate[4];
    Pixyy1 = translate[5];
    Pixyw1 = translate[6];
    Pixyh1 = translate[7];

    Pixyx2 = translate[11];
    Pixyy2 = translate[12];
    Pixyw2 = translate[13];
    Pixyh2 = translate[14];
    */
  }

  frc::SmartDashboard::PutNumber("Pixyx1", Pixyx1);
  frc::SmartDashboard::PutNumber("Pixyy1", Pixyy1);
  frc::SmartDashboard::PutNumber("Pixyw1", Pixyw1);
  frc::SmartDashboard::PutNumber("Pixyh1", Pixyh1);

  frc::SmartDashboard::PutNumber("Pixyx2", Pixyx2);
  frc::SmartDashboard::PutNumber("Pixyy2", Pixyy2);
  frc::SmartDashboard::PutNumber("Pixyw2", Pixyw2);
  frc::SmartDashboard::PutNumber("Pixyh2", Pixyh2);

//removal of size code for tracking
  bigSize = Pixyw1 * Pixyh1;
  smallSize = Pixyw2 * Pixyh2;

  frc::SmartDashboard::PutNumber("PixyArea1", bigSize);
  frc::SmartDashboard::PutNumber("PixyArea2", smallSize); 
  frc::SmartDashboard::PutNumber("areaDifference", (bigSize - smallSize));

 
 
  frc::SmartDashboard::PutNumber("pixy0", translate[0]);
  frc::SmartDashboard::PutNumber("pixy1", translate[1]);
  frc::SmartDashboard::PutNumber("pixy2", translate[2]);
  frc::SmartDashboard::PutNumber("pixy3", translate[3]);
  frc::SmartDashboard::PutNumber("pixy4", translate[4]);
  frc::SmartDashboard::PutNumber("pixy5", translate[5]);
  frc::SmartDashboard::PutNumber("pixy6", translate[6]);
  frc::SmartDashboard::PutNumber("pixy7", translate[7]);
  frc::SmartDashboard::PutNumber("pixy8", translate[8]);
  frc::SmartDashboard::PutNumber("pixy9", translate[9]);
  frc::SmartDashboard::PutNumber("pixy10", translate[10]);
  frc::SmartDashboard::PutNumber("pixy11", translate[11]);
  frc::SmartDashboard::PutNumber("pixy12", translate[12]);
  frc::SmartDashboard::PutNumber("pixy13", translate[13]);
  frc::SmartDashboard::PutNumber("pixy14", translate[14]);
  frc::SmartDashboard::PutNumber("pixy15", translate[15]); 

  pixyDistanceBetweenTargets = abs(Pixyx1-Pixyx2);
  frc::SmartDashboard::PutNumber("pixyDistanceBetweenTargets", pixyDistanceBetweenTargets);

  angle = ahrs->GetAngle();
  
  if (revoultions >= 1) {
    angle = angle - (revoultions * 360);
  }
  
  if (revoultions <= 0) {
    angle = angle + abs(revoultions * 360);
  }

  if ((angle > 360) or (angle < 0)) {
    if (angle > 360) {
      revoultions++;
    }

    if (angle < 360) {
      revoultions--;
    }

    angle = ahrs->GetAngle();

    if (revoultions >= 1) {
      angle = angle - (revoultions * 360);
    }
  
    if (revoultions <= 0) {
      angle = angle + abs(revoultions * 360);
    }
  }

  frc::SmartDashboard::PutNumber("gyro 0 - 360", angle);
  
}

void PixyManager::pixyFunct() {
  distanceToIdealCenter = ((1.0 * Pixyx1 + Pixyx2) / 2) - PIXY_CENTER_X;

  strafeCorrectionToIdealCenter = (distanceToIdealCenter/125) * 0.85;

  if (strafeCorrectionToIdealCenter > ANTI_MISSILE_CODE) {
    strafeCorrectionToIdealCenter = ANTI_MISSILE_CODE;
  }

  if (strafeCorrectionToIdealCenter < -ANTI_MISSILE_CODE) {
    strafeCorrectionToIdealCenter = -ANTI_MISSILE_CODE;
  }

  //xCenter is the center of the 2 x points in relation to the current screen pixels
  xCenter = (1.0 * Pixyx1 + Pixyx2) / 2;
  // if (((PIXY_CENTER_X - PIXY_DEADBAND_X) < xCenter)  &&  ((PIXY_CENTER_X + PIXY_DEADBAND_X) > xCenter)) {
  //   strafeCorrectionToIdealCenter = 0;
  // }

  if(abs(PIXY_CENTER_X-xCenter) < PIXY_DEADBAND_X) {
    strafeCorrectionToIdealCenter = 0;
    frc::SmartDashboard::PutBoolean("deadbandTest", true);
  }
  else {
    frc::SmartDashboard::PutBoolean("deadbandTest", false);
  }

//turn correction 
/*
  if ((Pixyx1-Pixyx2) < 0) {
    leftTargetBig = false;
  }
  else if ((Pixyx1-Pixyx2) > 0) {
    leftTargetBig = true; 
  }

  if (leftTargetBig) {
    turnOffset = smallSize - bigSize;
  }
  else if (!leftTargetBig) {
    turnOffset = bigSize - smallSize;
  }

  frc::SmartDashboard::PutNumber("TurnOffset", turnOffset);

  turnWant = ((1.0 * turnOffset/600) * 0.635);

//outside deadband (max value)
  if (turnWant > PIXY_DEADBAND_TURN) {
    turnWant = PIXY_DEADBAND_TURN;
  }

  if (turnWant < -PIXY_DEADBAND_TURN) {
    turnWant = -PIXY_DEADBAND_TURN;
  }

  if ((abs(turnOffset) < 100)) {
    turnWant = 0;
  }
*/

if (angle < 180) {
  turnWant = 90;
}

if (angle > 180) {
  turnWant = 270; 
}

turnOffset = turnWant - angle; 
turnCorrection = (1.0 * turnOffset/180) * 0.65;

frc::SmartDashboard::PutNumber("turnOffset", turnOffset);
if (abs(turnOffset) < PIXY_DEADBAND_TURN) {
  turnCorrection = 0;
}


  pixyDistanceCorrection = PIXY_DISTANCE_X - pixyDistanceBetweenTargets;

  driveCorrection = (1.0 * pixyDistanceCorrection / 120) * 0.95;

  if (driveCorrection > PIXY_DEADBAND_DISTANCE) {
    driveCorrection = PIXY_DEADBAND_DISTANCE;
  }

  if (driveCorrection < -PIXY_DEADBAND_DISTANCE) {
    driveCorrection = -PIXY_DEADBAND_DISTANCE;
  }

  if ((abs(pixyDistanceCorrection) < 20)) {
    driveCorrection = 0;
  }


  frc::SmartDashboard::PutNumber("turnCorrection", turnCorrection);
  frc::SmartDashboard::PutNumber("strafeCorrection", strafeCorrectionToIdealCenter);
  frc::SmartDashboard::PutNumber("driveCorrection", driveCorrection);

  if (goodTargets) {
    //driveManager->control(strafeCorrectionToIdealCenter ,strafeCorrectionToIdealCenter, driveCorrection);
    driveManager->control(turnCorrection, strafeCorrectionToIdealCenter, 0);
  }
  else {
    driveManager->control(0, 0, 0);
  }
  
}