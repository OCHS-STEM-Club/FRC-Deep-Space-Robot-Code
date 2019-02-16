#include "Lift.hpp"

LiftManager::LiftManager() {
stick = new frc::Joystick{ 0 };
xbox = new frc::XboxController{ 1 };

leftClimber = new WPI_TalonSRX(11);
rightClimber = new WPI_TalonSRX(9);
backClimber = new WPI_TalonSRX(10);

/*
leftClimber->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, K_PID_LOOP_IDX, K_TIMEOUT_MS);
leftClimber->SetSensorPhase(true);
leftClimber->ConfigAllowableClosedloopError(K_PID_LOOP_IDX, 0, K_TIMEOUT_MS);

leftClimber->Config_kP(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS); 
leftClimber->Config_kI(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS);
leftClimber->Config_kD(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS); 
leftClimber->Config_kF(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS); */

verticalClimberSpeed = new double;
rightStick = new double;
leftStick = new double;

leftDistance = new double;
rightDistance = new double;
backDistance = new double;

leftBoost = new double; 
rightBoost = new double;
backBoost = new double;

leftPower = new double;
rightPower = new double;
backPower = new double;

liftLatch = new bool;
liftLatchTwo = new bool;
liftToggle = new int;
backToggle = new bool;
backSetpoint = new double;
*liftLatch = false;
*liftLatchTwo = false;
*liftToggle = 0;
*backToggle = false;

frontToggle = new bool;
leftSetpoint = new double;
rightSetpoint = new double;
*frontToggle = false;

offToggle = new bool;
offLatch = new bool;
*offToggle = true;
*offLatch = false;

leftClimber->SetNeutralMode(Brake);
rightClimber->SetNeutralMode(Brake);
backClimber->SetNeutralMode(Brake);

leftClimber->GetSensorCollection().SetQuadraturePosition(0,10);
rightClimber->GetSensorCollection().SetQuadraturePosition(0,10);
backClimber->GetSensorCollection().SetQuadraturePosition(0,10);
}

void LiftManager::Lift() {
    frc::SmartDashboard::PutNumber("left climber current", leftClimber->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("right climber current", rightClimber->GetOutputCurrent());
    frc::SmartDashboard::PutNumber("back climber current", backClimber->GetOutputCurrent());



    *rightStick = xbox->GetRawAxis(3) * 0.45; //no more than 25%
    *leftStick = xbox->GetRawAxis(2) * 0.45; //no more than 25%

    frc::SmartDashboard::PutNumber("left trigger", xbox->GetRawAxis(2));
    frc::SmartDashboard::PutNumber("right trigger", xbox->GetRawAxis(3));

    if ((xbox->GetRawAxis(3) > 0.1) or (xbox->GetRawAxis(2) > 0.1)) {
        if (*rightStick > *leftStick) {
            *verticalClimberSpeed = *rightStick;
        }
        else if (*leftStick > *rightStick) {
            *verticalClimberSpeed = -*leftStick;
        } 
    }
    else {
        *verticalClimberSpeed = 0;
    }


    frc::SmartDashboard::PutNumber("liftPower", *verticalClimberSpeed);

    *leftDistance = (1.0 * -leftClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *rightDistance = (1.0 * rightClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *backDistance = (1.0 * backClimber->GetSensorCollection().GetQuadraturePosition() / 4096);

    frc::SmartDashboard::PutNumber("leftClimberEncoder", *leftDistance);
    frc::SmartDashboard::PutNumber("rightClimberEncoder", *rightDistance);
    frc::SmartDashboard::PutNumber("backClimberEncoder", *backDistance);


    if (xbox->GetRawButton(6) and !*liftLatch) {
        *liftToggle = *liftToggle + 1;
        *liftLatch = true;
    }
    else if (!xbox->GetRawButton(6) and *liftLatch) {
        *liftLatch = false;
    }

    if (xbox->GetRawButton(5) and !*liftLatchTwo) {
        *liftToggle = *liftToggle - 1;
        *liftLatch = true;
    }
    else if (!xbox->GetRawButton(5) and *liftLatchTwo) {
        *liftLatch = false;
    }

    if (*liftToggle == 4) {
        *liftToggle = 0;
    }
    if (*liftToggle == -1) {
        *liftToggle = 2;
    }


    if (*liftToggle == 0) {

        frc::SmartDashboard::PutString("lift phase", "control of all");
        if (*verticalClimberSpeed > 0) {
            if (*leftDistance > *rightDistance and *leftDistance > *backDistance) {
                *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *leftBoost = 0;
            }
            else if (*rightDistance > *leftDistance and *rightDistance > *backDistance) {
                *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = 0;
            }
            else if (*backDistance > *leftDistance and *backDistance > *leftDistance) {
                *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = 0;
            }

            *leftPower = *leftBoost + *verticalClimberSpeed;
            *rightPower = *rightBoost + *verticalClimberSpeed;
            *backPower = *backBoost + *verticalClimberSpeed;
        }

        if (*verticalClimberSpeed < 0) {
            if (*leftDistance < *rightDistance and *leftDistance < *backDistance) {
                *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *leftBoost = 0;
            }
            else if (*rightDistance < *leftDistance and *rightDistance < *backDistance) {
                *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = 0;
            }
            else if (*backDistance < *leftDistance and *backDistance < *leftDistance) {
                *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = 0;
            }

            *leftPower = *leftBoost + *verticalClimberSpeed;
            *rightPower = *rightBoost + *verticalClimberSpeed;
            *backPower = *backBoost + *verticalClimberSpeed;
        }
    }

 /*   if (*liftToggle == 1) {
        frc::SmartDashboard::PutString("lift phase", "back move");
        if (!*frontToggle) {
            *leftSetpoint = *leftDistance;
            *rightSetpoint = *rightDistance;
            *frontToggle = true;
        }

        *leftPower = (*leftSetpoint - *leftDistance) * 0.55;
        *rightPower = (*rightSetpoint - *rightDistance) * 0.55;

        *backPower = *verticalClimberSpeed * 1.5;
    }
    else if (!(*liftToggle ==1)) {
        *frontToggle = false;
    } */

    if (*liftToggle == 1) {
        frc::SmartDashboard::PutString("lift phase", "middle control");
        if (*verticalClimberSpeed > 0) {
            if (*leftDistance > *rightDistance and *leftDistance > *backDistance) {
                *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *leftBoost = 0;
            }
            else if (*rightDistance > *leftDistance and *rightDistance > *backDistance) {
                *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = 0;
            }
            else if (*backDistance > *leftDistance and *backDistance > *leftDistance) {
                *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = 0;
            }

            *leftPower = *leftBoost + *verticalClimberSpeed;
            *rightPower = *rightBoost + *verticalClimberSpeed;
            *backPower = *backBoost + *verticalClimberSpeed;
        }

        if (*verticalClimberSpeed < 0) {
            if (*leftDistance < *rightDistance) {
                *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *leftBoost = 0;
            }
            else if (*rightDistance < *leftDistance) {
                *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = 0;
            }

            *leftPower = (*leftBoost + *verticalClimberSpeed) * 0.8;
            *rightPower = (*rightBoost + *verticalClimberSpeed) * 0.8;
        }

        if (*verticalClimberSpeed > -0.05) {
            *leftBoost = 0;
            *rightBoost = 0;
        }

        if (!*backToggle) {
            *backSetpoint = *backDistance;
            *backToggle = true;
        }

        *backPower = (*backSetpoint - *backDistance) * 0.45;
    }
    if (!(*liftToggle == 1)) {
        *backToggle = false;
    }

    if (*liftToggle == 2) {
        frc::SmartDashboard::PutString("lift phase", "back control");
        *backPower = *verticalClimberSpeed * 1.6;
        *leftPower = 0;
        *rightPower = 0;
    }

    frc::SmartDashboard::PutNumber("lift number", *liftToggle);


    if (xbox->GetRawButton(4)) {
        *leftPower = ((LEFT_SETPOINT - *leftDistance) / 4.0);
        *rightPower = ((RIGHT_SETPOINT - *rightDistance) / 4.0);
        *backPower = ((BACK_SETPOINT - *backDistance) / 4.0);

        if (((*leftDistance + *rightDistance + *backDistance) / 3.0) > 5) {
            if (*leftDistance > *rightDistance and *leftDistance > *backDistance) {
                *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *leftBoost = 0;
            }
            else if (*rightDistance > *leftDistance and *rightDistance > *backDistance) {
                *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = 0;
            }
            else if (*backDistance > *leftDistance and *backDistance > *leftDistance) {
                *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
                *backBoost = 0;
            }
        }
        else {
            *leftBoost = 0;
            *rightBoost = 0;
            *backBoost = 0;
        }

        *leftPower = *leftBoost + *leftPower;
        *rightPower = *rightBoost + *rightPower;
        *backPower = *backBoost + *backPower;

        if (*leftPower > 0.5) {
            *leftPower = 0.5;
        }
        if (*rightPower > 0.5) {
            *rightPower = 0.5;
        }
        if (*backPower > 0.5) {
            *backPower = 0.5;
        }
    }




    if (xbox->GetRawButton(8) and !*offLatch) {
        *offToggle = !*offToggle;
        *offLatch = true;
    }
    else if (!xbox->GetRawButton(8) and *offLatch) {
        *offLatch = false;
    }

    if (*offToggle) {
        
    }
    else if (!*offToggle) {
        *leftPower = 0;
        *rightPower = 0;
        *backPower = 0;
    }

    frc::SmartDashboard::PutBoolean("lift on?", *offToggle);


    leftClimber->Set(-*leftPower); 
    rightClimber->Set(*rightPower);
    backClimber->Set(*backPower); 
 
  /*  leftClimber->Set(-*verticalClimberSpeed);
    rightClimber->Set(*verticalClimberSpeed);
    backClimber->Set(*verticalClimberSpeed); */

    //frc::SmartDashboard::PutNumber("leftEnc", leftClimber->GetSensorCollection().GetQuadraturePosition());

//leftClimber->GetSensorCollection().IsFwdLimitSwitchClosed();
}