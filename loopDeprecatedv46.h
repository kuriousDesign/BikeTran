void loopDeprecated()
{
  // SerialLogging::info("Dimitri loop() started");

  timeNowUs = micros();
  if ((timeNowUs - lastUpdateScanUs) >= UPDATE_TIME_US)
  {
    updateMotors();
    Logger::process();
    //Serial.println(timeNowUs - lastUpdateScanUs);
    lastUpdateScanUs = timeNowUs;
  }
  else if (timeNowUs - lastLoopUpdateScanUs > (NUM_MOTORS - 1) * UPDATE_TIME_US && motorId == 0)
  {
    loopState.run();
    // lastUpdateUs = timeNowUs;
    //Serial.println(timeNowUs - lastLoopUpdateScanUs);
    lastLoopUpdateScanUs = timeNowUs;

    readInputs();
    int shiftTypeReq = checkForGearShiftRequests();
    // timeNow = millis();

    // detect resetRequest
    if (inputs[Inputs::ShiftDownSw] && inputs[Inputs::ShiftUpSw])
    {
      resetReq = true && !prevResetReq;
      prevResetReq = true;
    }
    else
    {
      resetReq = false;
      prevResetReq = false;
    }

    // AUTO ABORT IF ERROR
    if (errors.present && loopState.Step >= int(Modes::RESETTING))
    {
      loopState.transitionToStep(Modes::ABORTING);
    }
    else if (resetReq && loopState.Step == int(Modes::IDLE))
    {
      // iSerial.setNewMode(Modes::RESETTING);
    }

    switch (loopState.Step)
    {
    case Modes::ABORTING:
      loopState.StepDescription("ABORTING - turning all motors off");
      turnAllOff();
      loopState.transitionToStep(Modes::KILLED);
      break;
    case Modes::KILLED:
      loopState.StepDescription("KILLED - all motors off");
      if (loopState.FirstScan)
      {
        turnAllOff();
      }

      if (errors.present)
      {
        loopState.transitionToStep(Modes::ERROR);
      }
      else
      {
        loopState.transitionToStep(Modes::INACTIVE);
      }

      break;
    case Modes::ERROR:
      loopState.StepDescription("ERROR - handling error state");
      shiftData.targetGear = 0;
      isHomed = false;
      if (loopState.FirstScan)
      {
        turnAllOff();
      }
      else
      {
        // auto clear the errors after 5 seconds
        if (resetReq)
        {
          clearErrors();
        }
        if (!errors.present)
        {
          // iSerial.setNewMode(Modes::INACTIVE);
          loopState.transitionToStep(Modes::INACTIVE);
        }
        if (loopState.getStepActiveTime() > 3000 && false)
        {
          Logger::error("ERROR: %s", errors.list[0].c_str());
        }
      }

      break;
    case Modes::INACTIVE:
      loopState.StepDescription("INACTIVE - motors off, waiting for reset");
      // turnAllOff();
      if (OPERATING_MODE == OperatingModes::AUTO || OPERATING_MODE == OperatingModes::MANUAL_CLUTCH_ENGAGE)
      {
        if (!AUTO_RESET)
        {
          // iSerial.debugPrintln("Press both shift buttons to reset");
        }

        if (AUTO_RESET || (resetReq))
        {
          loopState.transitionToStep(Modes::RESETTING);
        }
      }
      else
      {
        loopState.transitionToStep(Modes::MANUAL);
      }
      break;

    case Modes::RESETTING:
      loopState.StepDescription("RESETTING - performing homing routines");

      // skip homing if already homed
      if (isHomed)
      {
        // SerialLogging::info("RESETTING: already homed, skipping homing routine");
        if (enableAllMotors())
        {
          loopState.transitionToStep(Modes::IDLE);
        }
      }
      else
      {
        // SerialLogging::info("RESETTING: running homing routines");
        //  runHomingRoutineClutchMotor(true);
        // motors[Motors::CLUTCH].setDebug(true);
        loopState.transitionToStep(51);
      }
      break;

    case 51:
      loopState.StepDescription("RESETTING - requesting clutch homing process");
      // Initialize any variables or states needed before starting homing
      // motors[Motors::CLUTCH].requestProcess(Motor::MotorProcesses::HOME);
      if (loopState.FirstScan)
      {
        motors[Motors::CLUTCH].requestProcess(Motor::MotorProcesses::HOME);
      }
      else if (motors[Motors::CLUTCH].ActiveProcess == Motor::MotorProcesses::HOME && !motors[Motors::CLUTCH].isHomed)
      {
        loopState.transitionToStep(53);
      }
      else
      {
        triggerError("RESETTING: request clutch motor homing was rejected");
      }
      break;

    case 53:
      loopState.StepDescription("Waiting for clutch motor to finish homing");
      // Wait for clutch motor to reach disengaged position
      // SerialLogging::info("Clutch Motor State: %d", motors[Motors::CLUTCH].getState());
      if (motors[Motors::CLUTCH].ActiveProcess == Motor::MotorProcesses::NONE_PROCESS && motors[Motors::CLUTCH].isHomed)
      {
        if (OPERATING_MODE == OperatingModes::MANUAL_CLUTCH_ENGAGE)
        {
          loopState.transitionToStep(Modes::MANUAL);
        }
        else
        {
          loopState.transitionToStep(55);
        }
      }
      break;

    case 55:
      loopState.StepDescription("RESETTING - moving clutch to engaged pedaling position");
      if (loopState.FirstScan)
      {
        // Start moving clutch to engaged position
        motors[Motors::CLUTCH].moveAbs(POSITION_CLUTCH_PEDALING);
      }

      if (motors[Motors::CLUTCH].getState() == Motor::States::MOVING)
      {
        loopState.transitionToStep(56);
      }
      break;

    case 56:
      loopState.StepDescription("Waiting for clutch motor to reach pedaling position");
      // Wait for clutch motor to reach engaged position
      if (motors[Motors::CLUTCH].atPositionAndStill && motors[Motors::CLUTCH].getState() == Motor::States::IDLE)
      {
        setClutchRolloverPosition();
        loopState.transitionToStep(60);
      }

      else if (loopState.getStepActiveTime() > 5000)
      {
        triggerError("RESETTING: Clutch motor did not reach pedaling position in time");
      }
      break;

    case 60:
      loopState.StepDescription("RESETTING - running homing routine for linear motors");
      if (loopState.FirstScan)
      {
        runHomingRoutineLinearMotors(true);
      }
      else if (runHomingRoutineLinearMotors())
      {
        isHomed = true;
        loopState.transitionToStep(Modes::IDLE);
      }

      break;

    case Modes::IDLE:
      loopState.StepDescription("IDLE - waiting for gear change request");
      if (gearChangeReq)
      {
        processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
        // shiftStartTimeMs = timeNow;
        loopState.transitionToStep(Modes::SHIFTING);
      }
      else if (!atTarget)
      {
        // triggerError(Errors::MOTOR_NOT_AT_TARGET_WHILE_IDLE);
        Logger::info("linear motors not at target while in idle, adjust position now");
        loopState.transitionToStep(Modes::SHIFTING);
      }
      else if (resetReq)
      {
        loopState.transitionToStep(Modes::RESETTING);
      }

      break;

    case Modes::SHIFTING:
      loopState.StepDescription("SHIFTING - processing gear shift");
      if (loopState.FirstScan)
      {
        disengageClutch(true);
      }
      else
      {
        if (disengageClutch())
        {
          moveLinearMotorsToGear(shiftData.targetGear, true);
          loopState.transitionToStep(210);
        }
      }
      break;

    case 210:
      loopState.StepDescription("SHIFTING - moving linear motors to target gear");
      disengageClutch();

      if (moveLinearMotorsToGear(shiftData.targetGear) && atTarget)
      {
        loopState.transitionToStep(220);
      }
      else if (gearChangeReq)
      {
        processShiftReqNum(shiftTypeReq, shiftTargetGearParam);
        // iSerial.resetModeTime();
        // loopState.transitionToStep(1);
      }

      break;

    case 220:
      loopState.StepDescription("SHIFTING - engaging clutch");
      if (loopState.FirstScan)
      {
        // digitalWrite(PIN_CLUTCH_SOL, !SOL_ON);
        engageClutch(true);
        // reset mode time
      }
      else if (engageClutch())
      {
        // shiftTimeMs = timeNow - shiftStartTimeMs;
        loopState.transitionToStep(Modes::IDLE);
      }
      else if (loopState.getStepActiveTime() > 4000)
      {
        triggerError("SHIFTING: TOGGLE_MOTOR_ENGAGE_MOVE_TIMED_OUT");
      }

      break;

    case Modes::MANUAL:
      if (loopState.FirstScan)
      {
        disengageClutch(true);
      }

      switch (OPERATING_MODE)
      {
      case OperatingModes::MANUAL_CLUTCH_JOGGING:
        loopState.StepDescription("MANUAL - clutch jogging - use up/down");
        runMotorManualMode(Motors::CLUTCH);
        break;
      case OperatingModes::MANUAL_CLUTCH_ENGAGE:
        if (inputs[Inputs::ShiftUpSw])
        {
          disengageClutch(true);
          loopState.transitionToStep(1150);
        }
        break;
      case OperatingModes::MANUAL_LINEAR_P:
        loopState.StepDescription("MANUAL - linear P jogging - use up/down");
        runMotorManualMode(Motors::LINEAR_P);
        break;
      case OperatingModes::MANUAL_LINEAR_S:
        loopState.StepDescription("MANUAL - linear S jogging - use up/down");
        runMotorManualMode(Motors::LINEAR_S);
        break;
      }

      break;
    case 1150:
      loopState.StepDescription("waiting for clutch disengage");
      if (disengageClutch() && inputs[Inputs::ShiftUpSw])
      {
        loopState.transitionToStep(1151);
      }
      break;

      case 1151:
      loopState.StepDescription("holding at disengage");
      if (loopState.getStepActiveTime() > 2000)
      {
        engageClutch(true);
        loopState.transitionToStep(1155);
      }

      break;

    case 1155:
      loopState.StepDescription("waiting for clutch engage");
      if (engageClutch())
      {
        loopState.transitionToStep(1156);
      }
      break;

      case 1156:
      loopState.StepDescription("re-engaged");
      if (loopState.getStepActiveTime() > 2000)
      {
        disengageClutch(true);
        loopState.transitionToStep(Modes::MANUAL);
      }
      break;

    default:
      Logger::info(("DEBUG: UNRECOGNIZED Loop Step: " + String(loopState.Step)).c_str());
      break;
    }
  

    checkActualGear();
    updateGearNumberDigitalOutputs(shiftData.targetGear);
    gearChangeReq = false;

    if (DIAGNOSTIC_MODE == DiagnosticModes::UI && timeNowUs/1000 - lastUiUpdateMs > UI_UPDATE_RATE_MS)
    {
      updatePublishedDataChunk();
      uint8_t id = 0;
      lastUiUpdateMs = timeNowUs/1000;
      //uint8_t dataLength = (MOTOR_DATA_SIZE * NUM_MOTORS + 2);
      Logger::publishData(publishedData, PACKET_SIZE, id);
    }
    Logger::process();
  }
  else
  {
    Logger::process();
    
  }
}
