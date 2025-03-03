/************************************************************/
/*    NAME: Douglas Lima, Eduardo Eiras                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: lanchaPID.cpp                                        */
/*    DATE: September, 2023                          */
/************************************************************/

#include "MBUtils.h"
#include "ACTable.h"
#include "lanchaPID.h"
//#include "PIDParameterReader.h"
#include <cmath>
#include <string>
#include <vector>
#include <iterator>

using namespace std;

//Variáveis globais
double nav_heading;  // Rumo atual do navio
double desired_heading;  // Rumo desejado do navio
double nav_speed;  // Velocidade atual do navio
double desired_speed;  // Velocidade desejada do navio

double desired_thrust;  // Saída do controle de velocidade
double desired_rudder;  // Saída do controle de rumo

double speed_kp = 0.0; //kp da veloc
double speed_ki = 0.0; //ki da veloc
double speed_kd = 0.0; //kd da veloc
int thrust_map = 1;  // set to 1 to use thrust_map speed control

double heading_kp = 0.9; //kp do heading - Modified value for USV
double heading_ki = 0.0; //ki do heading
double heading_kd = 0.2; //kd do heading

double dt = 0.0;
std::string deploy; //deploy
std::string moos_manual_overide;
std::string moos_manual_override;

std::string constant_heading; //Variável para verificar se o rumo constante é verdadeiro
double setpoint_heading; //Variável em que eu coloco o setpoint de rumo

//Cria os objetos para o controle PID com valores iniciais de kp, ki e kd lidos do arquivo
//PIDController speedController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/speed_pid_parameters.txt");
//PIDController headingController("/home/dueiras/vsnt/moos-ivp-vsnt/src/planchaPID/heading_pid_parameters.txt");
PIDController speedController(speed_kp, speed_ki, speed_kd);
PIDController headingController(heading_kp, heading_ki, heading_kd);

//---------------------------------------------------------
// Constructor()

PIDController::PIDController(double kp, double ki, double kd)
{
  kp          = kp;
  ki          = ki;
  kd          = kd;
  upper_bound = 40; //Modified value for USV
  lower_bound = -40;
  integral    = 0;
  prevError   = 0;

}

//---------------------------------------------------------
// Calculate()

double PIDController::Calculate(double desired, double current, double dt) {
    double error = desired - current;

    //Passando para a escala de -180 a 180
    // Isso não é o ideal para o controlador de velocidade
    // Porém hoje os valores não chegam a -180 ou 180
    if (error < -180){
      error += 360;
    } else if (error > 180){
      error -= 360;
    }

    integral += error*dt; //Multiplico pelo período dt de tempo

    // 25-set-2023: add Anti-WindUp
    // Limits integral to maximum and minimum output values
    if (integral > upper_bound) {
      integral = upper_bound;
    }
    else if (integral < lower_bound) {
      integral = lower_bound;
    }

    double derivative = (error - prevError)/dt;

    double output = kp * error + ki * integral + kd * derivative;

    // Limits the output value
    if (output > upper_bound)
      output = upper_bound;
    else if (output < lower_bound)
      output = lower_bound;

    prevError = error;

    return output;
} // Calculate()

//---------------------------------------------------------
// setKP()

void PIDController::setKP(double KP) {
  kp = KP;
} // setKP()

//---------------------------------------------------------
// setKI()

void PIDController::setKI(double KI) {
  ki = KI;
  integral = 0;
} // setKI()

//---------------------------------------------------------
// setKD()

void PIDController::setKD(double KD) {
  kd = KD;
} // setKD()

//---------------------------------------------------------
// setUpper()

void PIDController::setUpper(double Upper) {
  upper_bound = Upper;
} // setUpper()

//---------------------------------------------------------
// setLower()

void PIDController::setLower(double Lower) {
  lower_bound = Lower;
} // setKD()

//---------------------------------------------------------
// getKP()

double PIDController::getKP() {
  return kp;
} // getKP()

//---------------------------------------------------------
// getKI()

double PIDController::getKI() {
  return ki;
} // getKI()

//---------------------------------------------------------
// getIterm()

double PIDController::getIterm() {
  return integral;
} // getIterm()

//---------------------------------------------------------
// getKD()

double PIDController::getKD() {
  return kd;
} // getKD()

//---------------------------------------------------------
// resetIntegral()

void PIDController::resetIntegral() {
  integral = 0.0;
} // resetIntegral()

//---------------------------------------------------------
// Constructor()

lanchaPID::lanchaPID()
{
}

//---------------------------------------------------------
// Destructor

lanchaPID::~lanchaPID()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool lanchaPID::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_HEADING") 
       nav_heading = msg.GetDouble();
     else if(key == "DESIRED_HEADING") 
       desired_heading = msg.GetDouble();
     else if(key == "NAV_SPEED") 
       nav_speed = msg.GetDouble();
     else if(key == "DESIRED_SPEED") 
       desired_speed = msg.GetDouble();  
     else if(key == "SPEED_KP") 
       speedController.setKP(msg.GetDouble());
     else if(key == "SPEED_KI") 
       speedController.setKI(msg.GetDouble());
     else if(key == "SPEED_KD") 
       speedController.setKD(msg.GetDouble());
     else if(key == "HEADING_KP") 
       headingController.setKP(msg.GetDouble());
     else if(key == "HEADING_KI") 
       headingController.setKI(msg.GetDouble());
     else if(key == "HEADING_KD") 
       headingController.setKD(msg.GetDouble());
     else if(key == "DEPLOY") {
       deploy = msg.GetString(); 
       // Reset PID error  
       headingController.resetIntegral();
       speedController.resetIntegral();
       }
     else if(key == "MOOS_MANUAL_OVERIDE") 
       moos_manual_overide = msg.GetString();  
     else if(key == "MOOS_MANUAL_OVERRIDE") 
       moos_manual_override = msg.GetString();  
     else if(key == "CONSTANT_HEADING") 
       constant_heading = msg.GetString();  
     else if(key == "SETPOINT_HEADING") 
       setpoint_heading = msg.GetDouble();  
    
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool lanchaPID::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool lanchaPID::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // Loop de controle
  
  // Calcule o controle de velocidade 
  // NÃO ESTÁ SENDO USADO
  if (thrust_map == 1) 
  {
    switch ((int) desired_speed) {
      case 0:
        desired_thrust = 0;
        break;
      case 1:
        desired_thrust = 20;
        break;
      case 4:
        desired_thrust = 34;
        break;
      case 5:
        desired_thrust = 44;
        break;
      case 6:
        desired_thrust = 48;
        break;
      default:
        desired_thrust = 0;
    } // end switch

  } else {
    desired_thrust = speedController.Calculate(desired_speed, nav_speed,dt);
  }

  // Calcule o controle de rumo
  //Caso o rumo constante esteja ativado o setpoint vai ser esse rumo
  if (constant_heading == "true"){
    desired_rudder = headingController.Calculate(setpoint_heading, nav_heading,dt);
  } else {
    desired_rudder = headingController.Calculate(desired_heading, nav_heading,dt);
  }

  // Imprima as saídas dos controladores

  //Publico DESIRED_RUDDER e DESIRED_THRUST
  //Só publica quando deploy for true e moos_manual_override for false
  if (deploy == "true" && moos_manual_overide == "false"){
    //Notify("DESIRED_THRUST", desired_thrust);
    Notify("DESIRED_RUDDER", desired_rudder);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool lanchaPID::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    double value = std::stod(line);

    bool handled = false;
    if(param == "apptick") {
      dt = 1/value;
      handled = true;
    }
    else if(param == "heading_kp") {
      headingController.setKP(value);
      handled = true;
    }
    else if(param == "heading_ki") {
      headingController.setKI(value);
      handled = true;
    }
    else if(param == "heading_kd") {
      headingController.setKD(value);
      handled = true;
    }
    else if(param == "heading_upper_bound") {
      headingController.setUpper(value);
      handled = true;
    }
    else if(param == "heading_lower_bound") {
      headingController.setLower(value);
      handled = true;
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void lanchaPID::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("DESIRED_SPEED", 0);
  Register("SPEED_KP", 0);
  Register("SPEED_KI", 0);
  Register("SPEED_KD", 0);
  Register("HEADING_KP", 0);
  Register("HEADING_KI", 0);
  Register("HEADING_KD", 0);
  Register("DEPLOY", 0);
  Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("CONSTANT_HEADING",0);
  Register("SETPOINT_HEADING",0);

}

//------------------------------------------------------------
// Procedure: buildReport()

bool lanchaPID::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab1(4);
  actab1 << "DESIRED_THRUST | DESIRED_RUDDER | CONSTANT_HEADING | SETPOINT_HEADING ";
  actab1.addHeaderLines();
  actab1 << desired_thrust << desired_rudder << constant_heading << setpoint_heading;
  m_msgs << actab1.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab2(3);
  actab2 << "SPEED_KP | SPEED_KI | SPEED_KD ";
  actab2.addHeaderLines();
  actab2 << speedController.getKP() << speedController.getKI() << speedController.getKD() ;
  m_msgs << actab2.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab3(3);
  actab3 << "HEADING_KP | HEADING_KI | HEADING_KD";
  actab3.addHeaderLines();
  actab3 << headingController.getKP() << headingController.getKI() << headingController.getKD();
  m_msgs << actab3.getFormattedString();
  m_msgs << endl;
  m_msgs << "---------------------------------------------------" << endl;

  ACTable actab4(2);
  actab4 << "HEADING_I_TERM | SPEED_I_TERM";
  actab4.addHeaderLines();
  actab4 << headingController.getIterm() << speedController.getIterm();
  m_msgs << actab4.getFormattedString();
  m_msgs << endl;

  return(true);
}
