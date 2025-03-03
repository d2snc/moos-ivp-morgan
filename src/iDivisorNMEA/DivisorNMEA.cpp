/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: DivisorNMEA.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <stdio.h>
#include <string.h>


// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"

//Includes do libais
#include "libais/ais.h"

//Includes do divisor NMEA

#include "DivisorNMEA.h"
#include "NMEAParserLib/NMEAParser.h"
#include "NMEAParserLib/NMEAParser.cpp"
#include "NMEAParserLib/NMEAParserData.h"
#include "NMEAParserLib/NMEAParserPacket.h"
#include "NMEAParserLib/NMEAParserPacket.cpp"
#include "NMEAParserLib/NMEASentenceBase.h"
#include "NMEAParserLib/NMEASentenceBase.cpp"
#include "NMEAParserLib/NMEASentenceGGA.h"
#include "NMEAParserLib/NMEASentenceGGA.cpp"
#include "NMEAParserLib/NMEASentenceGSA.h"
#include "NMEAParserLib/NMEASentenceGSA.cpp"
#include "NMEAParserLib/NMEASentenceGSV.h"
#include "NMEAParserLib/NMEASentenceGSV.cpp"
#include "NMEAParserLib/NMEASentenceRMC.cpp"
#include "NMEAParserLib/NMEASentenceRMC.h"

//Includes do libais
#include <memory>
#include <string>
#include "libais/ais.h"
#include "libais/ais.cpp"
#include "libais/ais_bitset.cpp"
#include "libais/ais1_2_3.cpp"

//Include do udpclient
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h> /* close() */
#include <string.h> /* memset() */

#define LOCAL_SERVER_PORT 10111 //Settings do recebedor UDP
#define MAX_MSG 1000 // Tamanho da mensagem UDP a ser recebida
#define BUFFER_SIZE 128


//Include do geodesy
//Serve para fazer a conversão do LAT/LON recebido pelo AIS para Coordenadas Locais 
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "../MOOSGeodesy.cpp" // adiciono a funcao de referencia
using namespace std;

//Variáveis globais para usar no programa e debugs

double lat_gps;
double long_gps;
double speed_gps;
double heading_gps;
double wind_angle;
double wind_speed;
double nav_depth;
int contador_ais;
double heading_giro;
double angulo_leme;
char* saida_pCmd;
char* saida_pData;
int msg_debug;
int current_gear;
int serial_fd;  // File descriptor for serial port

//Variáveis globais para o receiver UDP
int sd, rc, n;
socklen_t cliLen;
struct sockaddr_in cliAddr, servAddr;
char msg[MAX_MSG];


///
/// \class MyParser
/// \brief child class of CNMEAParser which will redefine notification calls from the parent class.
///
class MyNMEAParser : public CNMEAParser {

	///
	/// \brief This method is called whenever there is a parsing error.
	///
	/// Redefine this method to capture errors.
	///o
	/// \param pCmd Pointer to NMEA command that caused the error. Please note that this parameter may be NULL of not completely defined. Use with caution.
	///
	virtual void OnError(CNMEAParserData::ERROR_E nError, char *pCmd) {
		printf("ERROR for Cmd: %s, Number: %d\n", pCmd, nError);
	}

protected:
	///
	/// \brief This method is redefined from CNMEAParserPacket::ProcessRxCommand(char *pCmd, char *pData)
	///
	/// Here we are capturing the ProcessRxCommand to print out status. We also are looking for
	/// the GPGGA message and displaying some data from it.
	///
	/// \param pCmd Pointer to the NMEA command string
	/// \param pData Comma separated data that belongs to the command
	/// \return Returns CNMEAParserData::ERROR_OK If successful
	///
	virtual CNMEAParserData::ERROR_E ProcessRxCommand(char *pCmd, char *pData) {

		// Call base class to process the command
		try {
      CNMEAParser::ProcessRxCommand(pCmd, pData);
    }
    catch (std::system_error& e)
    {
      std::cout << e.what();
    }
    
    // Coloquei para debug
    saida_pCmd = pCmd;
    saida_pData = pData;
		printf("Cmd: %s\nData: %s\n", pCmd, pData);

		// Check if this is the GPGGA command. If it is, then display some data
		if (strstr(pCmd, "GPGGA") != NULL) {
			CNMEAParserData::GGA_DATA_T ggaData;
			if (GetGPGGA(ggaData) == CNMEAParserData::ERROR_OK) {
				//printf("GPGGA Parsed!\n");
				//printf("   Time:                %02d:%02d:%02d\n", ggaData.m_nHour, ggaData.m_nMinute, ggaData.m_nSecond);
				//lat_gps = ggaData.m_dLatitude;
        //long_gps = ggaData.m_dLongitude;
        //printf("   Latitude:            %f\n", ggaData.m_dLatitude);
				//printf("   Longitude:           %f\n", ggaData.m_dLongitude);
				//printf("   Altitude:            %.01fM\n", ggaData.m_dAltitudeMSL);
				//printf("   GPS Quality:         %d\n", ggaData.m_nGPSQuality);
				//printf("   Satellites in view:  %d\n", ggaData.m_nSatsInView);
				//printf("   HDOP:                %.02f\n", ggaData.m_dHDOP);
				//printf("   Differential ID:     %d\n", ggaData.m_nDifferentialID);
				//printf("   Differential age:    %f\n", ggaData.m_dDifferentialAge);
				//printf("   Geoidal Separation:  %f\n", ggaData.m_dGeoidalSep);
				//printf("   Vertical Speed:      %.02f\n", ggaData.m_dVertSpeed);
			}
		} else if (strstr(pCmd, "GPRMC") != NULL) {
      CNMEAParserData::RMC_DATA_T rmcdata;
      if (GetGPRMC(rmcdata) == CNMEAParserData::ERROR_OK) {
        lat_gps = rmcdata.m_dLatitude; // Latitude recebida
        long_gps = rmcdata.m_dLongitude; // Longitude recebida 
        //speed_gps = rmcdata.m_dSpeedKnots; // SOG do GPS
        heading_gps = rmcdata.m_dTrackAngle; // Marcação vinda do GPS - tirei esse e coloquei o heading vindo da giro

      }
    } 

		return CNMEAParserData::ERROR_OK;
	}
};

//---------------------------------------------------------
// Constructor()

DivisorNMEA::DivisorNMEA()
{
  serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); //Change serial port here
  if (serial_fd == -1) {
      perror("Error opening serial port");
      exit(1);
  }

  // Configure the serial port
  struct termios tty;
  if (tcgetattr(serial_fd, &tty) != 0) {
      perror("Error getting serial port attributes");
      exit(1);
  }

  // Set baud rate to 115200
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  // Configure 8N1 mode
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
  tty.c_cflag |= (CLOCAL | CREAD);  // Ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // No parity
  tty.c_cflag &= ~CSTOPB;            // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;           // No hardware flow control

  // Set raw input mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // Apply settings
  if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
      perror("Error setting serial port attributes");
      exit(1);
  }

  printf("Serial port /dev/ttyUSB0 configured successfully\n");

}

//---------------------------------------------------------
// Destructor

DivisorNMEA::~DivisorNMEA()
{
    //close(serial_port);
    if (serial_fd != -1) {
      close(serial_fd);
      printf("Serial port closed.\n");
    }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool DivisorNMEA::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "FOO") 
       cout << "great!";

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool DivisorNMEA::OnConnectToServer()
{
   registerVariables();
   return(true);
}

bool validate_nmea_checksum(const std::string& message) {
  if (message.size() < 3) return false; // Minimum "$X*00"
  
  size_t star_pos = message.find('*');
  if (star_pos == std::string::npos || star_pos + 2 >= message.size()) {
      return false; // No checksum
  }

  unsigned char checksum = 0;
  for (size_t i = 1; i < star_pos; i++) {
      checksum ^= message[i];
  }

  unsigned int received_checksum;
  std::stringstream ss;
  ss << std::hex << message.substr(star_pos + 1, 2);
  ss >> received_checksum;

  return checksum == received_checksum;
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool DivisorNMEA::Iterate() {
  AppCastingMOOSApp::Iterate();

  static char buffer[BUFFER_SIZE];
  static int index = 0;
  static bool capturing = false;

  while (true) {
      char c;
      int n = read(serial_fd, &c, 1);

      if (n > 0) {
          if (!capturing) {
              if (c == '$') { // Start capturing on '$'
                  capturing = true;
                  index = 0;
                  buffer[index++] = c;
              }
          } else {
              buffer[index++] = c;

              if (index >= BUFFER_SIZE - 1) { // Prevent buffer overflow
                  capturing = false;
                  index = 0;
                  continue;
              }

              if (c == '*') {
                  // Asterisk detected, next two chars must be checksum
              } else if (index > 3 && buffer[index - 3] == '*') {
                  // End of message (*XX), validate
                  buffer[index] = '\0';

                  if (isxdigit(buffer[index - 2]) && isxdigit(buffer[index - 1])) {
                      std::string nmea_msg(buffer);
                      if (validate_nmea_checksum(nmea_msg)) {
                          printf("Valid NMEA: %s\n", nmea_msg.c_str());
                          Notify("MSG_SERIAL", nmea_msg);

                          std::string msg_string = nmea_msg;

                          // Parse depth message
                          std::stringstream ss(msg_string);
                          std::string token;
                          std::getline(ss, token, ',');

                          if (token == "$SDDPT") {
                              std::getline(ss, token, ',');
                              nav_depth = std::stof(token);
                              Notify("NAV_DEPTH", nav_depth);
                          }

                          // Process NMEA sentence
                          MyNMEAParser NMEAParser;
                          try {
                              NMEAParser.ProcessNMEABuffer((char *)msg_string.c_str(), msg_string.length());
                          } catch (std::system_error& e) {
                              std::cout << e.what();
                          }

                          // Process AIS messages
                          if (msg_string.substr(0, 6) == "!AIVDM") {
                              try {
                                  const std::string body(libais::GetBody(nmea_msg));
                                  const int pad = libais::GetPad(nmea_msg);

                                  if (pad >= 0) {
                                      std::unique_ptr<libais::Ais1_2_3> msg(new libais::Ais1_2_3(body.c_str(), pad));

                                      if (msg->mmsi != 710400014) {
                                          Notify("MESSAGE_ID", msg->message_id);
                                          Notify("MESSAGE_MMSI", msg->mmsi);
                                          Notify("MESSAGE_NAVSTATUS", msg->nav_status);
                                          Notify("MESSAGE_SOG", msg->sog);
                                          Notify("MESSAGE_LONGITUDE", msg->position.lng_deg);
                                          Notify("MESSAGE_LATITUDE", msg->position.lat_deg);
                                          Notify("MESSAGE_TRUEHEADING", msg->true_heading);
                                      }
                                  }
                              } catch (std::system_error& e) {
                                  std::cout << e.what();
                              }
                          }

                          // Parse other NMEA messages
                          else if (msg_string.substr(0, 6) == "$AGRSA") {
                              try {
                                  angulo_leme = std::stod(libais::GetNthField(nmea_msg, 1, ","));
                              } catch (std::system_error& e) {
                                  std::cout << e.what();
                              }
                              Notify("ANGULO_LEME", angulo_leme);
                              Notify("NAV_YAW", angulo_leme);
                          }

                          else if (msg_string.substr(0, 6) == "$GPHDT") {
                              try {
                                  heading_giro = std::stod(libais::GetNthField(nmea_msg, 1, ","));
                                  Notify("NAV_HEADING", heading_giro);
                              } catch (std::system_error& e) {
                                  std::cout << e.what();
                              }
                          }

                          else if (msg_string.substr(0, 6) == "$GPRMC") {
                              try {
                                  speed_gps = std::stod(libais::GetNthField(nmea_msg, 7, ","));
                              } catch (std::system_error& e) {
                                  std::cout << e.what();
                              }
                              Notify("NAV_SPEED", speed_gps);
                          }

                          else if (msg_string.substr(0, 6) == "$WIMWV") {
                              try {
                                  wind_angle = std::stod(libais::GetNthField(nmea_msg, 1, ","));
                                  Notify("WIND_ANGLE", wind_angle);
                                  wind_speed = std::stold(libais::GetNthField(nmea_msg, 3, ","));
                                  Notify("WIND_SPEED", wind_speed);
                              } catch (std::system_error& e) {
                                  std::cout << e.what();
                              } catch (...) {
                                  std::cout << "\nThere is an error with the $WIMWV message! \n";
                              }
                          }
                      } else {
                          printf("Invalid checksum: %s\n", nmea_msg.c_str());
                      }
                  } else {
                      printf("Malformed message: %s\n", buffer);
                  }

                  capturing = false;
                  index = 0;
              }
          }
      } else if (n < 0) {
          perror("Error reading from serial port");
          return false;
      }
  }
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool DivisorNMEA::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  //Cria o socket para receber UDP na porta 10110

  /* socket creation */
  sd=socket(AF_INET, SOCK_DGRAM, 0);
  if(sd<0) {
    printf(" cannot open socket \n");
    exit(1);
  }

  /* bind local server port */
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servAddr.sin_port = htons(LOCAL_SERVER_PORT);
  rc = bind (sd, (struct sockaddr *) &servAddr,sizeof(servAddr));
  if(rc<0) {
    printf(" cannot bind port number %d \n", 
      LOCAL_SERVER_PORT);
    exit(1);
  }

  printf(" waiting for data on port UDP %u\n", 
    LOCAL_SERVER_PORT);

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
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

void DivisorNMEA::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("LINHA_NMEA", 0);
  Register("MSG_SERIAL", 0);
  //Register("NAV_YAW", 0);
 
}


//------------------------------------------------------------
// Procedure: buildReport()

bool DivisorNMEA::buildReport() 
{
  #if 0
    m_msgs << "============================================" << endl;
    m_msgs << "File:                                       " << endl;
    m_msgs << "============================================" << endl;

    ACTable actab(4);
    actab << "Alpha | Bravo | Charlie | Delta";
    actab.addHeaderLines();
    actab << "one" << "two" << "three" << "four";
    m_msgs << actab.getFormattedString();
  #endif

  ACTable actab(4);
  actab << "msg_debug | Saida_PCmd | Saida_PData | String Recebida ";
  actab.addHeaderLines();
  actab << msg_debug << saida_pCmd << saida_pData << read_buf;
  m_msgs << actab.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  ACTable actab2(5);
  actab2 << "Speed_GPS | Heading_GPS | Heading_Giro | Lat_Recebida_GPS | Long_Recebida_GPS ";
  actab2.addHeaderLines();
  actab2 << speed_gps << heading_gps << heading_giro << lat_gps << long_gps;
  m_msgs << actab2.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  ACTable actab3(2);
  actab3 << "Wind_Speed | Wind_Angle";
  actab3.addHeaderLines();
  actab3 << wind_speed << wind_angle;
  m_msgs << actab3.getFormattedString();
  m_msgs << endl << "============================================" << endl;

  return(true);
}
