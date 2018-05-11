// ====================================================================== 
// \title  GpsImpl.cpp
// \author mstarch
// \brief  cpp file for Gps component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
// 
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ====================================================================== 


#include <GpsApp/Gps/GpsComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

//File path to UART device on UNIX system
#define UART_FILE_PATH "/dev/ttyACM0"
#include <cstring>
#include <iostream>
//POSIX includes for UART communication
extern "C" {
    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
}

namespace GpsApp {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  GpsComponentImpl ::
#if FW_OBJECT_NAMES == 1
    GpsComponentImpl(
        const char *const compName
    ) :
      GpsComponentBase(compName),
#else
      GpsComponentBase(void),
#endif
      m_setup(false),
      m_locked(false),
      m_fh(-1)
  {

  }

  void GpsComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    ) 
  {
    GpsComponentBase::init(queueDepth, instance);
  }

  GpsComponentImpl ::
    ~GpsComponentImpl(void)
  {

  }

  void GpsComponentImpl ::
    setup(void)
  {
      if (m_setup) {
          return;
      }
      //Open the GPS, and configure it for a raw-socket in read-only mode
      m_fh = open(UART_FILE_PATH, O_RDONLY);
      if (m_fh < 0) {
          std::cout << "[ERROR] Failed to open file: " << UART_FILE_PATH << std::endl;
          return;
      }
      //Setup the struct for termios configuration
      struct termios options;
      std::memset(&options, 0, sizeof(options));
      //Set to raw socket, remove modem control, allow input
      cfmakeraw(&options);
      options.c_cflag |= (CLOCAL | CREAD);
      //Set to 9600 baud
      cfsetispeed(&options, B9600);
      //Make the above options stick
      NATIVE_INT_TYPE status = tcsetattr(m_fh, TCSANOW, &options);
      if (status != 0) {
          std::cout << "[ERROR] Failed to setup UART options" << std::endl;
          return;
      }
      m_setup = true;
  }
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void GpsComponentImpl ::
    schedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      int status = 0;
      float lat = 0.0f, lon = 0.0f;
      GpsPacket packet;
      char buffer[1024];
      char* pointer = buffer;
      //Try to setup, if failure, do no more
      setup();
      if (!m_setup) {
          return;
      }
      ssize_t size = read(m_fh, buffer, sizeof(buffer));
      if (size <= 0) {
          std::cout << "[ERROR] Failed to read from UART with: " << size << std::endl;
          return;
      }
      for (U32 i = 0; i < sizeof(buffer) - 6; i++) {
          status = sscanf(pointer, "$GPGGA,%f,%f,%c,%f,%c,%u,%u,%f,%f",
              &packet.utcTime, &packet.dmNS, &packet.northSouth,
              &packet.dmEW, &packet.eastWest, &packet.lock,
              &packet.count, &packet.filler2, &packet.altitude);
          //Break when all GPS items are found
          if (status == 9) {
              break;
          }
          pointer = pointer + 1;
      }
      if (status != 9) {
          std::cout << "[ERROR] GPS parsing failed: " << status << std::endl;
          return;
      }
      //Latitude ddmm.mmmm
      lat = ((packet.northSouth == 'N') ? 1 : -1) *
            ((packet.dmNS - (U32)packet.dmNS)/60.0 + (U32)packet.dmNS);
      lon = ((packet.eastWest == 'E') ? 1 : -1) *
            ((packet.dmEW - (U32)packet.dmEW)/60.0 + (U32)packet.dmEW);
      tlmWrite_Gps_Latitude(lat);
      tlmWrite_Gps_Longitude(lon);
      tlmWrite_Gps_Altitude(packet.altitude);
      tlmWrite_Gps_Count(packet.count);
      //Lock update
      if (packet.lock == 0) {
          m_locked = false;
          log_WARNING_HI_Gps_LockLost();
      } else {
          m_locked = true;
          log_ACTIVITY_HI_Gps_LockAquired();
      }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void GpsComponentImpl ::
    Gps_ReportLockStatus_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    //Locked-force print
    if (m_locked) {
        log_ACTIVITY_HI_Gps_LockAquired();
    } else {
        log_WARNING_HI_Gps_LockLost();
    }
  }
} // end namespace GpsApp
