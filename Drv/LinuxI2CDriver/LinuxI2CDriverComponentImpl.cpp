// ======================================================================
// \title  LinuxI2CDriverComponentImpl.cpp
// \author aaron
// \brief  cpp file for LinuxI2CDriver component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <errno.h>

#define I2C_FILE (char*)"dev/i2c-1"

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LinuxI2CDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LinuxI2CDriverComponentImpl(
        const char *const compName
    ) :
      LinuxI2CDriverComponentBase(compName)
#else
    LinuxI2CDriverComponentImpl(void)
#endif
  {

  }

  void LinuxI2CDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    LinuxI2CDriverComponentBase::init(instance);
  }

  LinuxI2CDriverComponentImpl ::
    ~LinuxI2CDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LinuxI2CDriverComponentImpl ::
    write_handler(
        const NATIVE_INT_TYPE portNum,
        U32 address,
        Fw::Buffer &writeBuffer
    )
  {
    int file_i2c = open(I2C_FILE, O_CLOEXEC | O_RDWR);
    FW_ASSERT(!(file_i2c < 0));
    ioctl(file_i2c, I2C_SLAVE, address);
    write(file_i2c, (void*)writeBuffer.getdata(), writeBuffer.getsize());
  }

} // end namespace Drv
