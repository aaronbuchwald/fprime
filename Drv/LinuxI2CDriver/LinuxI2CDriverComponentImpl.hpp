// ======================================================================
// \title  LinuxI2CDriverComponentImpl.hpp
// \author aaron
// \brief  hpp file for LinuxI2CDriver component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef LinuxI2CDriver_HPP
#define LinuxI2CDriver_HPP

#include "Drv/LinuxI2CDriver/LinuxI2CDriverComponentAc.hpp"

namespace Drv {

  class LinuxI2CDriverComponentImpl :
    public LinuxI2CDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LinuxI2CDriver
      //!
      LinuxI2CDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LinuxI2CDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object LinuxI2CDriver
      //!
      ~LinuxI2CDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for write
      //!
      void write_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 address, 
          Fw::Buffer &writeBuffer 
      );


    };

} // end namespace Drv

#endif
