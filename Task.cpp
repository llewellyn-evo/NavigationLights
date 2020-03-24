//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Llewellyn-Fernandes                                              *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "CommLights.hpp"

namespace Monitors
{
  namespace Navlight
  {
    using DUNE_NAMESPACES;

     struct Arguments
    {
      //Duty Cycle for NavLight between 0 to 100%
      float NavDutyCycle;
      //Navigation light PWM Period
      float  NavPeriod;
      //Navigation Light default state
      uint8_t NavState;
      //Flash light Duty Cycle 0 to 100%
      float FlashDutyCycle;
      //Flash light PWM period
      float FlashPeriod;
      //Flash light Intensity
      uint16_t FlashIntensity;
      //Flash Light Default State
      uint8_t FlashState;
      //i2c bus for Digital to Analog Convertor
      std::string i2c_bus;
      //! I2C address of Digital to Analog Convertor.
      uint8_t i2c_addr;

    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Arguments m_args;
      CommLights m_lights;


      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
          param("Nav-Light DutyCycle", m_args.NavDutyCycle)
          .defaultValue("50.0")
          .description("This controls the brightness of the Navigation light");

          param("Nav-Light Period", m_args.NavPeriod)
          .defaultValue("0.01")
          .description("The value is set in seconds for Navigation light PWM");

          param("Nav-Light Default-State", m_args.NavState)
          .defaultValue("0")
          .description("The Sets Navigation Light ON or OFF");


          param("Flash-Light DutyCycle", m_args.FlashDutyCycle)
          .defaultValue("1")
          .description("This controls the flash ON period set in percentage");

          param("Flash-Light Period", m_args.FlashPeriod)
          .defaultValue("1")
          .description("This controls the total period of flash ON+OFF");

          param("Flash-Light Default-State", m_args.FlashState)
          .defaultValue("0")
          .description("The Sets Flash Light ON or OFF");

          param("Flash-LightIntensity", m_args.FlashIntensity)
          .defaultValue("512")
          .description("Sets the intensity for Flash Light ");


           param("I2C Bus", m_args.i2c_bus)
          .defaultValue("/dev/i2c-0")
          .description("I2C device(s) bus (dev-filename)");

            param("I2C DAC Address", m_args.i2c_addr)
          .defaultValue("0x34")
          .description("Address of DAC");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_lights.InitNavLight(m_args.NavPeriod , m_args.NavDutyCycle);
        m_lights.InitFlashLight( m_args.i2c_bus , m_args.i2c_addr , m_args.FlashPeriod , m_args.FlashDutyCycle);
       
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
         m_lights.SetNavLight( m_args.NavState , m_args.NavDutyCycle);
         m_lights.SetFlashLight( m_args.FlashState , m_args.FlashDutyCycle , m_args.FlashIntensity);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        //m_lights.SetFlashLight(0);
        //m_lights.SetNavLight(0);
      }

      //! Main loop.
      void
      onMain(void)
      {
        //uint16_t x;
        while (!stopping())
        {
          m_lights.NavLightTimerCkeck();
          waitForMessages(0.3);
        }
      }
    };
  }
}

DUNE_TASK
