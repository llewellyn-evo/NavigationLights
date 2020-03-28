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
#include <cstring>
#include "CommLights.hpp"

namespace Control
{
  namespace NavigationLights
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      //! Brightness for NavLight between 0 to 100%
      float nav_brightness;
      //! Navigation light PWM Period
      float  nav_period;
      //! Navigation Light default state
      uint8_t nav_default_state;
      //! Flash light Duty Cycle 0 to 100%
      float flash_duty_cycle;
      //! Navigation light PWM chip
      uint8_t nav_pwm_chip;
      //! Name of Navigation LED channel
      std::string nav_id;
      //! Flash light PWM period
      float flash_period;
      //! Flash light Intensity
      uint16_t flash_brightness;
      //! Flash Light Default State
      uint8_t flash_default_state;
      //! Flash light PWM chip
      uint8_t flash_pwm_chip;
      //! Name of Flash LED channel
      std::string flash_id;
      //! i2c bus for Digital to Analog Convertor
      std::string i2c_bus;
      //! i2c address of Digital to Analog Convertor.
      uint8_t i2c_addr;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Arguments m_args;
      CommLights* m_lights;


      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx),
      m_lights(NULL)
      {
        param("Nav-Light Brightness", m_args.nav_brightness)
        .defaultValue("50.0")
        .units(Units::Percentage)
        .minimumValue("0.0")
        .maximumValue("100.0")
        .description("This controls the brightness of the Navigation light");

        param("Nav-Light Period", m_args.nav_period)
        .defaultValue("0.01")
        .units(Units::Second)
        .description("The value is set in seconds for Navigation light PWM");

        param("Nav-Light Default-State", m_args.nav_default_state)
        .defaultValue("0")
        .description("The Sets Navigation Light ON or OFF");

         param("Nav-Light ID", m_args.nav_id)
        .defaultValue("NAV")
        .description("This sets the ID for NAV Light channel");


        param("Flash-Light DutyCycle", m_args.flash_duty_cycle)
        .defaultValue("1")
        .units(Units::Percentage)
        .minimumValue("0.0")
        .maximumValue("100.0")
        .description("This controls the flash ON period set in percentage");

        param("Flash-Light Period", m_args.flash_period)
        .defaultValue("1")
        .units(Units::Second)
        .description("This controls the total period of flash ON+OFF");

        param("Flash-Light Default-State", m_args.flash_default_state)
        .defaultValue("0")
        .description("The Sets Flash Light ON or OFF");

        param("Flash-Light Brightness", m_args.flash_brightness)
        .defaultValue("512")
        .minimumValue("1")
        .maximumValue("1023")
        .description("Sets the intensity for Flash Light ");

        param("Flash-Light ID", m_args.flash_id)
        .defaultValue("FLASH")
        .description("This sets the ID for Flash Light channel");


        param("I2C Bus", m_args.i2c_bus)
        .defaultValue("/dev/i2c-0")
        .description("I2C device(s) bus (dev-filename)");

        param("I2C DAC Address", m_args.i2c_addr)
        .defaultValue("0x34")
        .description("Address of DAC");

        param("Nav-Light PWMChip", m_args.nav_pwm_chip)
        .defaultValue("0")
        .description("PWM chip number for navigation light");

        param("Flash-Light PWMChip", m_args.flash_pwm_chip)
        .defaultValue("1")
        .description("PWM chip number for flash light");

        bind<IMC::QueryLedBrightness>(this);
        bind<IMC::SetLedBrightness>(this);
        bind<IMC::QueryPowerChannelState>(this);
        bind<IMC::PowerChannelControl>(this);
        bind<IMC::SetPWM>(this);

      }
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_lights)
        {
          if (paramChanged(m_args.i2c_bus) || paramChanged(m_args.i2c_addr) || paramChanged(m_args.nav_pwm_chip) || paramChanged(m_args.flash_pwm_chip))
            throw RestartNeeded(DTR("restarting to change parameters"), 1);

          if (paramChanged(m_args.nav_brightness))
            m_lights->setNavBrightness(m_args.nav_brightness);

          if (paramChanged(m_args.nav_period))
            m_lights->setNavPeriod(m_args.nav_period);

          if (paramChanged(m_args.nav_default_state))
            m_lights->setNavState( m_args.nav_default_state );


          if (paramChanged(m_args.flash_period))
            m_lights->setFlashPeriod( m_args.flash_period );

          if (paramChanged(m_args.flash_duty_cycle))
            m_lights->setFlashDutyCycle( m_args.flash_duty_cycle );

          if (paramChanged(m_args.flash_brightness))
            m_lights->setFlashBrightness( m_args.flash_brightness );

          if (paramChanged(m_args.flash_default_state))
            m_lights->setFlashState( m_args.flash_default_state );
        }
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_lights = new CommLights(this);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_lights->initNavLight("/sys/class/pwm/pwmchip" + std::to_string(m_args.nav_pwm_chip) + "/", m_args.nav_period , m_args.nav_brightness , m_args.nav_default_state  );
        m_lights->initFlashLight(m_args.i2c_bus , "/sys/class/pwm/pwmchip" + std::to_string(m_args.flash_pwm_chip) + "/"  , m_args.i2c_addr , m_args.flash_period , m_args.flash_duty_cycle , m_args.flash_brightness , m_args.flash_default_state );
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_lights);
      }

      //! Querry LED brightness
      void
      consume(const IMC::QueryLedBrightness* msg)
      {
        IMC::LedBrightness brightness_state;
        if (msg->getDestination() != getSystemId())
          return;
        if (!m_args.nav_id.compare(msg->name))
        {
          brightness_state.name = msg->name;
          brightness_state.value = (uint8_t)m_lights->m_nav_brightness;
        }
         else if (!m_args.flash_id.compare(msg->name))
        {
          brightness_state.name = msg->name;
          brightness_state.value = (uint8_t)m_lights->m_flash_brightness;
        }
        else
        {
          return;
        }
        dispatch(brightness_state);
      }

      //! Set LED brightness
      void
      consume(const IMC::SetLedBrightness* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (m_args.nav_id == msg->name)
        {
          m_lights->setNavBrightness((float) msg->value);
        }
        else if (m_args.flash_id == msg->name)
        {
          //! Change 0 to 100 % to 0 to 1023
          m_lights->setFlashBrightness((uint16_t) msg->value * 10.23 );
        }
      }

      //! Querry Power channel state
      void
      consume(const IMC::QueryPowerChannelState* msg)
      {
        IMC::PowerChannelState nav_chan_state;
        if (msg->getDestination() != getSystemId())
          return;
        nav_chan_state.name = m_args.nav_id;
        if (m_lights->m_nav_state)
        {
          nav_chan_state.state =  IMC::PowerChannelState::PCS_ON;
        }
        else
        {
          nav_chan_state.state =  IMC::PowerChannelState::PCS_OFF;
        }
        dispatch(nav_chan_state);
        IMC::PowerChannelState flash_chan_state;
        flash_chan_state.name = m_args.flash_id;
        if (m_lights->m_flash_state)
        {
          flash_chan_state.state =  IMC::PowerChannelState::PCS_ON;
        }
        else
        {
          flash_chan_state.state =  IMC::PowerChannelState::PCS_OFF;
        }
        dispatch(flash_chan_state);
      }

      //! Set Power channel state
      void
      consume(const IMC::PowerChannelControl* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (!m_args.nav_id.compare(msg->name))
        {
          if (msg->op == IMC::PowerChannelControl::PCC_OP_TURN_OFF)
          {
            m_lights->setNavState(0);
          }
          else if (msg->op == IMC::PowerChannelControl::PCC_OP_TURN_ON)
          {
            m_lights->setNavState(1);
          }
        }
        else if (!m_args.flash_id.compare(msg->name))
        {
          if (msg->op == IMC::PowerChannelControl::PCC_OP_TURN_OFF)
          {
            m_lights->setFlashState(0);
          }
          else if (msg->op == IMC::PowerChannelControl::PCC_OP_TURN_ON)
          {
            m_lights->setFlashState(1);
          }
        }
      }

      //! Set PWM
      void
      consume(const IMC::SetPWM* msg)
      {
        if (msg->getDestination() != getSystemId())
        {
          return;
        }

        if (msg->id == m_args.flash_pwm_chip)
        {
          //! Convert to Seconds
          m_lights->setFlashPeriod((float)msg->period/1000000);
          //! Convert to Percentage
          m_lights->setFlashDutyCycle((float) (msg->duty_cycle/msg->period) * 100);
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          m_lights->updateNavTimer();
          waitForMessages(0.05);
        }
      }
    };
  }
}

DUNE_TASK
