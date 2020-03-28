#ifndef MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED
#define MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>

#define MAX5811_LOAD_DAC_A_IN_REG_A   0xC0
#define MAX5811_DAC_POWER_UP          0x40

namespace Control
{
  namespace NavigationLights
  {
    using DUNE_NAMESPACES;
    class CommLights
    {
    public:
      CommLights(Tasks::Task* task):
      m_task(task)
      {
        m_nav_lights_flashing_timer.setTop(0.5);
      }

      ~CommLights()
      {
        m_nav_pwm->disable();
        m_flash_pwm->disable();
        delete m_nav_pwm;
        delete m_flash_pwm;
      }

      void
      updateNavTimer()
      {
        static uint8_t flashing_state = 0;
        if (m_nav_lights_flashing_timer.overflow())
        {
          if (m_is_being_identified)
          {
            switch (flashing_state){
              case 0:
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
                setNavState(!m_nav_state);
                flashing_state++;
                break;
              case 6:
                flashing_state = 0;
                m_is_being_identified = false;
              break;
            }
          }
          m_nav_lights_flashing_timer.reset();
        }
      }

      //! Function Used to Identify the vehicle
      //! Will blink the Navigation lights 3 times
      void
      identifyVehicle()
      {
        m_is_being_identified = true;
      }

      void
      initNavLight(const std::string& nav_pwm_chip, const float nav_period, const float nav_brightness, const uint8_t state)
      {
        //Validate brightness and period
        if (!nav_period && !nav_brightness)
        {
          return;
        }
        m_nav_pwm =  new  DUNE::Hardware::PWM(0 , nav_pwm_chip);
        setNavPeriod(nav_period);
        setNavBrightness(nav_brightness);
        setNavState(state);
      }

      void
      setNavBrightness(const float nav_brightness)
      {
        //state needs to be 0 for off and 1 or above for on
        //Duty_cycle is between 0 to 100%
        if (nav_brightness > 0 && nav_brightness < 101)
        {
          m_nav_brightness = nav_brightness;
          m_nav_pwm->setDutyCyclePercentage(m_nav_brightness);
        }
      }

      void
      setNavState(const uint8_t state)
      {
        m_nav_state = state;
        if (m_nav_state)
        {
          m_nav_pwm->enable();
        }
        else
        {
          m_nav_pwm->disable();
        }
      }

      void
      setNavPeriod(const float nav_period)
      {
        m_nav_period = nav_period;
        m_nav_pwm->setPeriod(m_nav_period);
      }

      void
      initFlashLight(const std::string& i2c_dev, const std::string& pwm_chip, const uint8_t address, const float period, const float duty_cycle, const uint16_t brightness, const uint8_t state) 
      {
        if (!period && !duty_cycle)
        {
          return;
        }
        m_flash_pwm =  new  DUNE::Hardware::PWM(0 , pwm_chip);
        setFlashPeriod(period);
        setFlashDutyCycle(duty_cycle);
        try
        {
          m_dac = new DUNE::Hardware::I2C(i2c_dev);
          m_dac->connect(address);
        }
        catch (...)
        {
          throw RestartNeeded("I2C Bus/device opening/connecting failure", 30);
        }

        try
        {
          uint8_t command = MAX5811_DAC_POWER_UP;
          m_dac->write(&command , 1);
          setFlashBrightness(brightness);
        }
        catch (...)
        {
          m_task->err("Error in writting to i2c Device \r\n");
        }
        setFlashState(state);
      }

      void
      setFlashDutyCycle(const float  duty_cycle)
      {
        if (duty_cycle > 0 && duty_cycle < 101)
        {
          m_flash_duty_cycle = duty_cycle;
          m_flash_pwm->setDutyCyclePercentage(m_flash_duty_cycle);
        }
      }

      void
      setFlashPeriod(const float period)
      {
         m_flash_period = period;
         m_flash_pwm->setPeriod(m_flash_period);
      }

      void
      setFlashState(const uint8_t state)
      {
        m_flash_state = state;
        if (m_flash_state)
        {
          m_flash_pwm->enable();
        }
        else
        {
          m_flash_pwm->disable();
        }
      }

      void
      setFlashBrightness(const uint16_t brightness)
      {
        uint8_t command[2];
        if (brightness > 0 && brightness < 1024)
        {
          m_flash_brightness = brightness;
          command[0] = MAX5811_LOAD_DAC_A_IN_REG_A;
          command[0] |= m_flash_brightness >> 6;
          command[1] = (m_flash_brightness & 0x3f) << 2;

          try
          {
            m_dac->write(command , 2);
          }
          catch (...)
          {
            m_task->err("Error in writting to i2c Device \r\n");
          }
        }
      }
      //! PWM for Navigation Light
      DUNE::Hardware::PWM* m_nav_pwm;
      //! PWM for Flash Light
      DUNE::Hardware::PWM* m_flash_pwm;
      //! DAC for Flash Brightness
      DUNE::Hardware::I2C* m_dac;
      //! Counter for flashing of Navigation lights for Identifying vehicle
      DUNE::Time::Counter<float> m_nav_lights_flashing_timer;
      //! Task
      Tasks::Task* m_task;
      //! Variables for Navigation Light
      uint8_t m_nav_state;
      float m_nav_period , m_nav_brightness;
      //! Variables for Flash light
      uint8_t m_flash_state;
      uint16_t m_flash_brightness;
      float m_flash_period , m_flash_duty_cycle;
      //! Variable to identify vehicle
      bool m_is_being_identified;
    };
  }
}
#endif
