#ifndef MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED
#define MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>

#define MAX5811_LOAD_DAC_A_IN_REG_A   0xC0
#define MAX5811_DAC_POWER_UP          0x40

namespace Control
{
  namespace Navlight
  {
    using DUNE_NAMESPACES;
    class CommLights
    {
    public:
      CommLights(Tasks::Task* task):
      m_task(task)
      {
        m_counter.setTop(0.5);
      }

      ~CommLights()
      {
        m_nav_pwm->disable();
        m_flash_pwm->disable();
        m_nav_pwm->~PWM();
        m_flash_pwm->~PWM();
      }

      void
      NavLightTimerCkeck()
      {
        static uint8_t flashing_state = 0;
        if(m_counter.overflow()){
          if(m_identify){
            switch(flashing_state){
              case 0:
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              SetNavState(!m_nav_state);
              flashing_state++;
              break;
              case 6:
              flashing_state = 0;
              m_identify = 0;
              break;
            }
          }
          m_counter.reset();
        }
      }

      //! Function Used to Identify the vehicle
      //! Will blink the Navigation lights 3 times
      void
      IdentifyVehicle()
      {
        m_identify = 1;
      }

      void
      InitNavLight( std::string& nav_pwm_chip,  float nav_period , float nav_brightness , uint8_t state )
      {
        //Validate brightness and period
        if(!nav_period && !nav_brightness)
        {
          return;
        }
        m_nav_pwm =  new  DUNE::Hardware::PWM(0 , nav_pwm_chip);
        m_nav_period = nav_period;
        m_nav_brightness = nav_brightness;
        m_nav_pwm->setPeriod(m_nav_period);
        m_nav_pwm->setDutyCyclePercentage(m_nav_brightness);
        m_nav_state = state;
        if(m_nav_state){
          m_nav_pwm->enable();
        }else{
          m_nav_pwm->disable();
        }
      }

      void
      SetNavBrightness( float nav_brightness )
      {
        //state needs to be 0 for off and 1 or above for on
        //Duty_cycle is between 0 to 100%
        if(nav_brightness > 0 && nav_brightness < 101){
          m_nav_brightness = nav_brightness;
          m_nav_pwm->setDutyCyclePercentage(m_nav_brightness);
        }
      }

      void
      SetNavState(uint8_t state)
      {
        m_nav_state = state;
        if(m_nav_state){
          m_nav_pwm->enable();
        }else{
          m_nav_pwm->disable();
        }
      }

      void
      SetNavPeriod( float nav_period )
      {
        m_nav_period = nav_period;
        m_nav_pwm->setPeriod(m_nav_period);
      }

      void
      InitFlashLight( std::string& i2c_dev , std::string& pwm_chip ,  uint8_t address , float period , float duty_cycle , uint16_t brightness , uint8_t state)
      {
        uint8_t command[2];
        if(!period && !duty_cycle){
          return;
        }
        m_flash_period = period;
        m_flash_brightness = brightness;
        m_flash_duty_cycle = duty_cycle;

        try{
          m_dac = new DUNE::Hardware::I2C(i2c_dev);
          m_dac->connect(address);
        }catch(...){
          throw RestartNeeded("I2C Bus/device opening/connecting failure", 30);
        }

        try{
          command[0] = MAX5811_DAC_POWER_UP;
          m_dac->write(command , 1);
          command[0] = MAX5811_LOAD_DAC_A_IN_REG_A;
          command[0] |= brightness >> 6;
          command[1] = (brightness & 0x3f) << 2;
          m_dac->write(command , 2);
        }catch(...){
          m_task->err("Error in writting to i2c Device \r\n");
        }

        m_flash_pwm =  new  DUNE::Hardware::PWM(0 , pwm_chip);
        m_flash_pwm->setPeriod(m_flash_period);
        m_flash_pwm->setDutyCyclePercentage(m_flash_duty_cycle);

        m_flash_state = state;
        if(m_flash_state){
          m_flash_pwm->enable();
        }else{
          m_flash_pwm->disable();
        }
      }

      void
      SetFlashDutyCycle(float  duty_cycle)
      {
        if(duty_cycle > 0 && duty_cycle < 101)
        {
          m_flash_duty_cycle = duty_cycle;
          m_flash_pwm->setDutyCyclePercentage(m_flash_duty_cycle);
        }
      }

      void
      SetFlashPeriod(float period)
      {
         m_flash_period = period;
         m_flash_pwm->setPeriod(m_flash_period);
      }

      void
      SetFlashState(uint8_t state)
      {
        m_flash_state = state;
        if(m_flash_state){
          m_flash_pwm->enable();
        }else{
          m_flash_pwm->disable();
        }
      }

      void
      SetFlashBrightness(uint16_t brightness)
      {
        uint8_t command[2];
        if(brightness > 0 && brightness < 1024)
        {
          m_flash_brightness = brightness;
          command[0] = MAX5811_LOAD_DAC_A_IN_REG_A;
          command[0] |= m_flash_brightness >> 6;
          command[1] = (m_flash_brightness & 0x3f) << 2;

          try{
            m_dac->write(command , 2);
          }catch(...){
            m_task->err("Error in writting to i2c Device \r\n");
          }
        }
      }

      //! Pointer to PWM for Navigation Light
      DUNE::Hardware::PWM* m_nav_pwm;
      //! Pointer to PWM for Flash Light
      DUNE::Hardware::PWM* m_flash_pwm;
      //! Pointer to DAC for Flash Brightness
      DUNE::Hardware::I2C* m_dac;
      //! Pointer to Counter
      DUNE::Time::Counter<float> m_counter;
      //! Pointer to task
      Tasks::Task* m_task;
      //! Defines for Navigation Light
      uint8_t m_nav_state;
      float m_nav_period , m_nav_brightness;
      //! Definations for Flash light
      uint8_t m_flash_state;
      uint16_t m_flash_brightness;
      float m_flash_period , m_flash_duty_cycle;
      //! Flash to identify vehicle
      uint8_t m_identify;
    };
  }
}

#endif
