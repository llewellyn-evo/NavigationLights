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
// Author: Llewellyn-Fernandes                                                    *
//***************************************************************************

#ifndef MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED
#define MONITORS_NAV_LIGHT_COMM_LIGHTS_INCLUDED

// ISO C++ 98 headers.
#include <cstring>
#include <iostream>
// DUNE headers.
#include <DUNE/DUNE.hpp>

#define MAX5811_LOAD_DAC_A_IN_REG_A   0xC0
#define MAX5811_DAC_POWER_UP          0x40

namespace Monitors
{
  namespace Navlight
  {
    using DUNE_NAMESPACES;
    class CommLights
    {
    public:
        DUNE::Hardware::PWM* n_pwm;
        DUNE::Hardware::PWM* f_pwm;
        DUNE::Hardware::I2C* dac;
        DUNE::Time::Counter<fp64_t> m_counter;

        void NavLightTimerCkeck(){
          static uint8_t m_state = 0;
          if(m_counter.overflow()){
            if(getIdentify()){
              switch(m_state){
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                  SetNavLight(!getNavState());
                  m_state++;
                  break;
                case 6:
                  m_state = 0;
                  setIdentify(0);
                  break;
              }
            }
            m_counter.reset();
          }
        }


        void IdentifyVehicle(){
           setIdentify(1);
        }

       int InitNavLight(float period , float duty_cycle){
           //Validate duty_cycle and period
          if(!period && !duty_cycle){
            return -1;
          }
          n_pwm =  new  DUNE::Hardware::PWM(0 , std::string("/sys/class/pwm/pwmchip0/"));
          setNavPeriod(period);
          setNavDutyCycle(duty_cycle);
          n_pwm->setDutyCyclePercentage(getNavDutyCycle());
          n_pwm->setPeriod(getNavPeriod());
          //500ms timer 
          m_counter.setTop(0.5);
          return 0;
       }

       void SetNavLight(uint8_t state , float  duty_cycle = 0.0){
        //state needs to be 0 for off and 1 or above for on
        //Duty_cycle is between 0 to 100%
        if(duty_cycle > 0.0){
          setNavDutyCycle(duty_cycle);
          n_pwm->setDutyCyclePercentage(getNavDutyCycle());
        }

        setNavState(state);
        if(getNavState()){
          n_pwm->enable();
        }else{
          n_pwm->disable();
        }
       }

       int InitFlashLight( std::string& dev ,  uint8_t address , float period , float duty_cycle){
        uint8_t buffer[1];

        if(!period && !duty_cycle){
          return -1;
        }
        try{
          dac = new DUNE::Hardware::I2C(dev);
          
        }catch(...){
            throw RestartNeeded("I2C Bus/device opening/connecting failure", 30);
        }
        dac->connect(address);
        try{
          buffer[0] = MAX5811_DAC_POWER_UP;
          dac->write((uint8_t*)buffer , 1);
        }catch(...){
          std::cout << "Error in writting to i2c Device \r\n";
        }

        f_pwm =  new  DUNE::Hardware::PWM(0 , std::string("/sys/class/pwm/pwmchip1/"));
        setFlashPeriod(period);
        setFlashDutyCycle(duty_cycle);
        f_pwm->setDutyCyclePercentage(getFlashDutyCycle());
        f_pwm->setPeriod(getFlashPeriod());
        return 0;
       }

      void SetFlashLight(uint8_t state , float  duty_cycle = 0.0 , int intensity = 0){
        //state needs to be 0 for off and 1 or above for on
        //intensity is between 0 to 100%
        uint8_t outbuf[2];
        if(duty_cycle > 0.0){
          setFlashDutyCycle(duty_cycle);
          f_pwm->setDutyCyclePercentage(getFlashDutyCycle());
        }

        setFlashState(state);
        if(getFlashState()){
          f_pwm->enable();
        }else{
          f_pwm->disable();
        }

        if (intensity > 1023 || intensity == 0){
          return;
        }
        outbuf[0] = MAX5811_LOAD_DAC_A_IN_REG_A;
        outbuf[0] |= intensity >> 6;
        outbuf[1] = (intensity & 0x3f) << 2;
        dac->write(outbuf , 2);
      }

      
    private:
      //Navigation Light 
      uint8_t NavState = 0;
      float NavPeriod , NavDutyCycle;
      //Flash Light 
      uint8_t FlashState;
      uint16_t FlashIntensity;
      float FlashPeriod , FlashDutyCycle;

      //Identify Vehicle
      uint8_t Identify;

      uint8_t getIdentify(){
        return Identify;
      }

      void setIdentify(uint8_t val){
        Identify = val;
      }


       uint8_t getNavState(){
        return NavState;
       }

       void setNavState(uint8_t state){
          NavState = state;
       }

       float getNavPeriod(){
        return NavPeriod;
       }

       void setNavPeriod(float period){
          NavPeriod = period;
       }

       float getNavDutyCycle(){
        return NavDutyCycle;
       }

       void setNavDutyCycle(float duty_cycle){
          NavDutyCycle = duty_cycle;
       }


       uint8_t getFlashState(){
        return FlashState;
       }

       void setFlashState(uint8_t state){
          FlashState = state;
       }

      float getFlashPeriod(){
        return FlashPeriod;
       }

       void setFlashPeriod(float period){
          FlashPeriod = period;
       }

       float getFlashDutyCycle(){
        return FlashDutyCycle;
       }

       void setFlashDutyCycle(float duty_cycle){
          FlashDutyCycle = duty_cycle;
       }

       uint16_t getFlashIntensity(){
          return FlashIntensity;
       }

       void setFlashIntensity(uint16_t value){
          FlashIntensity = value;
       }
    };
  }
}

#endif
