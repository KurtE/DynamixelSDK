/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#if defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)

#include <Arduino.h>


#include "../../include/dynamixel_sdk/port_handler_arduino.h"

#if defined (__OPENCR__)
#define DYNAMIXEL_SERIAL  Serial3
#endif

#define LATENCY_TIMER     16  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerArduino::PortHandlerArduino(const char *port_name)
  : baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  port_open_ = false;
  setPortName(port_name);

#if defined(__OPENCR__)
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);

  setPowerOff();
#elif defined(__OPENCM904__) || defined(ARDUINO)
  if (port_name[0] == '1')
  {
    socket_fd_ = 0;
    p_dxl_serial = &Serial1;
  }
  else if (port_name[0] == '2')
  {
    socket_fd_ = 1;
    p_dxl_serial = &Serial2;
  }
  else if (port_name[0] == '3')
  {
    socket_fd_ = 2;
    p_dxl_serial = &Serial3;
  }
  else
  {
    socket_fd_ = 0;
    p_dxl_serial = &Serial1;
  }

#if defined(__OPENCM904__)
  drv_dxl_begin(socket_fd_);
#else  
  pin_tx_enable = -1;
#endif
#endif

  setTxDisable();
}

bool PortHandlerArduino::openPort()
{
  port_open_ = true;
#if defined(__OPENCR__)
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);

  setPowerOn();
  delay(1000);
#endif

  return setBaudRate(baudrate_);
}

void PortHandlerArduino::closePort()
{
  port_open_ = false;
  setPowerOff();
}

void PortHandlerArduino::clearPort()
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.flush();
#elif defined(__OPENCM904__) || defined(ARDUINO)
  p_dxl_serial->flush();
#endif
}

void PortHandlerArduino::setPortName(const char *port_name)
{
  strcpy(port_name_, port_name);
}

char *PortHandlerArduino::getPortName()
{
  return port_name_;
}

bool PortHandlerArduino::setBaudRate(const int baudrate)
{
  baudrate_ = checkBaudrateAvailable(baudrate);

  if (baudrate_ == -1)
    return false;

  setupPort(baudrate_);

  return true;
}

int PortHandlerArduino::getBaudRate()
{
  return baudrate_;
}

int PortHandlerArduino::getBytesAvailable()
{
  int bytes_available;

#if defined(__OPENCR__)
  bytes_available = DYNAMIXEL_SERIAL.available();
#elif defined(__OPENCM904__) || defined(ARDUINO)
  bytes_available = p_dxl_serial->available();
#endif

  return bytes_available;
}

int PortHandlerArduino::readPort(uint8_t *packet, int length)
{
  int rx_length;

#if defined(__OPENCR__)
  rx_length = DYNAMIXEL_SERIAL.available();
#elif defined(__OPENCM904__)|| defined(ARDUINO)
  rx_length = p_dxl_serial->available();
#endif

  if (rx_length > length)
    rx_length = length;

  for (int i = 0; i < rx_length; i++)
  {
#if defined(__OPENCR__)
    packet[i] = DYNAMIXEL_SERIAL.read();
#elif defined(__OPENCM904__) || defined(ARDUINO)
    packet[i] = p_dxl_serial->read();
#endif
  }

  return rx_length;
}

int PortHandlerArduino::writePort(uint8_t *packet, int length)
{
  int length_written;

  setTxEnable();

#if defined(__OPENCR__)
  length_written = DYNAMIXEL_SERIAL.write(packet, length);
#elif defined(__OPENCM904__) || defined(ARDUINO)
  length_written = p_dxl_serial->write(packet, length);
#endif

  setTxDisable();

  return length_written;
}

void PortHandlerArduino::setPacketTimeout(uint16_t packet_length)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerArduino::setPacketTimeout(double msec)
{
  packet_start_time_  = getCurrentTime();
  packet_timeout_     = msec;
}

bool PortHandlerArduino::isPacketTimeout()
{
  if (getTimeSinceStart() > packet_timeout_)
  {
    packet_timeout_ = 0;
    return true;
  }

  return false;
}

double PortHandlerArduino::getCurrentTime()
{
	return (double)millis();
}

double PortHandlerArduino::getTimeSinceStart()
{
  double elapsed_time;

  elapsed_time = getCurrentTime() - packet_start_time_;
  if (elapsed_time < 0.0)
    packet_start_time_ = getCurrentTime();

  return elapsed_time;
}

bool PortHandlerArduino::setupPort(int baudrate)
{
#if defined(__OPENCR__)
  DYNAMIXEL_SERIAL.begin(baudrate);
#elif defined(__OPENCM904__)
  p_dxl_serial->setDxlMode(true);
  p_dxl_serial->begin(baudrate);

#elif defined(ARDUINO)      //1
  if (p_dxl_serial == (Stream*)&Serial1) {
    Serial1.begin(baudrate);
#if defined(KINETISK) || defined(__MKL26Z64__)  //2
    if (pin_tx_enable == -1) {
      // BUGBUG:: assumes default USART pins... 
      UART0_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
      CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
      puart_C3_ = &UART0_C3;

    } else {
      Serial1.transmitterEnable(pin_tx_enable);
    }
#elif defined(TEENSYDUINO) // 2
    // Handle on Teensy2...
    if (pin_tx_enable != -1) {
      Serial1.transmitterEnable(pin_tx_enable);
    }
#endif //1
  }  
#ifdef SERIAL_PORT_HARDWARE1  //2
  if (p_dxl_serial == &Serial2) {
    Serial2.begin(baudrate);
#if defined(KINETISK)  || defined(__MKL26Z64__) //3
    if (pin_tx_enable == -1) {
        UART1_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
        CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        puart_C3_ = &UART1_C3;
    } else {
        Serial2.transmitterEnable(pin_tx_enable);
    }

#endif  //2
  }
#endif  //1
#ifdef SERIAL_PORT_HARDWARE2  //2
  if (p_dxl_serial == &Serial3) {
    Serial3.begin(baudrate);
#if defined(KINETISK)  || defined(__MKL26Z64__) //3
    if (pin_tx_enable == -1) {
        UART2_C1 |= UART_C1_LOOPS | UART_C1_RSRC;
        CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3) | PORT_PCR_PE | PORT_PCR_PS; // pullup on output pin;
        puart_C3_ = &UART2_C3;
    } else {
        Serial3.transmitterEnable(pin_tx_enable);
    }
#endif  //2
  }
#endif // 1
#if !defined(TEENSYDUINO) //2
  if (pin_tx_enable != -1) {
      // For other setups...
      pinMode(pin_tx_enable, OUTPUT);
      digitalWrite(pin_tx_enable, LOW);
  }
#endif  //1

#endif  //0

  delay(100);

  tx_time_per_byte = (1000.0 / (double)baudrate) * 10.0;
  return true;
}

int PortHandlerArduino::checkBaudrateAvailable(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return 9600;
    case 57600:
      return 57600;
    case 115200:
      return 115200;
    case 1000000:
      return 1000000;
    case 2000000:
      return 2000000;
    case 3000000:
      return 3000000;
    case 4000000:
      return 4000000;
    case 4500000:
      return 4500000;
    default:
      return -1;
  }
}

void PortHandlerArduino::setPowerOn()
{
#if defined(__OPENCR__)
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
#endif
}

void PortHandlerArduino::setPowerOff()
{
#if defined(__OPENCR__)
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
#endif
}

void PortHandlerArduino::setTxEnable()
{
#if defined(__OPENCR__)
  drv_dxl_tx_enable(TRUE);
#elif defined(__OPENCM904__)
  drv_dxl_tx_enable(socket_fd_, TRUE);
#elif defined(ARDUINO)  
  if (pin_tx_enable != -1) {
#if !defined(TEENSYDUINO)
    digitalWrite(pin_tx_enable, HIGH);
#endif
    return;
  }

#if defined(KINETISK)  || defined(__MKL26Z64__)
  // Teensy 3.X or LC
  *puart_C3_ |=UART_C3_TXDIR;
#elif defined(__AVR__)
  if (p_dxl_serial == (Stream*)&Serial1)
      UCSR1B = /*(1 << UDRIE1) |*/ (1 << TXEN1);
#ifdef SERIAL_PORT_HARDWARE1
  if (p_dxl_serial == (Stream*)&Serial2)
      UCSR2B = /*(1 << UDRIE3) |*/ (1 << TXEN2);
#endif
#ifdef SERIAL_PORT_HARDWARE2
  if (p_dxl_serial == (Stream*)&Serial3)
      UCSR3B =  /*(1 << UDRIE3) |*/ (1 << TXEN3);
#endif
#endif
#endif    
}

void PortHandlerArduino::setTxDisable()
{
#if defined(__OPENCR__)
  drv_dxl_tx_enable(FALSE);
#elif defined(__OPENCM904__)
  drv_dxl_tx_enable(socket_fd_, FALSE);
#elif defined(ARDUINO)  
  if (pin_tx_enable != -1) {
#if !defined(TEENSYDUINO)
    if (port_open_)
      p_dxl_serial->flush();  // make sure stuff is output
    digitalWrite(pin_tx_enable, LOW);

#endif
    return;
  }

#if defined(KINETISK)  || defined(__MKL26Z64__)
  // Teensy 3.X or LC
  if (port_open_) {
    p_dxl_serial->flush();  // make sure stuff is output
    *puart_C3_ &= ~UART_C3_TXDIR;
  }
#elif defined(__AVR__)
  if (port_open_) {
    p_dxl_serial->flush();  // make sure stuff is output
    if (p_dxl_serial == (Stream*)&Serial1)
        UCSR1B = ((1 << RXCIE1) | (1 << RXEN1));
  #ifdef SERIAL_PORT_HARDWARE1
    if (p_dxl_serial == (Stream*)&Serial2)
        UCSR2B = ((1 << RXCIE1) | (1 << RXEN1));
  #endif
  #ifdef SERIAL_PORT_HARDWARE2
    if (p_dxl_serial == (Stream*)&Serial3)
        UCSR3B = ((1 << RXCIE1) | (1 << RXEN1));
  #endif
    }
#endif

#endif
}

bool PortHandlerArduino::setTXEnablePin(int pin)
{
  pin_tx_enable = pin;
  return true;
}

int     PortHandlerArduino::getTXEnablePin()
{
  return pin_tx_enable;
}

#if !defined(__OPENCR__) && !defined(__OPENCM904__)
namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
    while (1) ;   // Does not return...
  }

  void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
    while (1) ;   // Does not return...
  }
}
#endif
#endif
