// Load the USB Host System core
#define LOAD_USB_HOST_SYSTEM
// Load USB Host Shield
#define LOAD_USB_HOST_SHIELD
// Use USB hub, you might want this for multiple devices.
#define LOAD_UHS_HUB

// Patch printf so we can use it.
#define LOAD_UHS_PRINTF_HELPER
#define DEBUG_PRINTF_EXTRA_HUGE 0
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HID 1

#define LOAD_UHS_HID

#include <Arduino.h>
#ifdef true
#undef true
#endif
#ifdef false
#undef false
#endif

#include "UHS_host.h"

volatile bool NumLock = true;
volatile bool CapsLock = false;
volatile bool ScrollLock = false;

volatile uint8_t led_states = 0b1;
volatile uint8_t led_states_prev = 0;
uint8_t led_states_prev2 = 0;
UHS_HID_base *dev_kb0;

unsigned millis0;

class myHID_processor : public UHS_HID_PROCESSOR {
public:
  myHID_processor(void) {}

  void onRelease(UHS_HID_base *d) {
    printf_P(PSTR("HID driver type %d no longer available.\r\n"), d->driver);
  }
  void onStart(UHS_HID_base *d) {
    printf_P(PSTR("HID driver type %d started, Subclass %02x, Protocol %02x\r\n"), d->driver, d->parent->bSubClass, d->parent->bProtocol);
  }
  void onPoll(UHS_HID_base *d, uint8_t *data, uint16_t length) {
    switch (d->driver) {
      case UHS_HID_raw:
        printf_P(PSTR("RAW input %d bytes interface %d, Subclass %02x, Protocol %02x Data:"), length, d->parent->bIface, d->parent->bSubClass, d->parent->bProtocol);
        for (int i = 0; i < length; i++) {
          printf_P(PSTR(" %02x"), data[i]);
        }

        if (d->parent->bProtocol == 1) {
          dev_kb0 = d;

          if ((length == 8) && (data[0] == 0) && (data[1] == 0) && (data[3] == 0) && (data[4] == 0) && (data[5] == 0) && (data[6] == 0) && (data[7] == 0)) {

            if (data[2] == 0x53) {
              if (NumLock == false) {
                led_states |= 0b1;

                NumLock = true;
              } else {
                led_states &= ~0b1;

                NumLock = false;
              }
            } else if (data[2] == 0x39) {
              if (CapsLock == false) {
                led_states |= 0b10;

                CapsLock = true;
              } else {
                led_states &= ~0b10;

                CapsLock = false;
              }
            } else if (data[2] == 0x47) {
              if (ScrollLock == false) {
                led_states |= 0b100;

                ScrollLock = true;
              } else {
                led_states &= ~0b100;

                ScrollLock = false;
              }
            }
          }

          if (led_states_prev != led_states) {
            led_states_prev = led_states;

            uint8_t rv;
            d->parent->pUsb->DisablePoll();
            rv = d->parent->SetReport(d->parent->bIface, 2, 0, 1, &led_states);
            d->parent->pUsb->EnablePoll();
          }

          printf_P(PSTR(" %02x"), led_states);
        }
        
        printf_P(PSTR("\r\n"));
        
        break;
      default:
        break;
    }
  }
};

myHID_processor HID_processor1;
myHID_processor HID_processor2;
MAX3421E_HOST UHS_Usb;
UHS_USBHub hub_1(&UHS_Usb);
UHS_HID hid1(&UHS_Usb, &HID_processor1);
UHS_HID hid2(&UHS_Usb, &HID_processor2);

void setup() {
  USB_HOST_SERIAL.begin(115200);
  while (UHS_Usb.Init(1000) != 0)
    ;
  printf_P(PSTR("\r\nHID RAW demo Begin.\r\n"));
}

void loop() {
  if ((millis() - millis0) > 500) {
    millis0 = millis();

    if (dev_kb0->parent->bProtocol == 1) {
      uint8_t rv;
      dev_kb0->parent->pUsb->DisablePoll();
      rv = dev_kb0->parent->SetReport(dev_kb0->parent->bIface, 2, 0, 1, &led_states);
      dev_kb0->parent->pUsb->EnablePoll();

      //      uint8_t led_tmp;
      //      dev_kb0->parent->pUsb->DisablePoll();
      //      rv = dev_kb0->parent->GetReport(dev_kb0->parent->bIface, 2, 0, 1, &led_tmp);
      //      dev_kb0->parent->pUsb->EnablePoll();
      //
      //      printf_P(PSTR("GetReport Data: %02x\r\n"), led_tmp);
    }
  }

  delay(1);
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if ((inChar >= '0') && (inChar <= '7')) {
      led_states = (inChar - 48) & 0b111;

      NumLock = led_states & 1;

      CapsLock = (led_states >> 1) & 1;

      ScrollLock = (led_states >> 2) & 1;
    }
  }
}
