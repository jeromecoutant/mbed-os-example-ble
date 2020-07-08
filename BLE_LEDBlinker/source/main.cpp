/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/gap/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
#include "pretty_printer.h"
#include "shci.h"
#include "app_conf.h"

//InterruptIn button2(BUTTON2);
InterruptIn button3(BUTTON3);

const static char PEER_NAME[] = "LED";

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic led_characteristic;
static bool trigger_led_characteristic = false;

void printpad(uint8_t num)
{
    if(num < 16) {
        printf("0");
        printf("%X", num);
    } else
    {
        printf("%02X", num);
    }
    
}

#define print_array(_arr, _size, big_end) print_array___(_arr, _size, big_end, #_arr)
void print_array___(uint8_t* arr, size_t size, bool big_end, const char *name) {
    printf("vvvvvvvvvv %s vvvvvvvvvv\n\t", name);
    for (size_t i=0; i < size; i+=4)
    {
        if (i && ((i % 16) == 0)){
            printf("\n\t");
        }
        else if (i && ((i % 4) == 0)){
            printf(" ");
        }
        printf("0x");
        if( big_end ) {           
            printpad(arr[i]);
            printpad(arr[i+1]);
            printpad(arr[i+2]);
            printpad(arr[i+3]);
        } else {
            printpad(arr[i+3]);
            printpad(arr[i+2]);
            printpad(arr[i+1]);
            printpad(arr[i]);
        }
    }
    if(big_end)
        printf("\n big endian presentation--------%s----\n", name);
    else
        printf("\n little endian presentation-----%s-------\n", name);
}

void service_discovery(const DiscoveredService *service) {
    if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
        printf("S UUID-%x attrs[%u %u]\r\n", service->getUUID().getShortUUID(), service->getStartHandle(), service->getEndHandle());
    } else {
        printf("S UUID-");
        const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
        for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
            printf("%02x", longUUIDBytes[i]);
        }
        printf(" attrs[%u %u]\r\n", service->getStartHandle(), service->getEndHandle());
    }
}

void update_led_characteristic(void) {
    if (!BLE::Instance().gattClient().isServiceDiscoveryActive()) {
        led_characteristic.read();
    }
}

void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    printf("  C UUID-%x valueAttr[%u] props[%x]\r\n", characteristicP->getUUID().getShortUUID(), characteristicP->getValueHandle(), (uint8_t)characteristicP->getProperties().broadcast());
    if (characteristicP->getUUID().getShortUUID() == 0xa001) { /* !ALERT! Alter this filter to suit your device. */
        led_characteristic        = *characteristicP;
        trigger_led_characteristic = true;
    }
}

void discovery_termination(ble::connection_handle_t connectionHandle) {
    printf("terminated SD for handle %u\r\n", connectionHandle);
    if (trigger_led_characteristic) {
        trigger_led_characteristic = false;
        event_queue.call(update_led_characteristic);
    }
}

void trigger_toggled_write(const GattReadCallbackParams *response) {
    if (response->handle == led_characteristic.getValueHandle()) {
        printf("trigger_toggled_write: handle %u, offset %u, len %u\r\n", response->handle, response->offset, response->len);
        for (unsigned index = 0; index < response->len; index++) {
            printf("%c[%02x]", response->data[index], response->data[index]);
        }
        printf("\r\n");

        uint8_t toggledValue = response->data[0] ^ 0x1;
        led_characteristic.write(1, &toggledValue);
    }
}

void trigger_read(const GattWriteCallbackParams *response) {
    if (response->handle == led_characteristic.getValueHandle()) {
        led_characteristic.read();
    }
}

volatile int _fus_option = 0;


class LEDBlinkerDemo : ble::Gap::EventHandler {
public:
    LEDBlinkerDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _alive_led(LED1, 1),
        _actuated_led(LED2, 0),
        _is_connecting(false){ }

    ~LEDBlinkerDemo() { }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &LEDBlinkerDemo::on_init_complete);
    
        //SHCI_C2_FUS_StartWs();

        _event_queue.call_every(500, this, &LEDBlinkerDemo::blink);
      
       _event_queue.dispatch_forever();
    }

private:

/*    int run_fus_install()
    {
        uint8_t fus_state_value;

        if(CFG_OTA_REBOOT_VAL_MSG == CFG_REBOOT_ON_FW_APP) {
            printf("wireless FW is running\n");
            return 0;
        } 
        
        printf("FUS FW is running\n");
        if(CFG_OTA_REBOOT_VAL_MSG == CFG_REBOOT_FUS_FW_UPGRADE) {
            printf("Call SHCI_C2_FUS_FwUpgrade\n");
            CFG_OTA_REBOOT_VAL_MSG = CFG_REBOOT_ON_CPU2_UPGRADE;
            SHCI_C2_FUS_FwUpgrade(0, 0);
            while(1);            
        }

        if(CFG_OTA_REBOOT_VAL_MSG == CFG_REBOOT_ON_CPU2_UPGRADE) {
            fus_state_value = SHCI_C2_FUS_GetState(NULL);
            printf("fus_state_value %d\n", fus_state_value);
            if(fus_state_value == 0xFF)
            {
                printf("Error FUS, should not get here\n");
                while(1);
            }

            if(fus_state_value != 0)
            {
                printf("FUS is upgrading\n");
                while(1);
            }

            if( fus_state_value == 0)
            {
                printf("FUS is done installation, start BLE FW\n");
                CFG_OTA_REBOOT_VAL_MSG = CFG_REBOOT_ON_FW_APP;
                SHCI_C2_FUS_StartWs();
                while(1);
            }
        }

        return 0;
    }
*/

    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.\n");
            return;
        }
        
        print_mac_address();
        printf("press button near blinking led to start FUS\n");  

        _ble.gattClient().onDataRead(trigger_toggled_write);
        _ble.gattClient().onDataWritten(trigger_read);

        ble::ScanParameters scan_params;
        _ble.gap().setScanParameters(scan_params);
        _ble.gap().startScan();
        
    }

    void blink() {
        _alive_led = !_alive_led;        

        //print_array((uint8_t*)CFG_OTA_REBOOT_VAL_MSG, 0x200, false);

        if( _fus_option == 3) {       
          printf("starting FUS\n");  
          _fus_option = 0;
          printf("one\n");  
          SHCI_C2_FUS_GetState(NULL);
          printf("two\n");  
          SHCI_C2_FUS_GetState(NULL);
          printf("while\n");  
          while(1) {
              //printf("restart\n");  
          };
        } 
        
/*        if( _fus_option == 2) {
            printf("start upgrade%d\n", _fus_option); 
            _fus_option = 0;            
            run_fus_install();
        }
*/
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startScan();
        _is_connecting = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        if (event.getOwnRole() == ble::connection_role_t::CENTRAL) {
            _ble.gattClient().onServiceDiscoveryTermination(discovery_termination);
            _ble.gattClient().launchServiceDiscovery(
                event.getConnectionHandle(),
                service_discovery,
                characteristic_discovery,
                0xa000,
                0xa001
            );
        } else {
            _ble.gap().startScan();
        }
        _is_connecting = false;
    }

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        /* don't bother with analysing scan result if we're already connecting */
        if (_is_connecting) {
            return;
        }

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();

            /* connect to a discoverable device */
            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                field.value.size() == strlen(PEER_NAME) &&
                (memcmp(field.value.data(), PEER_NAME, field.value.size()) == 0)) {

                printf("Adv from: ");
                print_address(event.getPeerAddress().data());
                printf(" rssi: %d, scan response: %u, connectable: %u\r\n",
                       event.getRssi(), event.getType().scan_response(), event.getType().connectable());

                ble_error_t error = _ble.gap().stopScan();

                if (error) {
                    print_error(error, "Error caused by Gap::stopScan");
                    return;
                }

                const ble::ConnectionParameters connection_params;

                error = _ble.gap().connect(
                    event.getPeerAddressType(),
                    event.getPeerAddress(),
                    connection_params
                );

                if (error) {
                    _ble.gap().startScan();
                    return;
                }

                /* we may have already scan events waiting
                 * to be processed so we need to remember
                 * that we are already connecting and ignore them */
                _is_connecting = true;

                return;
            }
        }
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _alive_led;
    DigitalOut _actuated_led;
    bool _is_connecting;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

/*
void flip2()
{
    _fus_option = 2;
}*/

void flip3()
{
    _fus_option = 3;
}

int main()
{
    //button2.fall(&flip2);  // attach the address of the flip function to the rising edge
    button3.fall(&flip3);
    
    //printf("CFG_OTA_REBOOT_VAL_MSG %d\n", CFG_OTA_REBOOT_VAL_MSG);
    //CFG_OTA_REBOOT_VAL_MSG = CFG_REBOOT_ON_CPU2_UPGRADE;
    //printf("CFG_OTA_REBOOT_VAL_MSG again %d\n", CFG_OTA_REBOOT_VAL_MSG);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    LEDBlinkerDemo demo(ble, event_queue);
    demo.start();
    
    return 0;
}
