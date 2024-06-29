/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "can.h"
#include "string.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

#ifdef SUPPORT_CAN2040
/*
    Structure that is used in order to send / receive can packets via the CAN2040
    library
*/
static struct can2040 cbus;
#endif /* SUPPORT_CAN2040 */

#ifdef SUPPORT_MCP2515
/* Object that is used to send can packets via the MCP2515 dev-board
    - The pins are defined as per the schematic of ECU-Emulator
    - SPI0_SPEED is set to 10MHz
    - The MCP2515 is connected to SPI0, because GPIO_CS_MCP2515, GPIO_MOSI_MCP2515,
    GPIO_MISO_MCP2515, GPIO_SCK_MCP2515 are connected to the SPI0 ip of the Raspberry
    Pi Pico
*/
static MCP2515 can0(
        spi0,
        GPIO_CS_MCP2515,
        GPIO_MOSI_MCP2515,
        GPIO_MISO_MCP2515,
        GPIO_SCK_MCP2515,
        SPI0_SPEED
    );
#endif /* SUPPORT_MCP2515 */
////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                  ////
////////////////////////////////////////////////////////////////////////////////

static uint8_t counter_dat_packets = 0U; // Counter for the number of data packets sent

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

#ifdef SUPPORT_CAN2040
//function to read data from the CAN bus
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);

//function to handle interrupts from the PIO
static void PIOx_IRQHandler();

//function to send CAN package
static void can_transmit(struct can2040_msg *outbound);

//function to read CAN packages
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    struct can2040_msg message;
    memcpy(&message, msg, sizeof(message));

    if ((message.id != CAN_PACKET_ECU_1) && (message.id != CAN_PACKET_ECU_2)) \
    {
        printf("CAN message with id %x\n", message.id);
        printf("Data[0] %d\n", message.data[0]);
        printf("\n");
    }

}

static void PIOx_IRQHandler()
{
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup()
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125000000, bitrate = 500e3;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void can_transmit(struct can2040_msg *outbound)
{
    if(can2040_check_transmit(&cbus)) {
        if(can2040_transmit(&cbus, outbound) == 0) {
            //printf("Packet %d was transmitted \n", outbound->id);
        }
    }
    else {
        printf("No space \n");
    }
}
#endif /* SUPPORT_CAN2040 */

#ifdef SUPPORT_MCP2515
void mcp2515_setup()
{
    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
    can0.setNormalMode();
}
#endif /* SUPPORT_MCP2515 */


void send_can(display_data_t *data)
{
    ////////////////////////////////////////////////////////////////////////////
    ////                       DEFINE CAN PACKETS                            ///
    ////////////////////////////////////////////////////////////////////////////
    #ifdef SUPPORT_CAN2040
    struct can2040_msg outbound1;
    outbound1.id = CAN_PACKET_ECU_1;
    outbound1.dlc = 8;

    struct can2040_msg outbound2;
    outbound2.id = CAN_PACKET_ECU_2;
    outbound2.dlc = 8;
    #endif /* SUPPORT_CAN2040 */

    #ifdef SUPPORT_MCP2515
    struct can_frame outbound1;
    outbound1.can_id = CAN_PACKET_ECU_1;
    outbound1.can_dlc = 8;

    struct can_frame outbound2;
    outbound2.can_id = CAN_PACKET_ECU_2;
    outbound2.can_dlc = 8;

    struct can_frame outbound3;
    outbound3.can_id  = CAN_PACKET_VDU;
    outbound3.can_dlc = 8;

    struct can_frame outbound4;
    outbound4.can_id  = CAN_PACKET_PMU_1;
    outbound4.can_dlc = 8;

    struct can_frame outbound5;
    outbound5.can_id  = CAN_PACKET_DAT;
    outbound5.can_dlc = 8;
    #endif /* SUPPORT_MCP2515 */

    ////////////////////////////////////////////////////////////////////////////
    ////                       INITIALIZE CAN PACKET DATA                    ///
    ////////////////////////////////////////////////////////////////////////////
    /* RPM */
    outbound1.data[0] = data->rpm / 100;

    /* CurrGear */
    outbound1.data[1] = data->current_gear;

    /* TPS*/
    outbound1.data[2] = data->tps;

    /* Oil pressure */
    outbound1.data[3] = data->oil_pressure;

    /* Water temperature */
    outbound1.data[4] = data->coolant_temperature;

    /* Fuel rail pressure */
    outbound1.data[5] = data->fuel_pressure;

    /* Lambda */
    outbound1.data[6] = data->lambda;

    /* IAT */
    /* Not shown on display, use rpm as placeholder */
    outbound1.data[7] = data->rpm / 100;

    /* EGT 1 */
    outbound2.data[0] = data->egt[0];

    /* EGT 2 */
    outbound2.data[1] = data->egt[1];

    /* EGT 3 */
    outbound2.data[2] = data->egt[2];

    /* EGT 4 */
    outbound2.data[3] = data->egt[3];

    /* Veh speed */
    /* Not shown on display, use rpm as placeholder */
    outbound2.data[4] = data->rpm / 100;

    /* MAP */
    outbound2.data[5] = data->map;

    /* Brake pressure (raw) */
    outbound2.data[6] = data->brake_pressure_raw;

    /* Oil temperature */
    outbound2.data[7] = data->oil_temperature;

    /* Tyre pressure from VDU */
    outbound3.data[4] = data->rpm/100;
    outbound3.data[5] = data->rpm/100;
    outbound3.data[6] = data->rpm/100;
    outbound3.data[7] = data->rpm/100;

    /* LV battery voltage */
    outbound4.data[0] = data->battery_voltage;
    outbound4.data[1] = data->rpm/100;
    outbound4.data[2] = data->rpm/100;

    /* RTC data from DAT */
    outbound5.data[0] = data->time_hour;
    outbound5.data[1] = data->time_minute;
    outbound5.data[2] = data->time_second;

    ////////////////////////////////////////////////////////////////////////////
    ////                       SEND CAN PACKETS                              ///
    ////////////////////////////////////////////////////////////////////////////
    #ifdef SUPPORT_CAN2040
    can_transmit(&outbound1);

    /* Wait for outbound1 packet to be sent */
    sleep_ms(20);

    can_transmit(&outbound2);
    #endif /* SUPPORT_CAN2040 */

    #ifdef SUPPORT_MCP2515
    if(can0.sendMessage(&outbound1) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound1.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }

    /* Wait for outbound1 packet to be sent */
    sleep_ms(5);

    if(can0.sendMessage(&outbound2) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound2.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }

    /* Wait for outbound2 packet to be sent */
    sleep_ms(5);

    if(can0.sendMessage(&outbound3) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound3.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }

    /* Wait for outbound3 packet to be sent */
    sleep_ms(5);

    if(can0.sendMessage(&outbound4) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound4.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }

    /* Wait for outbound4 packet to be sent */
    sleep_ms(5);
    if(can0.sendMessage(&outbound5) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound5.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }

    if (counter_dat_packets < 10)
    {
        sleep_ms(5);
        if(can0.sendMessage(&outbound5) == MCP2515::ERROR_OK)
        {
            printf("Message sent: %d\n", outbound5.can_id);
        }
        else
        {
            printf("Error sending message\n");
        }
        counter_dat_packets++;
    }

    /* Wait for outbound5 packet to be sent */
    sleep_ms(20);

    if(can0.readMessage(&outbound1) == MCP2515::ERROR_OK)
    {
        printf("New frame from ID: %10x\n", outbound1.can_id);
    }
    #endif /* SUPPORT_MCP2515 */
}