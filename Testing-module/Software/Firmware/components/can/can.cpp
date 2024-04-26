/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "inc/can.h"
#include "string.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

//static struct can2040 cbus;

static MCP2515 can0(spi0, 5, 3, 4, 2, 10000000);


////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

// //function to read data from the CAN bus
// static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);

// //function to handle interrupts from the PIO
// static void PIOx_IRQHandler();

// //function to send CAN package
// static void can_transmit(struct can2040_msg *outbound);

// //function to read CAN packages
// static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
// {
//     struct can2040_msg message;
//     memcpy(&message, msg, sizeof(message));

//     if ((message.id != CAN_PACKET_ECU_1) && (message.id != CAN_PACKET_ECU_2)) \
//     {
//         printf("CAN message with id %x\n", message.id);
//         printf("Data[0] %d\n", message.data[0]);
//         printf("\n");
//     }

// }

// static void PIOx_IRQHandler()
// {
//     can2040_pio_irq_handler(&cbus);
// }

// void canbus_setup()
// {
//     uint32_t pio_num = 0;
//     uint32_t sys_clock = 125000000, bitrate = 500e3;
//     uint32_t gpio_rx = 4, gpio_tx = 5;

//     // Setup canbus
//     can2040_setup(&cbus, pio_num);
//     can2040_callback_config(&cbus, can2040_cb);

//     // Enable irqs
//     irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
//     NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
//     NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

//     // Start canbus
//     can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
// }

// void can_transmit(struct can2040_msg *outbound)
// {
//     if(can2040_check_transmit(&cbus)) {
//         if(can2040_transmit(&cbus, outbound) == 0) {
//             //printf("Packet %d was transmitted \n", outbound->id);
//         }
//     }
//     else {
//         printf("No space \n");
//     }
// }

void mcp2515_setup()
{
    can0.reset();
    can0.setBitrate(CAN_500KBPS, MCP_8MHZ);
    can0.setNormalMode();
}


void send_can(uint8_t *data)
{
    //struct can2040_msg outbound;
    struct can_frame outbound;

    /* FORMAT THE CAN_PACKET_ECU_1 as in CAN Packet allocation */
    outbound.can_id = CAN_PACKET_ECU_1;
    outbound.can_dlc = 8;

    /* RPM */
    outbound.data[0] = ((((*(data) << 8) & 0xFF00) + *(data + 1)) / 100);

    /* CurrGear */
    outbound.data[1] = *(data + 2);

    /* TPS*/
    outbound.data[2] = *(data + 3);

    /* Oil pressure */
    outbound.data[3] = *(data + 4);

    /* Water temperature */
    outbound.data[4] = *(data + 5);

    /* Fuel rail pressure */
    outbound.data[5] = *(data + 6);

    /* Lambda */
    outbound.data[6] = *(data + 7);

    /* IAT */
    outbound.data[7] = *(data + 8);
    //can_transmit(&outbound);
    if(can0.sendMessage(&outbound) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }
    sleep_ms(20);

    /* FORMAT THE CAN_PACKET_ECU_2 as in CAN Packet allocation */
    outbound.can_id = CAN_PACKET_ECU_2;
    outbound.can_dlc = 8;

    /* EGT 1 */
    outbound.data[0] = *(data + 9);

    /* EGT 2 */
    outbound.data[1] = *(data + 10);

    /* EGT 3 */
    outbound.data[2] = *(data + 11);

    /* EGT 4 */
    outbound.data[3] = *(data + 12);

    /* Veh speed */
    outbound.data[4] = *(data + 13);

    /* MAP */
    outbound.data[5] = *(data + 14);

    /* Brake pressure (raw) */
    outbound.data[6] = *(data + 15);

    /* Oil temperature */
    outbound.data[7] = *(data + 16);

    //can_transmit(&outbound);
    if(can0.sendMessage(&outbound) == MCP2515::ERROR_OK)
    {
        printf("Message sent: %d\n", outbound.can_id);
    }
    else
    {
        printf("Error sending message\n");
    }
    sleep_ms(10);

    if(can0.readMessage(&outbound) == MCP2515::ERROR_OK)
    {
        printf("New frame from ID: %10x\n", outbound.can_id);
    }
}