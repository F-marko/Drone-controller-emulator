/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#define DRONE_BIND_COUNT       1000
#define DRONE_PACKET_PERIOD    2000
#define DRONE_PACKET_SIZE      15
#define DRONE_RF_NUM_CHANNELS  4
#define DRONE_RF_BIND_CHANNEL  0
#define DRONE_ADDRESS_LENGTH   5

static uint8_t Drone_rf_chan;
static uint8_t Drone_rf_channels[DRONE_RF_NUM_CHANNELS] = {0,};
static uint8_t Drone_rx_tx_addr[DRONE_ADDRESS_LENGTH];

enum{
    // flags going to packet[2]
    DRONE_FLAG_RTH      = 0x01,
    DRONE_FLAG_HEADLESS = 0x02,
    DRONE_FLAG_FLIP     = 0x08,
    DRONE_FLAG_VIDEO    = 0x10,
    DRONE_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    DRONE_FLAG_INVERT   = 0x80,
};

uint32_t process_Drone()
{
    uint32_t timeout = micros() + DRONE_PACKET_PERIOD;
    Drone_send_packet(0);
    return timeout;
}

void Drone_init()
{
    uint8_t i;
    const u8 bind_address[] = {0,0,0,0,0};
    for(i=0; i<DRONE_ADDRESS_LENGTH; i++) {
        Drone_rx_tx_addr[i] = random() & 0xff;
    }
    Drone_rf_channels[0] = 0x00;
    for(i=1; i<DRONE_RF_NUM_CHANNELS; i++) {
        Drone_rf_channels[i] = random() % 0x42;
    }
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    XN297_SetTXAddr(bind_address, DRONE_ADDRESS_LENGTH);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);
}

void Drone_bind()
{
    uint16_t counter = DRONE_BIND_COUNT;
    while(counter) {
        Drone_send_packet(1);
        delayMicroseconds(DRONE_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    XN297_SetTXAddr(Drone_rx_tx_addr, DRONE_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))

void Drone_send_packet(u8 bind)
{
    union {
        u16 value;
        struct {
            u8 lsb;
            u8 msb;
        } bytes;
    } chanval;

    if (bind) {
        packet[0] = 0xa4;
        memcpy(&packet[1], Drone_rx_tx_addr, 5);
        memcpy(&packet[6], Drone_rf_channels, 4);
        packet[10] = transmitterID[0];
        packet[11] = transmitterID[1];
    } else {
        packet[0] = 0xa5;
        packet[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
        packet[2] = GET_FLAG(AUX2, DRONE_FLAG_FLIP)
                  | GET_FLAG(AUX5, DRONE_FLAG_HEADLESS)
                  | GET_FLAG(AUX6, DRONE_FLAG_RTH)
                  | GET_FLAG(AUX3, DRONE_FLAG_SNAPSHOT)
                  | GET_FLAG(AUX4, DRONE_FLAG_VIDEO);
        packet[3] = GET_FLAG(AUX1, DRONE_FLAG_INVERT);
        chanval.value = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 0x3ff);   // aileron
        packet[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[5] = chanval.bytes.lsb;
        chanval.value = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 0x3ff);   // elevator
        packet[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[7] = chanval.bytes.lsb;
        chanval.value = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 0x3ff);   // throttle
        packet[8] = chanval.bytes.msb + 0x7c;
        packet[9] = chanval.bytes.lsb;
        chanval.value = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 0x3ff);   // rudder
        packet[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[11] = chanval.bytes.lsb;
    }
    packet[12] = transmitterID[2];
    packet[13] = 0x0a;
    packet[14] = 0;
    for(uint8_t i=0; i<DRONE_PACKET_SIZE-1; i++) {
        packet[14] += packet[i];
    }
    
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? DRONE_RF_BIND_CHANNEL : Drone_rf_channels[Drone_rf_chan++]);
    Drone_rf_chan %= sizeof(Drone_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WritePayload(packet, DRONE_PACKET_SIZE);
}
