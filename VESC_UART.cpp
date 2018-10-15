#include <VESC_UART.h>

SoftwareSerial SWSerial(14, 12, false, 256);

void VESC_UART::Init() {
	UART.begin(115200);
#ifdef Debug
	DEBUG.begin(115200);
#endif
}

void VESC_UART::Init(int Timeout) {
	ReceiveTimeout = Timeout;
	Init();
}

void VESC_UART::ProcessSuccess() {
	#ifdef Debug
		DEBUG.println("Process success");
	#endif
	DataReady = false;
	RXDataIndex = 0;
}

unsigned short VESC_UART::crc16(unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}

void VESC_UART::ProcessRX() {
	if (RXIndex > 0) {
		byte len = 0;
		if (RX[0] == 0x02) {
			if (RXIndex >= 2) {
				len = RX[1];
				if (RXIndex == 3) {
					if (RX[2] != WaitingID) {
						// Command ID not correct
						#ifdef Debug
							DEBUG.println("Command ID not correct");
						#endif
						ReceiveFailed();
						return;
					}
				}
				if (RXIndex >= 5 + len) {
					if (RX[4 + len] == 0x03) {
						RXDataIndex = 0;
						for (int Index = 2; Index < len + 2; Index++) {
							RXData[Index - 2] = RX[Index];
							RXDataIndex++;
						}
						unsigned short CheckSum = crc16(RXData, len);
						unsigned short ReceivedSum = RX[2 + len];
						ReceivedSum = ReceivedSum << 8;
						ReceivedSum |= RX[3 + len];
						if (ReceivedSum == CheckSum) {
							ReceiveSuccess();
						} else {
							// Failed checksum
							#ifdef Debug
								DEBUG.println("Failed checksum");
								DEBUG.println(CheckSum);
								DEBUG.println(ReceivedSum);
							#endif
							ReceiveFailed();
							IsWaiting = false;
							return;
						}
					} else {
						// Last byte not correct
						#ifdef Debug
							DEBUG.println("Last byte not correct");
						#endif
						ReceiveFailed();
						IsWaiting = false;
						return;
					}
				}
			}
		} else if (RX[0] == 0x03) {
			// Not implemented yet
			#ifdef Debug
				DEBUG.println("Not implemented");
			#endif
		} else {
			// First byte not correct
			#ifdef Debug
				DEBUG.println("First byte not correct");
			#endif
			ReceiveFailed();
			return;
		}
	}
}

void VESC_UART::ProcessReceive() {
	if (IsWaiting) {
		RX[RXIndex++] = UART.read();
		LastReceived = millis();
		ProcessRX();
	}
}

void VESC_UART::ReceiveFailed() {
	#ifdef Debug
		DEBUG.println("Receive failed");
	#endif
	//IsWaiting = false;
	RXIndex = 0;
}

void VESC_UART::ReceiveSuccess() {
	#ifdef Debug
		DEBUG.println("Receive success");
	#endif
	IsWaiting = false;
	RXIndex = 0;
	DataReady = true;
}

bool VESC_UART::Handle() {
	if (UART.available() > 0) {
		ProcessReceive();
	}
	if (IsWaiting) {
		if (millis() - LastReceived > ReceiveTimeout) {
			// Timeout
			#ifdef Debug
				DEBUG.println("Timeout");
			#endif
			ReceiveFailed();
			IsWaiting = false;
		}
	}
	if (DataReady) {
		return true;
	}
	return false;
}

void VESC_UART::Send(byte *buf, unsigned int len) {
	if (!IsWaiting) {
		unsigned short sum = crc16(buf, len);
		byte first = sum >> 8;
		byte second = sum;
		UART.write(0x02);
		UART.write(len);
		UART.write(buf, len);
		UART.write(first);
		UART.write(second);
		UART.write(0x03);
		LastReceived = millis();
		IsWaiting = true;
		WaitingID = buf[0];
	}
}

void VESC_UART::ClearData() {
	RXDataIndex = 0;
	DataReady = false;
}