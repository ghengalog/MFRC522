/*
 * RFID.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * Based on code Dr.Leong   ( WWW.B2CQSHOP.COM )
 * Created by Miguel Balboa, Jan, 2012.
 * Released into the public domain.
 *
 */


// Includes
#include <Arduino.h>
#include <MFRC522.h>

MFRC522::MFRC522(int sad, int reset) {
  _sad = sad;
  pinMode(_sad, OUTPUT);                  // Set digital as OUTPUT to connect it to the RFID /ENABLE pin
  digitalWrite(_sad, LOW);


  _reset = reset;
  pinMode(_reset,OUTPUT);                 // Set digital pin, Not Reset and Power-down
  digitalWrite(_reset, HIGH);

}
/******************************************************************************
 * User API
 ******************************************************************************/

bool MFRC522::isCard() {
  uint8_t status;
  uint8_t str[MAX_LEN];

  status = requestCard(MF_REQIDL, str);
  if (status == MI_OK) {
    return true;
  } else {
    return false;
  }
}

bool MFRC522::readCardSerial() {
  uint8_t status;
  uint8_t serial[MAX_LEN];

  // Anti-collision, returns card serial number. 4 bytes.
  status = anticollision(serial);
  memcpy(_serial, serial, 5);

  return status == MI_OK;
}

uint8_t MFRC522::getFirmwareVersion() {
  uint8_t response;
  response = readFromRegister(VersionReg);
  return response;
}


void MFRC522::begin() {
  digitalWrite(_sad, HIGH);

  reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  writeToRegister(TModeReg, 0x8D);		// Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  writeToRegister(TPrescalerReg, 0x3E);         // TModeReg[3..0] + TPrescalerReg
  writeToRegister(TReloadRegL, 30);
  writeToRegister(TReloadRegH, 0);

  writeToRegister(TxAutoReg, 0x40);	        // 100%ASK
  writeToRegister(ModeReg, 0x3D);		// CRC initial value 0x6363

  antennaOn();
}

void MFRC522::reset() {
  writeToRegister(CommandReg, MFRC522_SOFTRESET);
}

void MFRC522::writeToRegister(uint8_t addr, uint8_t val) {
	digitalWrite(_sad, LOW);

	//Address format: 0XXXXXX0
	SPI.transfer((addr<<1)&0x7E);
	SPI.transfer(val);

	digitalWrite(_sad, HIGH);
}

void MFRC522::antennaOn() {
  uint8_t temp;

  temp = readFromRegister(TxControlReg);
  if (!(temp & 0x03)) {
    setBitMask(TxControlReg, 0x03);
  }
}

/*
 *  Read_MFRC522 Nombre de la función: Read_MFRC522
 *  Descripción: Desde el MFRC522 leer un byte de un registro de datos
 *  Los parámetros de entrada: addr - la dirección de registro
 *  Valor de retorno: Devuelve un byte de datos de lectura
 */
uint8_t MFRC522::readFromRegister(uint8_t addr) {
  uint8_t val;
  digitalWrite(_sad, LOW);
  SPI.transfer(((addr<<1)&0x7E) | 0x80);
  val =SPI.transfer(0x00);
  digitalWrite(_sad, HIGH);
  return val;
}

void MFRC522::setBitMask(uint8_t reg, uint8_t mask) {
  uint8_t current;
  current = readFromRegister(reg);
  writeToRegister(reg, current | mask);
}

void MFRC522::clearBitMask(uint8_t reg, uint8_t mask) {
  uint8_t current;
  current = readFromRegister(reg);
  writeToRegister(reg, current & (~mask));
}

/*
 * Method name: calculateCRC
 * Description:
 *     Sends data to the MFRC522 to calculate the CRC value.
 * Input parameters:
 *     data   -- data to send to the MFRC522.
 *     len    -- length of the data.
 *     output -- Result from the CRC calculation.
 */
void MFRC522::calculateCRC(uint8_t *data, uint8_t len, uint8_t *result) {
  uint8_t i, n;

  clearBitMask(DivIrqReg, 0x04);			// CRCIrq = 0
  setBitMask(FIFOLevelReg, 0x80);			// Clear the FIFO pointer

  //Writing data to the FIFO.
  for (i=0; i<len; i++) {
    writeToRegister(FIFODataReg, *(data+i));
  }
  writeToRegister(CommandReg, MFRC522_CALCCRC);

  // Wait for the CRC calculation to complete.
  i = 0xFF;
  do {
    n = readFromRegister(DivIrqReg);
    i--;
  } while ((i!=0) && !(n&0x04));			//CRCIrq = 1

  // Read the result from the CRC calculation.
  result[0] = readFromRegister(CRCResultRegL);
  result[1] = readFromRegister(CRCResultRegM);
}

uint8_t MFRC522::commandCard(uint8_t cmd, uint8_t *data, uint8_t dlen, uint8_t *result, uint8_t *rlen) {
  uint8_t status = MI_ERR;
  uint8_t irqEn = 0x00;
  uint8_t waitIRq = 0x00;
  uint8_t lastBits;
  uint8_t n;
  uint8_t i;

  switch (cmd) {
  case MFRC522_AUTHENT:
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
  case MFRC522_TRANSCEIVE:
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
  default:
    break;
  }

  writeToRegister(CommIEnReg, irqEn|0x80);	// interrupt request
  clearBitMask(CommIrqReg, 0x80);             // Clear all interrupt requests bits.
  setBitMask(FIFOLevelReg, 0x80);             // FlushBuffer=1, FIFO initialization.

  writeToRegister(CommandReg, MFRC522_IDLE);	//No action, cancel the current command.

  // Write to FIFO
  for (i=0; i<dlen; i++) {
    writeToRegister(FIFODataReg, data[i]);
  }

  // Execute the command.
  writeToRegister(CommandReg, cmd);
  if (cmd == MFRC522_TRANSCEIVE) {
    setBitMask(BitFramingReg, 0x80);		//StartSend=1, transmission of data starts
  }

  // Waiting for the command to complete so we can receive data.
  i = 25;	//According to the clock freq. adjustment, the max. wait time is 25ms.
  do {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    delay(1);
    n = readFromRegister(CommIrqReg);
    i--;
  } while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  clearBitMask(BitFramingReg, 0x80);			//StartSend=0

  if (i != 0) {
    if(!(readFromRegister(ErrorReg) & 0x1B)) {   //BufferOvfl Collerr CRCErr ProtecolErr
      status = MI_OK;
      if (n & irqEn & 0x01) {
        status = MI_NOTAGERR;
      }

      if (cmd == MFRC522_TRANSCEIVE) {
        n = readFromRegister(FIFOLevelReg);
        lastBits = readFromRegister(ControlReg) & 0x07;
        if (lastBits) {
          *rlen = (n-1)*8 + lastBits;
        } else {
          *rlen = n*8;
        }

        if (n == 0) {
          n = 1;
        }

        if (n > MAX_LEN) {
          n = MAX_LEN;
        }

        // Reading the recieved data in FIFO.
        for (i=0; i<n; i++) {
          result[i] = readFromRegister(FIFODataReg);
        }
      }
    } else {
      status = MI_ERR;
    }
  }

  return status;
}

/*
 * Function Name: requestCard
 * Description:
 *    Checks if there are any cards, returns card type.
 * Input parameters:
 *    mode - Request mode.
 *    type - Card type.
 *              0x4400 = Mifare_UltraLight
 *		0x0400 = Mifare_One(S50)
 *		0x0200 = Mifare_One(S70)
 *		0x0800 = Mifare_Pro(X)
 *		0x4403 = Mifare_DESFire
 * Return value: On success: MI_OK
 */
uint8_t  MFRC522::requestCard(uint8_t mode, uint8_t *type) {
  uint8_t status;
  uint8_t result;			       //   The returned bits.

  writeToRegister(BitFramingReg, 0x07);  // TxLastBists = BitFramingReg[2..0]

  type[0] = mode;
  status = commandCard(MFRC522_TRANSCEIVE, type, 1, type, &result);

  if ((status != MI_OK) || (result != 0x10)) {
    status = MI_ERR;
  }

  return status;
}

/*
 * Method name: anticollision
 * Description: Anti-collision detection, reading selected card serial number.
 * Input parameters:
 *    serial - returns 4 bytes card serial number,
               the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
uint8_t MFRC522::anticollision(uint8_t *serial) {
  uint8_t status;
  uint8_t i;
  uint8_t check = 0;
  uint8_t len;

  writeToRegister(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

  serial[0] = MF_ANTICOLL;
  serial[1] = 0x20;
  status = commandCard(MFRC522_TRANSCEIVE, serial, 2, serial, &len);

  if (status == MI_OK) {
    // Check card serial number.
    for (i=0; i<4; i++) {
      check ^= serial[i];
    }
    if (check != serial[i]) {
      status = MI_ERR;
    }
  }

  return status;
}

/*
 * Method name: authenticate
 * Description:
 *    Verify card password
 * Input parameters:
 *    mode   -- Password Authentication Mode
 *              KEY_A = Authentication Key A
 *              KEY_B = Authentication Key B
 *    block  -- Block address
 *    sector -- Sector password
 *    serial -- Card serial number, 4-byte
 * Return value: Returns MI_OK on success.
 */
uint8_t MFRC522::authenticate(uint8_t mode, uint8_t block, uint8_t *key, uint8_t *serial)
{
    uint8_t status;
    uint8_t result;
    uint8_t i;
    uint8_t buffer[12];

	//Verify the command block address + sector + password + card serial number
    buffer[0] = mode;
    buffer[1] = block;
    for (i=0; i<6; i++) {
      buffer[i+2] = *(key+i);
    }
    for (i=0; i<4; i++) {
      buffer[i+8] = *(serial+i);
    }

    status = commandCard(MFRC522_AUTHENT, buffer, 12, buffer, &result);

    if ((status != MI_OK) || (!(readFromRegister(Status2Reg) & 0x08))) {
      status = MI_ERR;
    }

    return status;
}

/*
 * Method name: readFromCard
 * Description: Read block data
 * Input parameters:
 *     block  -- block address
 *     result -- read block data
 * Return value: the successful return MI_OK
 */
uint8_t MFRC522::readFromCard(uint8_t block, uint8_t *result) {
  uint8_t status;
  uint8_t len;

  result[0] = MF_READ;
  result[1] = block;
  calculateCRC(result, 2, &result[2]);
  status = commandCard(MFRC522_TRANSCEIVE, result, 4, result, &len);

  if ((status != MI_OK) || (len != 0x90)) {
    status = MI_ERR;
  }

  return status;
}

/*
 * Method name: writeToChar
 * Description: Write block data to card.
 * Input parameters:
 *     block - block address
 *     data  - 16 byte of data to write to the block on the card.
 * Return value: the successful return MI_OK
 */
uint8_t MFRC522::writeToCard(uint8_t block, uint8_t *data) {
  uint8_t status;
  uint8_t result;
  uint8_t i;
  uint8_t buffer[18];

  buffer[0] = MF_WRITE;
  buffer[1] = block;
  calculateCRC(buffer, 2, &buffer[2]);
  status = commandCard(MFRC522_TRANSCEIVE, buffer, 4, buffer, &result);

    if ((status != MI_OK) || (result != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
      status = MI_ERR;
    }

    if (status == MI_OK) {
      for (i=0; i<16; i++) {		//?FIFO?16Byte?? Datos a la FIFO 16Byte escribir
        buffer[i] = *(data+i);
      }
      calculateCRC(buffer, 16, &buffer[16]);
        status = commandCard(MFRC522_TRANSCEIVE, buffer, 18, buffer, &result);

        if ((status != MI_OK) || (result != 4) || ((buffer[0] & 0x0F) != 0x0A)) {
          status = MI_ERR;
        }
    }

    return status;
}

/*
 * Method name: halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522::halt() {
  uint8_t status;
  uint8_t len;
  uint8_t buffer[4];

  buffer[0] = MF_HALT;
  buffer[1] = 0;
  calculateCRC(buffer, 2, &buffer[2]);

  status = commandCard(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
}
