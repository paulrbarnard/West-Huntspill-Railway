#include "myEEPROM.h"


void myEEPROM::write(ushort addr, byte data){
	put(addr, data);
}

byte myEEPROM::read(ushort addr){
	return get(addr);
}

void myEEPROM::writePage(byte array[],ushort size)
{
	writePage(0,array,size);
}
void myEEPROM::writePage(ushort addr, byte array[],ushort size)
{
	int i=0;
	while(size-EEPROM_BATCH_WRITE>0)
	{
		Wire.beginTransmission(AT24C256B_ADDR);
		Wire.write(addr>>8); // addr
		Wire.write(addr&0xFF); // addr
		Wire.write(array+i*EEPROM_BATCH_WRITE,EEPROM_BATCH_WRITE);
		Wire.endTransmission();
		delay(EEPROM_DELAY);
		i++;
		size-=EEPROM_BATCH_WRITE;
		addr+=EEPROM_BATCH_WRITE;
	}
	if(size>0)
	{
		Wire.beginTransmission(AT24C256B_ADDR);
		Wire.write(addr>>8); // addr
		Wire.write(addr&0xFF); // addr
		Wire.write(array+i*EEPROM_BATCH_WRITE,size);
		Wire.endTransmission();
		delay(EEPROM_DELAY);
	}
}
void myEEPROM::writeAddr(ushort addr)
{
	Wire.beginTransmission(AT24C256B_ADDR);
	Wire.write(addr>>8);  
	Wire.write(addr&0xFF);
	Wire.endTransmission(); 
	pos=addr;
}

void myEEPROM::request(ushort many)
{
	request(0,many);
}

void myEEPROM::request(ushort addr,ushort many)
{
	if(many>EEPROM_BATCH_READ)
	{
		Serial << "WARNING:\tEEPROM:\trequested more than " << EEPROM_BATCH_READ << "B, giving only " << EEPROM_BATCH_READ << "!!\n";
		many=EEPROM_BATCH_READ;
	}
	writeAddr(addr);
	Wire.requestFrom(AT24C256B_ADDR,many);
}

void myEEPROM::print(ushort many)
{
	print(0,many);
}

void myEEPROM::print(ushort addr, ushort many)
{
	for(int i=0;i<EEPROM_BATCH_READ;i++)
		Serial << i << '\t';
	Serial << endl;
	for(int i=0;i<EEPROM_BATCH_READ;i++)
		Serial << "________";
	Serial << endl;
	while(many-EEPROM_BATCH_READ>0)
	{
		request(addr,EEPROM_BATCH_READ);
		while(Wire.available())
		{
			Serial << Wire.read() << '\t';
		}
		Serial << endl;
		many-=EEPROM_BATCH_READ;
		addr+=EEPROM_BATCH_READ;
	}
	if(many > 0)
	{
		request(addr,many);
		while(Wire.available())
		{
			Serial << Wire.read() << '\t';
		}
		Serial << endl;
	}
}

byte myEEPROM::get(ushort addr)
{// CHECK: please check this function later
	request(addr,1);
	return Wire.read();
}

void myEEPROM::put(byte data)
{
	put(0,data);
}

void myEEPROM::put(ushort addr,byte data)
{
	Wire.beginTransmission(AT24C256B_ADDR);
	Wire.write(addr>>8); // addr
	Wire.write(addr&0xFF); // addr
	Wire.write(data);
	Wire.endTransmission();
	delay(EEPROM_DELAY);
}
