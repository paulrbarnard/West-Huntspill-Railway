#ifndef	myEEPROM_H
#define	myEEPROM_H
#include <Wire.h>
#include <Arduino.h>
#include <Streaming.h>

#define	AT24C256B_ADDR	0x50
#define	EEPROM_BATCH_WRITE	32
#define	EEPROM_BATCH_READ	64
#define	EEPROM_DELAY	3
class myEEPROM{
	public:
		void write(ushort addr, byte data);
		byte read(ushort addr);
		void	writePage(byte array[],ushort size);
		void	writePage(ushort addr, byte array[],ushort size);
		void	writeAddr(ushort addr);
		void	request(ushort many);
		void	request(ushort addr,ushort many);
		void	print(ushort many);
		void	print(ushort addr, ushort many);
		byte	get(ushort addr);
		void	put(byte data);
		void	put(ushort addr,byte data);
	private:
		ushort	pos=0;
};
#endif
