/* Includes ------------------------------------------------------------------*/
#include "gnss_parser.h"
#include <string.h>

#include <stdlib.h>
#include <stdio.h>
#define GNRMC 0
#define GNGGA 1


//void substring(char *destination, const char *source, uint8_t beg, uint8_t n) {
// while (n > 0) {
// *destination = *(source + beg);
// destination++;
// source++;
// n--;
// }
// // null terminate destination string
// *destination = '\0';
//}



uint8_t hex2int(char ch)
{
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	if (ch >= 'A' && ch <= 'F')
		return ch - 'A' + 10;
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;
	return -1;
}

uint8_t nmea0183_checksum(char *nmea_data , uint8_t len)
{
	uint8_t crc = 0;
	// the first $ sign and the last two bytes of original CRC + the * sign
	for (uint8_t i = 0; i < len; i ++) {
		crc ^= nmea_data[i];
	}
	return crc;
}
uint8_t* nmea_parser(char *NR,uint8_t responseLenght,uint8_t *checkSum ,uint8_t *rCheckSum)
{
	char (*nmeaResponse)[responseLenght];
	uint8_t gnssCRC = 0;
	nmeaResponse=NR;
	uint8_t QOG = 0xC0;
	uint16_t courseStatus = 0x0;
	static uint8_t GPSInformation[18] = {0};


	char *p1;
	char *p2;
	p1 = strstr(nmeaResponse[GNGGA],"$GNGGA");
	p2 = strstr(nmeaResponse[GNRMC],"$GNRMC");

	if(p1 != NULL && p2 != NULL){


		for(uint8_t sen=0;sen<2;sen++){
			uint8_t recvCrc=0;
			char rawData[100];
			memset(rawData,0,sizeof(rawData));

			char *ind1;
			char *ind2;
			ind1=strchr(nmeaResponse[sen],'$');
			ind2=strchr(nmeaResponse[sen],'*');

			if(ind1!=NULL && ind2!=NULL){
				uint8_t len = ind2-ind1;
				gnssCRC = 0;
				recvCrc = hex2int(*(ind2+2)) | hex2int(*(ind2+1)) <<4;
				for(uint8_t i=0;i<len-1;i++){
					rawData[i]=*(ind1+1+i);
					gnssCRC ^= rawData[i];

				}
				*checkSum = gnssCRC;
				*rCheckSum = recvCrc;
			}
			else{
				return NULL;
			}
			if(recvCrc != gnssCRC){
				return NULL;
			}
		}




		//    char crcGNGGA[85];
		//    memset(crcGNGGA,0,sizeof(crcGNGGA));
		//    char mycc[3];
		//    memset(mycc,0,sizeof(mycc));
		// uint8_t crclen;
		// uint8_t istart,iend;
		//
		// uint8_t cc;
		// uint8_t finalcc;
		// char *mp1,*mp2;
		// mp1 = strchr(nmeaResponse[GNGGA], '$')+1;
		// mp2 = strchr(nmeaResponse[GNGGA], '*');
		// crclen = mp2-mp1;
		// istart = mp1 - nmeaResponse[GNGGA];
		// iend = mp2 - nmeaResponse[GNGGA];
		// mycc[0]=nmeaResponse[GNGGA][iend+1];
		// mycc[1]=nmeaResponse[GNGGA][iend+2];
		// finalcc = (uint8_t)strtol(mycc,NULL,16);
		//    substring(crcGNGGA,nmeaResponse[GNGGA],istart,crclen);
		//   cc = nmea0183_checksum(crcGNGGA,crclen);
		////   GPSInformation[0] = cc;
		////   GPSInformation[1]=finalcc;
		////   GPSInformation[2]=crclen;
		////   GPSInformation[3]=iend;
		////   GPSInformation[4]=istart;
		// if(finalcc != cc){
		// return NULL;
		// }
		//    memset(crcGNGGA,0,sizeof(crcGNGGA));
		//    memset(mycc,0,sizeof(mycc));
		//    mp1 = strchr(nmeaResponse[GNRMC], '$')+1;
		// mp2 = strchr(nmeaResponse[GNRMC], '*');
		// crclen = mp2-mp1;
		// istart = mp1 - nmeaResponse[GNRMC];
		// iend = mp2 - nmeaResponse[GNRMC];
		// mycc[0]=nmeaResponse[GNRMC][iend+1];
		// mycc[1]=nmeaResponse[GNRMC][iend+2];
		// finalcc = (uint8_t)strtol(mycc,NULL,16);
		//    substring(crcGNGGA,nmeaResponse[GNRMC],istart,crclen);
		//   cc = nmea0183_checksum(crcGNGGA,crclen);
		// if(finalcc != cc){
		// return NULL;
		// }

		//data is valid. go on
		uint8_t GNGGAComma[15];
		uint8_t GNRMCComma[15];
		memset(GNGGAComma, 0, sizeof(GNGGAComma)); // for automatically-allocated arrays
		memset(GNRMCComma, 0, sizeof(GNRMCComma)); // for automatically-allocated arrays


		uint8_t GNGGACInd=0;
		uint8_t GNRMCCInd=0;
		uint32_t latitude;
		uint32_t longitude;
		uint8_t tempDataIndex = 0;
		char tempData[15];




		//-------store all parameter's comma
		for(uint8_t i=0;i<responseLenght;i++ ){
			if(nmeaResponse[GNGGA][i]==44){
				GNGGAComma[GNGGACInd]=i;
				GNGGACInd++;
			}
			if(nmeaResponse[GNRMC][i]==44){
				GNRMCComma[GNRMCCInd]=i;
				GNRMCCInd++;
			}
		}

		//------------------------Time and date Stamp------------------------------------
		memset(tempData,0,sizeof(tempData));
		tempDataIndex = 0;
		for(uint8_t x =GNRMCComma[8]+1;x<GNRMCComma[9];x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}
		char tempData2[4];

		//Year (1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[4];
		tempData2[1]=tempData[5];
		GPSInformation[0] = (uint8_t)atoi(tempData2);

		//month (1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[2];
		tempData2[1]=tempData[3];
		GPSInformation[1] = atoi(tempData2);
		//day (1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[0];
		tempData2[1]=tempData[1];
		GPSInformation[2] = atoi(tempData2);

		memset(tempData,0,sizeof(tempData));
		tempDataIndex = 0;
		for(uint8_t x =GNRMCComma[0]+1;x<GNRMCComma[1]-4;x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}
		//hour(1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[0];
		tempData2[1]=tempData[1];
		GPSInformation[3] = (uint8_t)atoi(tempData2);
		//minutes(1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[2];
		tempData2[1]=tempData[3];
		GPSInformation[4] = (uint8_t)atoi(tempData2);
		//seconds(1 byte)
		memset(tempData2,0,sizeof(tempData2));
		tempData2[0]=tempData[4];
		tempData2[1]=tempData[5];
		GPSInformation[5] = (uint8_t)atoi(tempData2);
		//--------------------------------time/date stamp end-------------------------------------------------

		//-----Quantity of GPS information satellites(1 byte, left 4 bit for gps info len, right 4 bit for qty of sats)
		memset(tempData,0,sizeof(tempData));
		tempDataIndex = 0;
		for(uint8_t x =GNGGAComma[6]+1;x<GNGGAComma[7];x++){
			tempData[tempDataIndex] = nmeaResponse[GNGGA][x];
			tempDataIndex++;
		}
		QOG |= (atoi(tempData));
		GPSInformation[6] = QOG;
		//-------------------------------------QOG END------------------------------------------------------


		//------------------------------------latitude (4 bytes)--------------------------------------
		// char *ptr1;
		memset(tempData,0,sizeof(tempData));
		memset(tempData2,0,sizeof(tempData2));
		tempDataIndex = 0;
		for(uint8_t x =GNRMCComma[2]+1;x<GNRMCComma[3];x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}
		tempData2[0]=tempData[0];
		tempData2[1]=tempData[1];
		uint8_t t1=2;
		uint8_t t2=0;
		char tB[9];
		memset(tB,0,sizeof(tB));
		while(tempData[t1]!=NULL){
			if(tempData[t1]!='.'){
				tB[t2]=tempData[t1];
				t2++;
			}
			t1++;
		}
		latitude=atoi(tB);
		if(t2==4){
			latitude*=100;
		}
		else if(t2==5){
			latitude*=10;
		}

		latitude=latitude*3;

		latitude+= (atoi(tempData2)*60*30000);

		GPSInformation[7] = latitude>>24;
		GPSInformation[8] = latitude>>16;
		GPSInformation[9] = latitude>>8;
		GPSInformation[10] = latitude;
		//------------------------------------latitude end--------------------------------------



		//------------------------------------longitude (4 bytes)--------------------------------------
		// char *ptr2;
		memset(tempData,0,sizeof(tempData));
		memset(tempData2,0,sizeof(tempData2));
		tempDataIndex = 0;
		for(uint8_t x =GNRMCComma[4]+1;x<GNRMCComma[5];x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}

		tempData2[0]=tempData[0];
		tempData2[1]=tempData[1];
		tempData2[2]=tempData[2];
		memset(tB,0,sizeof(tB));
		t1=3;
		t2=0;
		while(tempData[t1]!=NULL){
			if(tempData[t1]!='.'){
				tB[t2]=tempData[t1];
				t2++;
			}
			t1++;
		}
		longitude=atoi(tB);
		if(t2==4){
			longitude*=100;
		}
		else if(t2==5){
			longitude*=10;
		}
		longitude=longitude*3;
		longitude+= (atoi(tempData2)*60*30000);
		GPSInformation[11] = longitude>>24;
		GPSInformation[12] = longitude>>16;
		GPSInformation[13] = longitude>>8;
		GPSInformation[14] = longitude;
		//------------------------------------longitude end--------------------------------------



		//----------------------------------speed----------------------------------------------------
		memset(tempData,0,sizeof(tempData));
		memset(tempData2,0,sizeof(tempData2));
		tempDataIndex = 0;
		float speedinf;
		for(uint8_t x =GNRMCComma[6]+1;x<GNRMCComma[7];x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}

		speedinf = strtof(tempData,NULL);
		speedinf= speedinf * 1.85;
		if(speedinf>255){
			speedinf=255;
		}
		if(speedinf<1){
			speedinf=0;

		}

		GPSInformation[15] = (int)speedinf;
		//--------------------------------speed end-----------------------------------------------



		//------------------------------Coursestatus block(2 byte)--------------------------------
		//Byte1, bit 4(gps position bit)
		if (nmeaResponse[GNRMC][GNRMCComma[1]+1] == 'A'){
			courseStatus |= 0x1000;
		}
		//Byte1, bit 5(realtime differential)
		if (nmeaResponse[GNGGA][GNGGAComma[5]+1] != '1'){
			courseStatus |= 0x2000;
		}
		//Byte1, bit 3(East/West bit)
		if(nmeaResponse[GNRMC][GNRMCComma[5]+1] == 'W' ){
			courseStatus |= 0x800;
		}
		//Byte1, bit 2(North/South bit)
		if(nmeaResponse[GNRMC][GNRMCComma[3]+1] == 'N' ){
			courseStatus |= 0x400;
		}

		//Course
		memset(tempData,0,sizeof(tempData));
		tempDataIndex = 0;
		for(uint8_t x =GNRMCComma[7]+1;x<GNRMCComma[8];x++){
			tempData[tempDataIndex]= nmeaResponse[GNRMC][x];
			tempDataIndex++;
		}
		courseStatus |= atoi(tempData);
		GPSInformation[16]= courseStatus>>8;
		GPSInformation[17]= courseStatus;
		//------------------------------Course Status Block END------------------------------------

		return GPSInformation;
	}
	else{
		*checkSum = 55;
		return NULL;
	}

}
