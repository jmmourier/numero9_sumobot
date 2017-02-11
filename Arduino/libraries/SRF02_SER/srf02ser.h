//srf02_ser_h
#ifndef srf02ser_h
#define srf02ser_h

#define US_TIMEOUT 100
#define US_TIME_REQUEST 70
#define US_TIMEOUT_WAIT 500

// TODO : revoir les commandes
#define US_CMD_REAL_RANGE 0x51 //Real Ranging Mode - Result in centimeters
#define US_CMD_REAL_RANGE_AUTO 0x54 // Real Ranging Mode - Result in centimeters, automatically Tx range back to controller as soon as ranging is complete.
#define US_CMD_GET_SOFTWARE_VERSION 0x5D
#define US_CMD_GET_RANGE 0x5E

class srf02ser
{
public : 
	// constructeur
	srf02ser(unsigned char);
	void init();
	bool checkCommunication();
	void process();
	bool hasAnErrorFlaged() {return errorFlag;}
	int getLastResult() {return lastResult;}

private:
	int lastResult;
	bool errorFlag;
};

#endif
