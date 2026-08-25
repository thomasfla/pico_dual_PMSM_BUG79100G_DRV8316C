

#include "./drv8316.h"
#include "shared_spi_bus.h"

#include "hardware/gpio.h"


void DRV8316Driver3PWM::init(SPIClass* _spi) {
	DRV8316Driver::init(_spi);
	setRegistersLocked(false);
	delayMicroseconds(1);
	DRV8316Driver::setPWMMode(DRV8316_PWMMode::PWM3_Mode);
	BLDCDriver3PWM::init();
};


void DRV8316Driver6PWM::init(SPIClass* _spi) {
	DRV8316Driver::init(_spi);
	setRegistersLocked(false);
	delayMicroseconds(1);
	DRV8316Driver::setPWMMode(DRV8316_PWMMode::PWM6_Mode); // default mode is 6-PWM
	BLDCDriver6PWM::init();
};






/*
 * SPI setup:
 *
 *  capture on falling, propagate on rising =
 *  MSB first
 *
 * 	16 bit words
 * 	 outgoing: R/W:1 addr:6 parity:1 data:8
 * 	 incoming: status:8 data:8
 *
 * 	 on reads, incoming data is content of register being read
 * 	 on writes, incomnig data is content of register being written
 *
 *
 */

void handleInterrupt() {

}

void DRV8316Driver::init(SPIClass* _spi) {
	spi = _spi;
	settings = SPISettings(DRV8316_SPI_HZ, MSBFIRST, SPI_MODE1);
	control1Shadow = 0;
	control2Shadow = 0;
	control3Shadow = 0;
	control4Shadow = 0;
	control5Shadow = 0;
	control6Shadow = 0;
	control10Shadow = 0;

	//setup pins
	pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH); // switch off

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();

	if (_isset(nFault)) {
		pinMode(nFault, INPUT);
		// TODO add interrupt handler on the nFault pin if configured
		// add configuration for how to handle faults... idea: interrupt handler calls a callback, depending on the type of fault
		// consider what would be a useful configuration in practice? What do we want to do on a fault, e.g. over-temperature for example?

		//attachInterrupt(digitalPinToInterrupt(nFault), handleInterrupt, PinStatus::FALLING);
	}
};




bool DRV8316Driver::getParity(uint16_t data) {
	//PARITY = XNOR(CMD, A5..A0, D7..D0)
	uint8_t par = 0;
	for (int i=0;i<16;i++) {
		if (((data)>>i) & 0x0001)
			par+=1;
	}
	return (par&0x01)==0x01; // even number of bits means true
}




uint16_t DRV8316Driver::readSPI(uint8_t addr) {
	uint16_t data = (((addr<<1) | 0x80)<<8)|0x0000;
	if (getParity(data))
		data |= 0x0100;
	uint16_t result = transferSPI(data);
//	Serial.print("SPI Read Result: ");
//	Serial.print(data, HEX);
//	Serial.print(" -> ");
//	Serial.println(result, HEX);
	return result;
}


uint16_t DRV8316Driver::writeSPI(uint8_t addr, uint8_t value) {
	uint16_t data = ((addr<<1)<<8)|value;
	if (getParity(data))
		data |= 0x0100;
	uint16_t result = transferSPI(data);
//	Serial.print("SPI Write Result: ");
//	Serial.print(data, HEX);
//	Serial.print(" -> ");
//	Serial.println(result, HEX);
	return result;
}

uint16_t DRV8316Driver::transferSPI(uint16_t data) {
	sharedSpiUseDrv8316();
	gpio_put(cs, 0);
	delayMicroseconds(DRV8316_SPI_CS_SETUP_US);
	uint16_t result = 0;
	spi_write16_read16_blocking(spi0, &data, &result, 1);
	delayMicroseconds(DRV8316_SPI_CS_HOLD_US);
	gpio_put(cs, 1);
	delayMicroseconds(DRV8316_SPI_CS_IDLE_US);
	return result;
}



DRV8316Status DRV8316Driver::getStatus() {
	IC_Status data;
	Status__1 data1;
	Status__2 data2;
	uint16_t result = readSPI(IC_Status_ADDR);
	data.reg = (result & 0x00FF);
	delayMicroseconds(1); // delay at least 400ns between operations
	result = readSPI(Status__1_ADDR);
	data1.reg = (result & 0x00FF);
	delayMicroseconds(1); // delay at least 400ns between operations
	result = readSPI(Status__2_ADDR);
	data2.reg = (result & 0x00FF);
	return DRV8316Status(data, data1, data2);
}

uint16_t DRV8316Driver::readRegisterRaw(uint8_t addr) {
	return readSPI(addr);
}








void DRV8316Driver::clearFault() {
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__2_ADDR, control2Shadow | CLR_FAULT_CLR);
};








bool DRV8316Driver::isRegistersLocked(){
	uint16_t result = readSPI(Control__1_ADDR);
	Control__1 data;
	data.reg = (result & 0x00FF);
	return data.REG_LOCK==REG_LOCK_LOCK;
}
void DRV8316Driver::setRegistersLocked(bool lock){
	Control__1 data;
	data.reg = control1Shadow;
	data.REG_LOCK = lock?REG_LOCK_LOCK:REG_LOCK_UNLOCK;
	control1Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__1_ADDR, control1Shadow);
}



DRV8316_PWMMode DRV8316Driver::getPWMMode() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_PWMMode)data.PWM_MODE;
};
void DRV8316Driver::setPWMMode(DRV8316_PWMMode pwmMode){
	Control__2 data;
	data.reg = control2Shadow;
	data.PWM_MODE = pwmMode;
	control2Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__2_ADDR, control2Shadow);
};



DRV8316_Slew DRV8316Driver::getSlew() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_Slew)data.SLEW;
};
void DRV8316Driver::setSlew(DRV8316_Slew slewRate) {
	Control__2 data;
	data.reg = control2Shadow;
	data.SLEW = slewRate;
	control2Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__2_ADDR, control2Shadow);
};



DRV8316_SDOMode DRV8316Driver::getSDOMode() {
	uint16_t result = readSPI(Control__2_ADDR);
	Control__2 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_SDOMode)data.SDO_MODE;
};
void DRV8316Driver::setSDOMode(DRV8316_SDOMode sdoMode) {
	Control__2 data;
	data.reg = control2Shadow;
	data.SDO_MODE = sdoMode;
	control2Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__2_ADDR, control2Shadow);
};



bool DRV8316Driver::isOvertemperatureReporting(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.OTW_REP==OTW_REP_ENABLE;
};
void DRV8316Driver::setOvertemperatureReporting(bool reportFault){
	Control__3 data;
	data.reg = control3Shadow;
	data.OTW_REP = reportFault?OTW_REP_ENABLE:OTW_REP_DISABLE;
	control3Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__3_ADDR, control3Shadow);
};



bool DRV8316Driver::isSPIFaultReporting(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.SPI_FLT_REP==SPI_FLT_REP_ENABLE;
}
void DRV8316Driver::setSPIFaultReporting(bool reportFault){
	Control__3 data;
	data.reg = control3Shadow;
	data.SPI_FLT_REP = reportFault?SPI_FLT_REP_ENABLE:SPI_FLT_REP_DISABLE;
	control3Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__3_ADDR, control3Shadow);
}



bool DRV8316Driver::isOvervoltageProtection(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return data.OVP_EN==OVP_EN_ENABLE;
};
void DRV8316Driver::setOvervoltageProtection(bool enabled){
	Control__3 data;
	data.reg = control3Shadow;
	data.OVP_EN = enabled?OVP_EN_ENABLE:OVP_EN_DISABLE;
	control3Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__3_ADDR, control3Shadow);
};



DRV8316_OVP DRV8316Driver::getOvervoltageLevel(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OVP)data.OVP_SEL;
};
void DRV8316Driver::setOvervoltageLevel(DRV8316_OVP voltage){
	Control__3 data;
	data.reg = control3Shadow;
	data.OVP_SEL = voltage;
	control3Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__3_ADDR, control3Shadow);
};



DRV8316_PWM100DUTY DRV8316Driver::getPWM100Frequency(){
	uint16_t result = readSPI(Control__3_ADDR);
	Control__3 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_PWM100DUTY)data.PWM_100_DUTY_SEL;
};
void DRV8316Driver::setPWM100Frequency(DRV8316_PWM100DUTY freq){
	Control__3 data;
	data.reg = control3Shadow;
	data.PWM_100_DUTY_SEL = freq;
	control3Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__3_ADDR, control3Shadow);
};



DRV8316_OCPMode DRV8316Driver::getOCPMode(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPMode)data.OCP_MODE;

};
void DRV8316Driver::setOCPMode(DRV8316_OCPMode ocpMode){
	Control__4 data;
	data.reg = control4Shadow;
	data.OCP_MODE = ocpMode;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



DRV8316_OCPLevel DRV8316Driver::getOCPLevel(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPLevel)data.OCP_LVL;
};
void DRV8316Driver::setOCPLevel(DRV8316_OCPLevel amps){
	Control__4 data;
	data.reg = control4Shadow;
	data.OCP_LVL = amps;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



DRV8316_OCPRetry DRV8316Driver::getOCPRetryTime(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPRetry)data.OCP_RETRY;
};
void DRV8316Driver::setOCPRetryTime(DRV8316_OCPRetry ms){
	Control__4 data;
	data.reg = control4Shadow;
	data.OCP_RETRY = ms;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



DRV8316_OCPDeglitch DRV8316Driver::getOCPDeglitchTime(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_OCPDeglitch)data.OCP_DEG;
};
void DRV8316Driver::setOCPDeglitchTime(DRV8316_OCPDeglitch ms){
	Control__4 data;
	data.reg = control4Shadow;
	data.OCP_DEG = ms;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



bool DRV8316Driver::isOCPClearInPWMCycleChange(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return data.OCP_CBC==OCP_CBC_ENABLE;
};
void DRV8316Driver::setOCPClearInPWMCycleChange(bool enable){
	Control__4 data;
	data.reg = control4Shadow;
	data.OCP_CBC = enable?OCP_CBC_ENABLE:OCP_CBC_DISABLE;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



bool DRV8316Driver::isDriverOffEnabled(){
	uint16_t result = readSPI(Control__4_ADDR);
	Control__4 data;
	data.reg = (result & 0x00FF);
	return data.DRV_OFF==DRV_OFF_ENABLE;
};
void DRV8316Driver::setDriverOffEnabled(bool enabled){
	Control__4 data;
	data.reg = control4Shadow;
	data.DRV_OFF = enabled?DRV_OFF_ENABLE:DRV_OFF_DISABLE;
	control4Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__4_ADDR, control4Shadow);
};



DRV8316_CSAGain DRV8316Driver::getCurrentSenseGain(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_CSAGain)data.CSA_GAIN;
};
void DRV8316Driver::setCurrentSenseGain(DRV8316_CSAGain gain){
	Control__5 data;
	data.reg = control5Shadow;
	data.CSA_GAIN = gain;
	control5Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__5_ADDR, control5Shadow);
};



bool DRV8316Driver::isActiveSynchronousRectificationEnabled(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return data.EN_ASR==EN_ASR_ENABLE;

};
void DRV8316Driver::setActiveSynchronousRectificationEnabled(bool enabled){
	Control__5 data;
	data.reg = control5Shadow;
	data.EN_ASR = enabled?EN_ASR_ENABLE:EN_ASR_DISABLE;
	control5Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__5_ADDR, control5Shadow);
};



bool DRV8316Driver::isActiveAsynchronousRectificationEnabled(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return data.EN_AAR==EN_AAR_ENABLE;
};
void DRV8316Driver::setActiveAsynchronousRectificationEnabled(bool enabled){
	Control__5 data;
	data.reg = control5Shadow;
	data.EN_AAR = enabled?EN_AAR_ENABLE:EN_AAR_DISABLE;
	control5Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__5_ADDR, control5Shadow);
};



DRV8316_Recirculation DRV8316Driver::getRecirculationMode(){
	uint16_t result = readSPI(Control__5_ADDR);
	Control__5 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_Recirculation)data.ILIM_RECIR;
};
void DRV8316Driver::setRecirculationMode(DRV8316_Recirculation recirculationMode){
	Control__5 data;
	data.reg = control5Shadow;
	data.ILIM_RECIR = recirculationMode;
	control5Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__5_ADDR, control5Shadow);
};



bool DRV8316Driver::isBuckEnabled(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return data.BUCK_DIS==BUCK_DIS_BUCK_ENABLE;
};
void DRV8316Driver::setBuckEnabled(bool enabled){
	Control__6 data;
	data.reg = control6Shadow;
	data.BUCK_DIS = enabled?BUCK_DIS_BUCK_ENABLE:BUCK_DIS_BUCK_DISABLE;
	control6Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__6_ADDR, control6Shadow);
};



DRV8316_BuckVoltage DRV8316Driver::getBuckVoltage(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_BuckVoltage)data.BUCK_SEL;
};
void DRV8316Driver::setBuckVoltage(DRV8316_BuckVoltage volts){
	Control__6 data;
	data.reg = control6Shadow;
	data.BUCK_SEL = volts;
	control6Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__6_ADDR, control6Shadow);
};



DRV8316_BuckCurrentLimit DRV8316Driver::getBuckCurrentLimit(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_BuckCurrentLimit)data.BUCK_CL;
};
void DRV8316Driver::setBuckCurrentLimit(DRV8316_BuckCurrentLimit mamps){
	Control__6 data;
	data.reg = control6Shadow;
	data.BUCK_CL = mamps;
	control6Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__6_ADDR, control6Shadow);
};



bool DRV8316Driver::isBuckPowerSequencingEnabled(){
	uint16_t result = readSPI(Control__6_ADDR);
	Control__6 data;
	data.reg = (result & 0x00FF);
	return data.BUCK_PS_DIS==BUCK_PS_DIS_ENABLE;

};
void DRV8316Driver::setBuckPowerSequencingEnabled(bool enabled){
	Control__6 data;
	data.reg = control6Shadow;
	data.BUCK_PS_DIS = enabled?BUCK_PS_DIS_ENABLE:BUCK_PS_DIS_DISABLE;
	control6Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__6_ADDR, control6Shadow);
};



DRV8316_DelayTarget DRV8316Driver::getDelayTarget(){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	return (DRV8316_DelayTarget)data.DLY_TARGET;
};
void DRV8316Driver::setDelayTarget(DRV8316_DelayTarget us){
	Control__10 data;
	data.reg = control10Shadow;
	data.DLY_TARGET = us;
	control10Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__10_ADDR, control10Shadow);
};



bool DRV8316Driver::isDelayCompensationEnabled(){
	uint16_t result = readSPI(Control__10_ADDR);
	Control__10 data;
	data.reg = (result & 0x00FF);
	return data.DLYCMP_EN==DLYCMP_EN_ENABLE;
};
void DRV8316Driver::setDelayCompensationEnabled(bool enabled){
	Control__10 data;
	data.reg = control10Shadow;
	data.DLYCMP_EN = enabled?DLYCMP_EN_ENABLE:DLYCMP_EN_DISABLE;
	control10Shadow = data.reg;
	delayMicroseconds(1); // delay at least 400ns between operations
	writeSPI(Control__10_ADDR, control10Shadow);
};
