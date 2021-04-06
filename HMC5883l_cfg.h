
#ifndef HMC5883L_CFG_H
#define HMC5883L_CFG_H


// http://www.farnell.com/datasheets/1683374.pdf
namespace HMC5883L_CFG {
    const uint8_t ADDRESS = 0x1E;
	
	// I2C value register addresses. Taken directly from the datasheet
    enum REGISTERS {
        CONFIG_A = 0x00,
        CONFIG_B = 0x01,
        MEASUREMENT_MODE = 0x02,
        OUT_X_M = 0x03,
        OUT_X_L = 0x04,
        OUT_Z_M = 0x05,
        OUT_Z_L = 0x06,
        OUT_Y_M = 0x07,
        OUT_Y_L = 0x08,
        STATUS = 0x09,
        IDENT_A = 0x0A,
        IDENT_B = 0x0B,
        IDENT_C = 0x0C
    };

	// The magic identifier that should always be present in the HMC's 
	// registers `IDENT_A`, `IDENT_B`, and `IDENT_C`
    enum EXPECTED_IDENT {
        A = 0x48,
        B = 0x34,
        C = 0x33,
    };


    //==========CFG REG A OPTIONS==========//
	// Number of samples averaged per reading. S1 = no averaging, 
	// S8 = 8 samples averaged
    const uint8_t SAMPLES_MASK = 0b01100000;
    enum SAMPLES {
        S8 = 0b11,
        S4 = 0b10,
        S2 = 0b01,
        S1 = 0b00
    };

	// Sample/Data rate mask. Can set be used to set sample rate from 
	// 0.75Hz to 75Hz
    const uint8_t DATA_RATE_MASK = 0b00011100;
    enum RATE {
        D_75HZ = 0b110,
        D_30HZ = 0b101,
        D_15HZ = 0b100,
        D_7_5HZ = 0b011,
        D_3HZ = 0b010,
        D_1_5HZ = 0b001,
        D_0_75HZ = 0b000
    };

	// Sets the internal electromagnet to + / - / none. SHOULD ONLY EVER
	// BE `POSITIVE` / `NEGATIVE` FOR CALIBRATION PURPOSES!!!!!
    const uint8_t MEASUREMENT_FLOW_MASK = 0b00000011;
    enum FLOW {
        NONE = 0b00,
        POSITIVE = 0b01,
        NEGATIVE = 0b10
    };

    //==========CFG REG A OPTIONS==========//
	//Compass gain. I.E. compass sensitivity.  
    const uint8_t GAIN_MASK = 0b11100000;
    enum GAIN {
        G_8_1GA = 0b111,
        G_5_6GA = 0b110,
        G_4_7GA = 0b101,
        G_4GA = 0b100,
        G_2_5GA = 0b011,
        G_1_9GA = 0b010,
        G_1_3GA = 0b001, //Default
        G_0_88GA = 0b000
    };

	// Returns factor used to convert raw values from compass to Gauss
    inline double gainToFactor(GAIN gain) {
        switch (gain) {
            case GAIN::G_8_1GA:  return 4.35l;
            case GAIN::G_5_6GA:  return 3.03l;
            case GAIN::G_4_7GA:  return 2.56l;
            case GAIN::G_4GA:    return 2.27l;
            case GAIN::G_2_5GA:  return 1.52l;
            case GAIN::G_1_9GA:  return 1.22l;
            case GAIN::G_0_88GA: return 0.73l;
            case GAIN::G_1_3GA: //Fallthrough to default
            default: return 0.92l;
        };
    };

    // Sets operating mode of the compass. More info on page 14 of
	// Datasheet
	//     IDLE = not operating
	//     SINGLE = one reading after certain register write.
	//     CONTINUOUS = constant readings.
	
    enum MODE {
        IDLE = 0b10,
        SINGLE = 0b01,
        CONTINUOUS = 0b00
    };

    //Status register masks
    const uint8_t STATUS_READY_MASK = 0b00000001;
    const uint8_t STATUS_LOCK_MASK = 0b00000010;

    //Calibration parameters.
    const double XY_EXCITATION = 1160.0l;
    const double Z_EXCITATION = 1080.0l;
};


#endif //STEPPERTEST_HMC5883L_CFG_H
