void LoadCalibration(void) {
    unsigned char index;
    
    if(ReadCharEEPROM(0x00) == 0) {
        ChangeMode();
        GenerateCalibration();        
    }
    
    else {
        unsigned char index;
        sensor_threshold = ReadIntEEPROM(0x01);
        for(index = 0; index < NO_OF_SENSORS; index++) {
            sensor_offsets[index] = ReadIntEEPROM(0x03 + (2 * index));
        }
    }
    
}

void GenerateCalibration(void) {
    unsigned char index;
    
    //GENERATE SENSOR OFFSETS
    while(PB1pressed() == 0) {
        
        CalibrateSensors();
        Delay10KTCYx(10);
        
        if(ReadMillis0() >= 100) {
            LATJbits.LATJ0 ^= 1;
            ResetMillis0();
        }
        
    }
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        WriteIntEEPROM(0x03 + (2 * index), sensor_offsets[index]);
    }
    
    ChangeMode();
    ResetMillis0();
    
    //
    while(PB1pressed() == 0) {
        
        CalibrateLine();
        Delay10KTCYx(10);
        
    }
    
    WriteIntEEPROM(0x01, sensor_threshold);
    WriteCharEEPROM(0x00, 1);
    
    
}
