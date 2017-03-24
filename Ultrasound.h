/* 
 * File:   Ultrasound.h
 * Author: Jack
 *
 * Created on 19 March 2017, 20:47
 */

#ifndef ULTRASOUND_H
#define	ULTRASOUND_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    void ConfigureUltrasound(float echo_to_cm, float echo_to_in);
    
    inline void UltrasoundISR(void);
    
    void GetDistance(void);
    
    unsigned char BusyDistanceAcq(void);
    
    unsigned int ReadEchoLength(void);
    
    void ResetEchoLength(void);
    
    float ConvertDistanceCM(unsigned int echo_time);
    
    float ConvertDistanceIN(unsigned int echo_time);

#ifdef	__cplusplus
}
#endif

#endif	/* ULTRASOUND_H */

