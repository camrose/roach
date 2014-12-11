/******************************************************************************
* Name: line_sensor.h
* Desc: line sensor support
* Date: 2014-06-12
* Author: Cameron Rose
******************************************************************************/
#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#define LINE_VIEW_ANGLE     (0.558)  // view angle in radians (32 degrees)
#define LINE_FRAME_WIDTH    (127)    // frame width in pixels

typedef struct {
    unsigned long timestamp;
    unsigned int frame_num;
    unsigned char pixels[128];
} LineCamStruct;
typedef LineCamStruct* LineCam;

typedef struct {
    unsigned long timestamp;
    unsigned int frame_num;
    unsigned char edges[6];
    float location;
    float distance;
} EdgesStruct;
typedef EdgesStruct* Edges;

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs);
void lsStartCapture(unsigned char flag);
unsigned char lsHasNewFrame(void);
LineCam lsGetFrame();
void lsSetExposure(unsigned int et, unsigned int fs);
void lsReturnFrame(LineCam frame);
unsigned int lsGetFrameNum(void);
unsigned char lsGetEdges(Edges edges);
unsigned char lsGetMarker(Edges edges);
unsigned char lsFoundMarker();

#endif // __LINE_SENSOR_H

