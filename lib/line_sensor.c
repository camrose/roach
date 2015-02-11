/**
* Copyright (c) 2011-2012, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
*   of its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE

* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*
* Line Sensor Support
*
* by Cameron Rose
*
* v.beta
*
* Revisions:
*  Cameron Rose     2014-06-12      Initial implementation
*  
* 
* Notes:
*
* TODO:
*/

// ==== REFERENCES ==========================================
#include "line_sensor.h"
#include "utils.h"
#include "timer.h"
#include "adc_line.h"
#include "sclock.h"
#include "carray.h"
#include "counter.h"
#include "dsp.h"
#include <stdlib.h>

// ==== CONSTANTS ===========================================

#define FCY                     (40000000)
//#define LS_CLOCK                (_RB4)
//#define LS_SI                   (_LATB3)
#define LS_SI                   (_RB4)
#define LS_CLOCK                (_LATB3)
#define LOW                     (0)
#define HIGH                    (1)
#define MAX_LINE                (4000)
#define PX_TO_M                 (35.8209)
#define CENTER_TO_WIDTH         (0.2227)
#define EDGE_THRESH             (1000)


// ==== STATIC VARIABLES ====================================

static unsigned char is_ready;
static unsigned int cnt;
static unsigned int max_cnt;
//static unsigned int cnt_line;
LineCam current_frame;
static unsigned char has_new_frame;
static unsigned char new_line_sent;
static unsigned char found_marker;

unsigned int max_sig;

static CircArray empty_frame_pool, full_frame_pool;

static LineCam getEmptyFrame(void);
static void enqueueEmptyFrame(LineCam frame);
static LineCam getOldestFullFrame(void);
static void enqueueFullFrame(LineCam frame);
static void convolve(int numElems1,int numElems2,
        int* dstV,int* srcV1,int* srcV2);
static void sort6(unsigned char* d);


static Counter frame_counter;
static Counter px_counter;

// ==== FUNCTION STUBS ======================================

void setupTimer7(unsigned int fs);
void _T7Interrupt(void);



// ==== FUNCTION BODIES =====================================

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs) {
    setupTimer7(fs);
    LS_CLOCK = LOW;
    LS_SI = LOW;
    cnt = 0;
    max_cnt = 1001;
    unsigned int i;
    current_frame = NULL;

    frame_counter = cntrCreate(); // Frame counter allocation
    if(frame_counter == NULL) { return; }
    px_counter = cntrCreate();
    if(px_counter == NULL) { return; }
    
    empty_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(empty_frame_pool == NULL) { return; }
    full_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(full_frame_pool == NULL) { return; }

    for(i = 0; i < num_frames; i++) {
        lsReturnFrame(&frames[i]);
    }

    max_sig = 1;

    is_ready = 1;
    has_new_frame = 0;
    new_line_sent = 0;
    found_marker = 0;
}

void lsStartCapture(unsigned char flag) {

    if(flag) {
        WriteTimer7(0);
        cntrSet(px_counter, 0);    // Reset row counter
        cntrSet(frame_counter, 0);  // Reset frame counter
        LS_CLOCK = LOW;
        LS_SI = LOW;
        cnt = 0;
        EnableIntT7;
    } else {
        DisableIntT7;
        LS_CLOCK = LOW;
        LS_SI = LOW;
        cnt = 0;
    }
    

}

void lsSetExposure(unsigned int et, unsigned int fs) {
    DisableIntT7;
    _T7IF = 0;
    CloseTimer7();
    LS_CLOCK = LOW;
    LS_SI = LOW;
    max_cnt = et;
    setupTimer7(fs);   
}

unsigned char lsHasNewFrame(void) {
    return has_new_frame;
}

LineCam lsGetFrame(void) {
    return getOldestFullFrame();
}

void lsReturnFrame(LineCam frame) {
    if(frame == NULL) { return; }
    enqueueEmptyFrame(frame);
}

unsigned int lsGetFrameNum(void) {
    return cntrRead(frame_counter);
}

unsigned char lsGetEdges(Edges edges) {
    LineCam frame = NULL;
    int deriv_gauss[9] = {-7,-25,-50,-48,0,48,50,25,7};
    int img_gauss[136];
    int img[128];
    int peaks_az[3] = {0,0,0};
    unsigned char peaks_az_loc[3];
    int peaks_bz[3] = {0,0,0};
    unsigned char peaks_bz_loc[3];
    int i;
    int val;
    int loc;

    while (frame == NULL) {
        frame = lsGetFrame();
    }
    for (i=0;i<129;i++){
        img[i] = (int)(frame->pixels[i]);
    }
    convolve(128,9,img_gauss,img,deriv_gauss);
    for (i=14;i<123;i++) {
        if (img_gauss[i]>img_gauss[i-1] && img_gauss[i+1]<img_gauss[i]) {
            val = VectorMin(3,peaks_az,&loc);
            if (img_gauss[i] > val) {
                peaks_az[loc] = img_gauss[i];
                peaks_az_loc[loc] = i-4;
            }
        } else if (img_gauss[i]<img_gauss[i-1] && img_gauss[i+1]>img_gauss[i]){
            val = VectorMax(3,peaks_bz,&loc);
            if (img_gauss[i] < val) {
                peaks_bz[loc] = img_gauss[i];
                peaks_bz_loc[loc] = i-4;
            }
        }
    }
    for(i=0;i<3;i++){
        if(abs(peaks_az[i])>EDGE_THRESH) {
            edges->edges[i] = peaks_az_loc[i];
        }
        if(abs(peaks_bz[i])>EDGE_THRESH) {
            edges->edges[i+3] = peaks_bz_loc[i];
        }
    }
    edges->frame_num = frame->frame_num;
    edges->timestamp = frame->timestamp;
    sort6(edges->edges);
    lsReturnFrame(frame);
    if (edges->edges[0] == 0) {
        return 0;
    }
    return 1;
}

unsigned char lsGetMarker(Edges edges) {
    float center;
    float ratio;
    float ratio_error;
    int marker_width;
    int center_width;

    if (lsGetEdges(edges) == 0) {
        found_marker = 0;
        return 0;
    }

    center_width = edges->edges[3] - edges->edges[2];
    center = ((float)(edges->edges[5] + edges->edges[0]))/2.0;
    marker_width = edges->edges[5] - edges->edges[0];

    ratio = (float)center_width/(float)marker_width;
    ratio_error = fabs(ratio-CENTER_TO_WIDTH);
    if (fabs(ratio-CENTER_TO_WIDTH) < 0.1) {
        edges->location = center;
        edges->distance = PX_TO_M/marker_width;
        found_marker = 1;
        return 1;
    } else {
        edges->distance = 0;
        edges->location = 0;
        found_marker = 0;
        return 0;
    }


}

unsigned char lsFoundMarker() {
    return found_marker;
}

// =========== Private Functions ===============================================



void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void) {
    unsigned int px_num;
    unsigned int line;
    
    if (cnt == 0){
        LS_SI = HIGH;
        LS_CLOCK = HIGH;
        cnt++;
    } else {
        if(LS_CLOCK) {
            if (LS_SI){
                LS_SI = LOW;
            }
            px_num = cntrRead(px_counter);
            if (px_num < 129) {
                if(current_frame == NULL) {
                    current_frame = getEmptyFrame(); // Load new frame
                }
                if(current_frame == NULL) { return; }

                line = adcGetLine();
                //line = 0;

                current_frame->pixels[px_num] = (unsigned char)(255*((float)line/1200.0));
                cntrIncrement(px_counter);
            } else if (px_num == 129) {
                cntrIncrement(px_counter);
                current_frame->frame_num = cntrRead(frame_counter); // write frame number
                current_frame->timestamp = sclockGetTicks();
                enqueueFullFrame(current_frame); // Add to output queue
                current_frame = NULL;
                cntrIncrement(frame_counter);
            } else {
                cntrIncrement(px_counter);
            }
            LS_CLOCK = LOW; // End pulse
        } else {
            LS_CLOCK = HIGH; // Begin pulse
        }
        cnt++;
        if (cnt > max_cnt) {
            cnt = 0;
            cntrSet(px_counter, 0);
        }
    }

    _T7IF = 0;

}

void setupTimer7(unsigned int frequency) {

    unsigned int con_reg, period;

    // prescale 1:64
    con_reg =     T7_ON &
    T7_IDLE_CON &
    T7_GATE_OFF &
    T7_PS_1_8 &
    T7_SOURCE_INT;

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(8*frequency);

    OpenTimer7(con_reg, period);
    ConfigIntTimer7(T7_INT_PRIOR_6 & T7_INT_OFF);

    _T7IF = 0;
}

static void convolve(int numElems1,int numElems2,
        int* dstV,int* srcV1,int* srcV2) {
    int n;
    for (n = 0; n < numElems1 + numElems2 - 1; n++) {
        int kmin, kmax, k;
        dstV[n] = 0;
        kmin = (n >= numElems2 - 1) ? n - (numElems2 - 1) : 0;
        kmax = (n < numElems1 - 1) ? n : numElems1 - 1;
        for (k = kmin; k <= kmax; k++) {
            dstV[n] += srcV1[k] * srcV2[n - k];
        }
    }
}

// Swap sort for 6 items using the Bose-Nelson algorithm
static void sort6(unsigned char* d) {
#define SWAP(x,y) if (d[y] < d[x]) { int tmp = d[x]; d[x] = d[y]; d[y] = tmp; }
    SWAP(1, 2);
    SWAP(0, 2);
    SWAP(0, 1);
    SWAP(4, 5);
    SWAP(3, 5);
    SWAP(3, 4);
    SWAP(0, 3);
    SWAP(1, 4);
    SWAP(2, 5);
    SWAP(2, 4);
    SWAP(1, 3);
    SWAP(2, 3);
#undef SWAP
}

/**
 * Get the next available empty frame. If no frames are available, automatically
 * dequeues and returns the oldest full frame.
 *
 * @return Next available frame for writing
 */
static LineCam getEmptyFrame(void) {

    LineCam frame;

    frame = carrayPopHead(empty_frame_pool);
    if(frame == NULL) {
        frame = getOldestFullFrame(); // If no more empty frames, get oldest full
    }
    return frame;

}

/**
 * Enqueues a frame for writing into.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueEmptyFrame(LineCam frame) {

    carrayAddTail(empty_frame_pool, frame);

}

/**
 * Returns the oldest full frame in the outgoing buffer.
 *
 * @return Oldest full frame object
 */
static LineCam getOldestFullFrame(void) {

    LineCam frame;

    frame = carrayPopHead(full_frame_pool);
    if(carrayIsEmpty(full_frame_pool)) {
        has_new_frame = 0;
    }

    return frame;

}

/**
 * Enqueues a full frame object in the outgoing buffer.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueFullFrame(LineCam frame) {

    carrayAddTail(full_frame_pool, frame);
    has_new_frame = 1;

}