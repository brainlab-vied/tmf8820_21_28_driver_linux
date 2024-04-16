# *****************************************************************************
# * Copyright by ams AG                                                       *
# * All rights are reserved.                                                  *
# *                                                                           *
# * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
# * THE SOFTWARE.                                                             *
# *                                                                           *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
# * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,      *
# * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY      *
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
# *****************************************************************************

import ctypes
import zmq

RASPI_IP_ADDR  = '169.254.0.2'
RASPI_ZMQ_PORT = 8083

ZMQ_URI = "tcp://{}:{}".format(RASPI_IP_ADDR, RASPI_ZMQ_PORT)

# constants for zmq message parsing
MAX_MSG_SIZE     = 8192
MAX_NUM_RESULTS  = 36
MAX_BINS         = 256
MAX_TDC          = 5
MAX_TDC_CHANNELS = 2 * MAX_TDC

# constants for logging
HISTOGRAM_ID_MEASUREMENT             = 3
ERROR_ID                             = 0xF
NUMBER_OF_CAPTURES_IN_8X8_MODE       = 4
HISTOGRAMS_PER_CAPTURE_IN_8X8        = 20
HISTOGRAMS_PER_SUBCAPTURE            = 10
NUMBER_OF_TDC_CHANNELS               = 2
BINS_PER_TDC_CHANNEL                 = 128
MAX_NUMBER_OF_NON_HISTOGRAM_MESSAGES = 100

class Tmf8820_msg_header(ctypes.Structure):
    _fields_ = [
            ('id', ctypes.c_int),
            ('len', ctypes.c_int),
    ]

class Tmf8820_msg_error(ctypes.Structure):
    _fields_ = [
            ('hdr', Tmf8820_msg_header),
            ('error_code', ctypes.c_int),
    ]

class Tmf8820_meas_result(ctypes.Structure):
    _fields_ = [
            ('confidence', ctypes.c_int),
            ('distance_mm', ctypes.c_int),
            ('channel', ctypes.c_int),
            ('ch_target_idx', ctypes.c_int),
            ('sub_capture', ctypes.c_int),
    ]

class Tmf8820_msg_meas_results(ctypes.Structure):
    _fields_ = [
            ('hdr', Tmf8820_msg_header),
            ('result_num', ctypes.c_int),
            ('temperature', ctypes.c_int),
            ('ambient_light', ctypes.c_int),
            ('photon_count', ctypes.c_int),
            ('ref_photon_count', ctypes.c_int),
            ('sys_ticks', ctypes.c_int),
            ('valid_results', ctypes.c_int),
            ('num_results', ctypes.c_int),
            ('results', Tmf8820_meas_result * MAX_NUM_RESULTS),
    ]

class Tmf8820_msg_meas_stats(ctypes.Structure):
    _fields_ = [
            ('hdr', Tmf8820_msg_header),
            ('capture_num', ctypes.c_int),
            ('sub_capture', ctypes.c_int),
            ('tdcif_status', ctypes.c_int),
            ('iterations_configured', ctypes.c_int),
            ('remaining_iterations', ctypes.c_int),
            ('accumulated_hits', ctypes.c_int),
            ('raw_hits', ctypes.c_int * MAX_TDC),
            ('saturation_cnt', ctypes.c_int * MAX_TDC),
    ]

class Tmf8820_msg_histogram(ctypes.Structure):
    _fields_ = [
            ('hdr', Tmf8820_msg_header),
            ('capture_num', ctypes.c_int),
            ('sub_capture', ctypes.c_int),
            ('histogram_type', ctypes.c_int),
            ('num_tdc', ctypes.c_int),
            ('num_bins', ctypes.c_int),
            ('bins', (ctypes.c_int * MAX_BINS) * MAX_TDC),
    ]

class Tmf8820_msg(ctypes.Union):
    _fields_ = [
            ('hdr', Tmf8820_msg_header),
            ('err_msg', Tmf8820_msg_error),
            ('hist_msg', Tmf8820_msg_histogram),
            ('meas_result_msg', Tmf8820_msg_meas_results),
            ('meas_stat_msg', Tmf8820_msg_meas_stats),
            ('msg_buf', ctypes.c_byte * MAX_MSG_SIZE),
    ]
    def __new__(cls, buf):
        data = bytearray(buf) + bytearray([0] * MAX_MSG_SIZE)
        return cls.from_buffer_copy(data)
    def __init__(self, buf):
        pass


# pixel positions
#  1  2  3  4  5  6  7  8
#  9 10 11 12 13 14 15 16
# 17 18 19 20 21 22 23 24
# 25 26 27 28 29 30 31 32
# 33 34 35 36 37 38 39 40
# 41 42 43 44 45 46 47 48
# 49 50 51 52 53 54 55 56
# 57 58 59 60 61 62 63 64

# map histogram numbers to pixel in the 8x8 matrix
pixelMap = {
     1 : 57,  2 : 61,  3 : 41,  4 : 45,  5 : 25,  6 : 29,  7 : 9 , 8 : 13,
    11 : 58, 12 : 62, 13 : 42, 14 : 46, 15 : 26, 16 : 30, 17 : 10,18 : 14,
    21 : 59, 22 : 63, 23 : 43, 24 : 47, 25 : 27, 26 : 31, 27 : 11,28 : 15,
    31 : 60, 32 : 64, 33 : 44, 34 : 48, 35 : 28, 36 : 32, 37 : 12,38 : 16,
    41 : 49, 42 : 53, 43 : 33, 44 : 37, 45 : 17, 46 : 21, 47 : 1 ,48 : 5 ,
    51 : 50, 52 : 54, 53 : 34, 54 : 38, 55 : 18, 56 : 22, 57 : 2 ,58 : 6 ,
    61 : 51, 62 : 55, 63 : 35, 64 : 39, 65 : 19, 66 : 23, 67 : 3 ,68 : 7 ,
    71 : 52, 72 : 56, 73 : 36, 74 : 40, 75 : 20, 76 : 24, 77 : 4 ,78 : 8 
}

fudge_res_num = 0
def calc_zn(res_num, sub_capture, ch):
    global fudge_res_num
    spad_ch_2_zone = [   0,   # unused
        39, 47, 55, 63, 40, 48, 56, 64, 7, 15, 23, 
        31, 8, 16, 24, 32, 37, 45, 53, 61, 38, 46, 
        54, 62, 5, 13, 21, 29, 6, 14, 22, 30, 35,
        43, 51, 59, 36, 44, 52, 60, 3, 11, 19, 27,
        4, 12, 20, 28, 33, 41, 49, 57, 34, 42, 50, 
        58, 1, 9, 17, 25, 2, 10, 18, 26]       # spad_ch is 1 to 64
    seq_step = (res_num + fudge_res_num) % 4   # one time-multiplexed result (2 captures)
    spad_ch = ch + 8*sub_capture
    spad_ch += seq_step*16

    zn = spad_ch_2_zone[spad_ch]  # 1-64 in 8x8 order
    return zn

# the 1-64 zones are mapped into the logfile obj columns
# zn(1 to 64) -> obj_pix (1 to 71)   (72 is always 0's)
zn_2_obj_pix = [ 0 ]   + \
    list(range(1,9))   + \
    list(range(10,18)) + \
    list(range(19,27)) + \
    list(range(28,36)) + \
    list(range(37,45)) + \
    list(range(46,54)) + \
    list(range(55,63)) + \
    list(range(64,72))

def calc_obj_pix(res_num, sub_capture, ch):
    zn = calc_zn(res_num, sub_capture, ch)
    return zn_2_obj_pix[zn]

logfile_obj_accum = list()    # indexing starts at 0 with pixel 1 object 0, pixel 1 object 1...

def clear_obj_entries():
    global logfile_obj_accum
    logfile_obj_accum = list()
    for x in range(72):
        logfile_obj_accum.append(Tmf8820_meas_result())    # object 0
        logfile_obj_accum[x*2].confidence      = 0
        logfile_obj_accum[x*2].distance_mm     = 0
        logfile_obj_accum[x*2].channel         = 0
        logfile_obj_accum[x*2].ch_target_idx   = 0
        logfile_obj_accum[x*2].sub_capture     = 0
        logfile_obj_accum.append(Tmf8820_meas_result())    # object 1
        logfile_obj_accum[x*2+1].confidence    = 0
        logfile_obj_accum[x*2+1].distance_mm   = 0
        logfile_obj_accum[x*2+1].channel       = 0
        logfile_obj_accum[x*2+1].ch_target_idx = 0
        logfile_obj_accum[x*2+1].sub_capture   = 0

clear_obj_entries()

def set_obj_entry(res_num, sub_capture, ch, ch_target_idx, distance, confidence):
    global logfile_obj_accum
    # ch_target_idx is object 0 or 1
    obji = (calc_obj_pix(res_num, sub_capture, ch) - 1) * 2 + ch_target_idx
    # print(f"obji {obji}")

    # add to global structure that will be the #OBJ log file pixel data
    logfile_obj_accum[obji].confidence    = confidence
    logfile_obj_accum[obji].distance_mm   = distance
    logfile_obj_accum[obji].channel       = ch
    logfile_obj_accum[obji].ch_target_idx = ch_target_idx
    logfile_obj_accum[obji].sub_capture   = sub_capture

def mapHistogramNumber( histogram, numbers=False ):
    if numbers:
        if histogram in pixelMap:
            return pixelMap[histogram]
        else:
            return 0
    else:             # strings
        if histogram in pixelMap:
            return f"{pixelMap[histogram]:02}"
        else:
            return ""
 
def connectToRaspi( ):
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(ZMQ_URI)
    return sub


# getAllHistogramsIn8x8Mode( sub, do_function=None)
# the function "do_function" has input of 1 histogram, and outputs anything

def getAllHistogramsIn8x8Mode( sub, do_function=None):
    global logfile_obj_accum

    logString = ""

    # initialize a results structure -- will be filled as results are received
    clear_obj_entries()

    # do not log until the first capture of a sequence is found
    logging = False

    # flags for detecting if we are getting the correct messages
    numberOfNonHistogramMessages = 0
    lastSubCapture = -1

    functions_val = dict()

    preflush = 0      # count of histograms read in for purposes of flushing the input buffer
    while True:
        msg = Tmf8820_msg(sub.recv())

        # Histogram message
        if msg.hdr.id == HISTOGRAM_ID_MEASUREMENT:
            
            numberOfNonHistogramMessages = 0
            if msg.hist_msg.sub_capture == 1:
                lastSubCapture = 1

            preflush += 1
            if preflush > 10:
                hist = msg.hist_msg
    
                if ( lastSubCapture < 1):
                    logstr = "#ERROR;Time multiplexed measurement not enabled. Exiting."
                    print(logstr)
                    return logstr
                else:
                    #lastSubCapture = hist.sub_capture
                    pass
    
                isFirstCapture = (hist.capture_num % NUMBER_OF_CAPTURES_IN_8X8_MODE == 0) and ( hist.sub_capture == 0 )
    
                # if isFirstCapture:
                #     print(f"HISTOGRAM_ID_MEASUREMENT preflush {preflush} isFirstCapture")
                # else:
                #     print(f"HISTOGRAM_ID_MEASUREMENT preflush {preflush}")
                # did we capture and log a complete sequence?
                if ( logging and isFirstCapture ):
                    break
    
                # did we find the first capture in a new sequence?
                if ( ( not logging ) and isFirstCapture):
                    logging = True
    
                if logging:
                    logString += f"#HISTINFO;Transaction ID: {hist.capture_num};Capture: {hist.capture_num % NUMBER_OF_CAPTURES_IN_8X8_MODE};Type: {hist.histogram_type};num_tdc: {hist.num_tdc};num_bins: {hist.num_bins};sub_capture: {hist.sub_capture}\n"
    
                    for tdc in range(0,hist.num_tdc):
                        currentHistogram = ( hist.capture_num % NUMBER_OF_CAPTURES_IN_8X8_MODE) * HISTOGRAMS_PER_CAPTURE_IN_8X8 + hist.sub_capture*HISTOGRAMS_PER_SUBCAPTURE + tdc*NUMBER_OF_TDC_CHANNELS
    
                        histogram_a = hist.bins[tdc][:BINS_PER_TDC_CHANNEL]
                        histogram_b = hist.bins[tdc][BINS_PER_TDC_CHANNEL:]

                        if do_function:            # a function was specified (it wasn't None)
                            functions_val.update({currentHistogram+0: do_function(histogram_a)})
                            functions_val.update({currentHistogram+1: do_function(histogram_b)})

                        binString = ""
    
                        for binValue in hist.bins[tdc][:BINS_PER_TDC_CHANNEL]:
                            binString += f";{binValue}"
    
                        logString += f"#HIST{mapHistogramNumber(currentHistogram+0)}{binString}\n"
    
                        binString = ""
    
                        for binValue in hist.bins[tdc][BINS_PER_TDC_CHANNEL:]:
                            binString += f";{binValue}"
    
                        logString += f"#HIST{mapHistogramNumber(currentHistogram+1)}{binString}\n"
            else:        # preflush < n
                pass
        elif msg.hdr.id == 1:
            if preflush > 10:   # TBD in order to get exact correlation w/histograms
                res = msg.meas_result_msg
                for r in range(res.valid_results):
                    # print("OBJ data res_num: {} num_res: {} ch: {} sub_capture: {} ch_target_idx: {} distance: {}".format(res.result_num,
                    #     res.num_results,
                    #     res.results[r].channel,
                    #     res.results[r].sub_capture,
                    #     res.results[r].ch_target_idx,
                    #     res.results[r].distance_mm))
                    # add to logfile_obj_accum[]
                    set_obj_entry(
                        res.result_num,
                        res.results[r].sub_capture,
                        res.results[r].channel,
                        res.results[r].ch_target_idx,
                        res.results[r].distance_mm,
                        res.results[r].confidence)
        else:
            # prevent endless loops if histogram dumping is not enabled
            numberOfNonHistogramMessages += 1
            if ( numberOfNonHistogramMessages > MAX_NUMBER_OF_NON_HISTOGRAM_MESSAGES ):
                logstr = "#ERROR;Histogram dumping not enabled. Exiting." 
                print(logstr)
                return logstr
        # error message, exit immediately
        if msg.hdr.id == ERROR_ID:
            err = msg.err_msg
            logstr = f"#ERROR;CODE: {err.error_code}\n"
            print(logstr)
            return logstr

    logString += f"#OBJ;12345678;0;8;8"
    for opix in range(1,73):
        confidence = logfile_obj_accum[(opix - 1)*2 + 0].confidence
        distance   = logfile_obj_accum[(opix - 1)*2 + 0].distance_mm
        logString += f";{distance};{confidence}"
        confidence = logfile_obj_accum[(opix - 1)*2 + 1].confidence
        distance   = logfile_obj_accum[(opix - 1)*2 + 1].distance_mm
        logString += f";{distance};{confidence}"
    logString += f"\n"

    if do_function:
        return logString, functions_val
    else:
        return logString


# TBD this needs the #OBJ addition like the 8x8 capture above

def getAllHistogramsIn4x4Mode( sub, returnzones=[]):
    
    logString = ""
    hlist = []

    # do not log until the first capture of a sequence is found
    logging = False

    # flags for detecting if we are getting the correct messages
    messagesSinceHistogram = 0
    lastSubCapture = -1

    preflush = 0      # count of histograms read in for purposes of flushing the input buffer
    while True:
        msg = Tmf8820_msg(sub.recv())
        messagesSinceHistogram += 1

        # Result message
        if msg.hdr.id == 1:
            res = msg.meas_result_msg
            # print("results res_num: {}\tnum_res: {}".format(res.result_num,
            #                                                 res.num_results))
            # for r in range(res.valid_results):
            #     print("ch: {} sub_capture: {} ch_target_idx: {} distance: {}".format(res.results[r].channel,
            #                                                                      res.results[r].sub_capture,
            #                                                                      res.results[r].ch_target_idx,
            #                                                                      res.results[r].distance_mm))

        # Stats message
        elif msg.hdr.id == 2:
            stats = msg.meas_stat_msg
            # print("stats capture_num: {} acc hits: {}".format(stats.capture_num,
            #                                                   stats.accumulated_hits))


        # Histogram message
        # "histogram_type" excludes some extraneous histograms (electrical cal?)
        elif msg.hdr.id == HISTOGRAM_ID_MEASUREMENT:       #  and msg.hist_msg.histogram_type != 0:
    
            hist = msg.hist_msg
            messagesSinceHistogram = 0

            preflush += 1
            if preflush > 40:
                # print("histogram message type: {}".format(hist.histogram_type))
                if hist.histogram_type == 1:   #  or hist.num_bins == 256:
                    for x in range(hist.num_tdc):
                        pass
                        # print("Electrical Calibration TDC[{}] max val: {}".format(x, max(hist.bins[x])))
                else:
                    # print("hist.capture_num {} hist.sub_capture {} lastSubCapture {} msg.hist_msg.histogram_type {}"
                    #     .format(hist.capture_num, hist.sub_capture, lastSubCapture, msg.hist_msg.histogram_type))
                    
                    lastSubCapture = hist.sub_capture
    
                    startOfMultiplexCapture =  hist.sub_capture == 0
    
                    # did we capture and log a complete sequence?
                    if ( logging and startOfMultiplexCapture ):            # have reached the end of a complete sequence
                        break
    
                    # did we find the first capture in a new sequence?
                    if ( ( not logging ) and startOfMultiplexCapture):
                        logging = True                                     # begin loggin the histograms
    
                    if logging:
                        # print("logging it....")
                        logString += f"#HISTINFO;Transaction ID: {hist.capture_num};Capture: {hist.capture_num % NUMBER_OF_CAPTURES_IN_8X8_MODE};Type: {hist.histogram_type};num_tdc: {hist.num_tdc};num_bins: {hist.num_bins};sub_capture: {hist.sub_capture}\n"
    
                        for tdc in range(0, hist.num_tdc):
                            histogramNumber =  hist.sub_capture*HISTOGRAMS_PER_SUBCAPTURE + tdc*NUMBER_OF_TDC_CHANNELS

                            binString = ""
                            bins = []
    
                            for binValue in hist.bins[tdc][:BINS_PER_TDC_CHANNEL]:
                                binString += f";{binValue}"
                                bins.append(binValue)
    
                            logString += f"#HIST{histogramNumber}{binString}\n"
                            if (histogramNumber in returnzones):
                                hlist.append(bins)
    
                            binString = ""
                            bins = []
    
                            for binValue in hist.bins[tdc][BINS_PER_TDC_CHANNEL:]:
                                binString += f";{binValue}"
                                bins.append(binValue)
    
                            logString += f"#HIST{histogramNumber+1}{binString}\n"
                            if (histogramNumber + 1 in returnzones):
                                hlist.append(bins)
                    else:          # not logging
                        pass
            else:    # preflush < n
                pass

        elif msg.hdr.id == ERROR_ID:
            err = msg.err_msg
            return f"#ERROR;CODE: {err.error_code}\n"

        if ( messagesSinceHistogram > MAX_NUMBER_OF_NON_HISTOGRAM_MESSAGES ):
            print("too many non-histogram messages")
            return "#ERROR;Histogram dumping not enabled. Exiting."

    if (returnzones == []):
        return logString
    else:
        return logString, hlist

def filterNonPixel(vals):
    used8x8 = [   False,True,True,True,True,True,True,True,True,False,False,True,True,True,True,True,
                  True,True,True,False,False,True,True,True,True,True,True,True,True,False,False,True,
                  True,True,True,True,True,True,True,False,False,True,True,True,True,True,True,True,
                  True,False,False,True,True,True,True,True,True,True,True,False,False,True,True,True,
                  True,True,True,True,True,False,False,True,True,True,True,True,True,True,True,False]
    return [vals[x] for x in range(80) if used8x8[x]]

def calc_crosstalk(h):
    return max(h[5:20])
    

if __name__ == "__main__":
    zmqSocket = connectToRaspi()
    print( getAllHistogramsIn8x8Mode( zmqSocket ) )

    _ , crosstalks = getAllHistogramsIn8x8Mode( zmqSocket, do_function = calc_crosstalk)
    pixel_crosstalks = filterNonPixel(crosstalks)

    print("Pixel crosstalks", pixel_crosstalks)
    print("minumum crosstalk", min(pixel_crosstalks))

    zmqSocket.close()


