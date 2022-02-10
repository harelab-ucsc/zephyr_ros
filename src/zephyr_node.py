#!/usr/bin/env python3

 #"args": ["--address A4:34:F1:EA:1C:A2"]
import sys
import math

from matplotlib.pyplot import pause
import rospy
import std_msgs.msg

from core_z.zephyr_dev_async import *
from zephyr.msg import *

# import pyphysio as ph
import pyphysio.indicators.TimeDomain as td_ind
import pyphysio.indicators.FrequencyDomain as fd_ind
import pyphysio.indicators.NonLinearDomain as nl_ind
import pyphysio.indicators.PeaksDescription as pd_ind
from pyphysio import EvenlySignal

"""Need 30s per waveform for accurate detection (Heard & Adams Paper)

Breathing Waveform - Report Fq: 1.008Hz
                     Samples/Report: 18 samples
                     Time between samples: 56ms
                     Data Min Needed: 30 Waveforms
                    
ECG Waveform - Report Fq: 252Hz
               Samples/Report: 63 samples
               Time between samples: 4ms
               Data Min Needed: 119 Waveforms"""

class ROSActions(ZephyrDataActions):
    
    def __init__(self):
        super(ROSActions, self).__init__()
        #'topic name', msg type, queue_size = 10 <-standard queue size
        self.hr_pub = rospy.Publisher('heartrate', Heartrate, queue_size=10)
        self.hrv_pub = rospy.Publisher('hrv', HRV, queue_size=10)
        self.br_pub = rospy.Publisher('br', Breath, queue_size=10)
        
        self.ecg_pub = rospy.Publisher('ecg', ECG, queue_size=10)
        self.rwv_pub = rospy.Publisher('rwv', RWV, queue_size=10)

        self.rr_wv = []
        self.ecg_wv = []
        
        self.switch = 0
    def onSummary(self,msg):
        self.rate = rospy.Rate(10)
        header = std_msgs.msg.Header()
        stamp = rospy.Time.from_sec(msg.stamp)
        content = msg.as_dict()
        
        #Heart Rate 
        hr = Heartrate()
        hr.header = header
        hr.header.stamp = stamp
        hr.rate = content['heart_rate']
        hr.confidence = content['heart_rate_confidence']
        hr.reliable = not(content['heart_rate_unreliable'])
        self.hr_pub.publish(hr)
        
        #Heart Rate Variablity
        hrv = HRV()
        hrv.header = header
        hrv.header.stamp = stamp
        hrv.variation = content['heart_rate_variability']
        hrv.reliable = not(content['hrv_unreliable'])
        self.hrv_pub.publish(hrv)
        
        #Breathing Rate
        br = Breath()
        br.header = header
        br.header.stamp = stamp
        br.rate = content['respiration_rate']
        br.confidence = bool(content['device_worn_confidence'])
        br.reliable = not(content['respiration_rate_unreliable'])
        self.br_pub.publish(br)
        
        self.rate.sleep()
            
    def onECG(self, msg):
        self.ecg_wv.append(msg.waveform)
        # print(len(self.ecg_wv))
        if len(self.ecg_wv) >= 119: # 119 Waveforms needed for 30s of data
            header = std_msgs.msg.Header()
            stamp = rospy.Time.from_sec(msg.stamp)

            flat_ecg = [item for elem in self.ecg_wv for item in elem]
            ecg = EvenlySignal(values=flat_ecg, sampling_freq=63, signal_type='ecg')

            # Create Float32MultiArray msg object for waveform data
            wvf_array = std_msgs.msg.Float32MultiArray()
            wvf_array.layout.data_offset = 0 
            wvf_array.layout.dim = [std_msgs.msg.MultiArrayDimension()]
            wvf_array.layout.dim[0].label = "ecg_waveform"
            wvf_array.layout.dim[0].size = len(flat_ecg)
            wvf_array.layout.dim[0].stride = 1

            wvf_array.data = flat_ecg

            # Time Domain Indicators
            rmssd = td_ind.RMSSD(name="RMSSD")(ecg)
            sdsd = td_ind.SDSD(name="SDSD")(ecg)
            rr_mean = td_ind.Mean(name="Mean")(ecg)
            rr_std = td_ind.StDev(name="RRstd")(ecg)
            rr_median = td_ind.Median(name="Median")(ecg)
            mn = td_ind.Min(name="Min")(ecg)
            mx = td_ind.Max(name="Max")(ecg)
            # triang = td_ind.Triang(name="Triang")(ecg)
            # tinn = td_ind.Triang(name="TINN")(ecg)

            # Non-linear Domain Indicators
            pnn10 = nl_ind.PNNx(threshold=10, name="pnn10")(ecg)
            pnn25 = nl_ind.PNNx(threshold=25, name="pnn25")(ecg)
            pnn50 = nl_ind.PNNx(threshold=50, name="pnn50")(ecg)
            sd1 = nl_ind.PoincareSD1(name="sd1")(ecg)
            sd2 = nl_ind.PoincareSD2(name="sd2")(ecg)
            sd12 = nl_ind.PoincareSD1SD2(name="sd12")(ecg)
            sdell = nl_ind.PoinEll(name="sdell")(ecg)
            # Commented out because of long computation time
            # dfa1 = ph.indicators.NonLinearDomain.DFAShortTerm(name="DFA1")(ecg)
            # dfa2 = ph.indicators.NonLinearDomain.DFALongTerm(name="DFA2")(ecg)
            # approx_entr = ph.indicators.NonLinearDomain.ApproxEntropy(name="Approx_Entropy")(ecg)
            # samp_entr = ph.indicators.NonLinearDomain.SampleEntropy(name="Samp_Entropy")(ecg)

            # Frequency Domain Indicators
            method = 'fft'
            vlf = fd_ind.PowerInBand(interp_freq=4, freq_max=0.04, freq_min=0.00001, method=method, name="VLF_Pow")(ecg)
            lf = fd_ind.PowerInBand(interp_freq=4, freq_max=0.15, freq_min=0.04, method=method, name="LF_Pow")(ecg)
            hf = fd_ind.PowerInBand(interp_freq=4, freq_max=0.4, freq_min=0.15, method=method, name="HF_Pow")(ecg)
            total = fd_ind.PowerInBand(interp_freq=4, freq_max=2, freq_min=0.00001, method=method, name="Total_Pow")(ecg)
            
            # Message
            ecg_msg = ECG()
            ecg_msg.header = header
            ecg_msg.header.stamp = stamp
            ecg_msg.ecg_wvf = wvf_array
            ecg_msg.rmssd = rmssd
            ecg_msg.sdsd = sdsd
            ecg_msg.rr_mean = rr_mean
            ecg_msg.rr_std = rr_std
            ecg_msg.rr_median = rr_median
            ecg_msg.pnn10 = pnn10
            ecg_msg.pnn25 = pnn25
            ecg_msg.pnn50 = pnn50
            ecg_msg.min = mn
            ecg_msg.max = mx
            # ecg_msg.triang = triang
            # ecg_msg.tinn = tinn
            ecg_msg.sd1 = sd1
            ecg_msg.sd2 = sd2
            ecg_msg.sd12 = sd12
            ecg_msg.sdell = sdell
            # ecg_msg.dfa1 = dfa1
            # ecg_msg.dfa2 = dfa2
            # ecg_msg.approx_entr = approx_entr
            # ecg_msg.samp_entr = samp_entr
            ecg_msg.vlf_band = vlf
            ecg_msg.lf_band = lf
            ecg_msg.hf_band = hf
            ecg_msg.total_pwr = total

            self.ecg_pub.publish(ecg_msg)

            self.ecg_wv.pop(0)
            # self.rate.sleep()
            # ecg.plot('r')
            # pause(.001)

    def onBreath(self, msg):
        self.rr_wv.append(msg.waveform)
        
        if len(self.rr_wv) >= 30: # 30 Waveforms needed for 30s of data
            #Establish ROS Items
            self.rate = rospy.Rate(10)
            header = std_msgs.msg.Header()
            stamp = rospy.Time.from_sec(msg.stamp)

            #Calculate Features
            flat_rr = [item for elem in self.rr_wv for item in elem]
            original_data_num = len(flat_rr)
            new_rr = [x for x in flat_rr.copy() if math.isnan(x) == False] # removes nan's for feature extraction
            new_data_num = len(new_rr)
            reliability = new_data_num/original_data_num # gives us an idea of how many nan's were present

            rr = EvenlySignal(values = new_rr, sampling_freq = 18, signal_type = 'rr')

            # Create Float32MultiArray msg object for waveform data
            wvf_array = std_msgs.msg.Float32MultiArray()
            wvf_array.layout.data_offset = 0 
            wvf_array.layout.dim = [std_msgs.msg.MultiArrayDimension()]
            wvf_array.layout.dim[0].label = "resp_waveform"
            wvf_array.layout.dim[0].size = len(new_rr)
            wvf_array.layout.dim[0].stride = 1

            wvf_array.data = flat_rr # maintain original data with nan's for playback
            #rr.plot()
            r_rate = pd_ind.PeaksNum(delta=1)(rr) #bpm/2
            lf_band = fd_ind.PowerInBand(freq_min=0,freq_max=0.25,method='fft')(rr)
            hf_band = fd_ind.PowerInBand(freq_min=0.25,freq_max=5,method='fft')(rr)
            if hf_band != 0:
                pwr_ratio = lf_band/hf_band #I'm guessing it is lf/hf
            else:
                pwr_ratio = 0

            #Build and Publish Msg
            rwv = RWV()
            rwv.header = header
            rwv.header.stamp = stamp
            rwv.r_wvf = wvf_array
            rwv.r_rate = r_rate
            rwv.lf_band = lf_band
            rwv.hf_band = hf_band
            rwv.pwr_ratio = pwr_ratio
            rwv.reliability = reliability
            self.rwv_pub.publish(rwv)
            # try:
            #     self.rwv_pub.publish(rwv)
            # except Exception as e:
            #     print("ERROR:", e)
            # print("\n<< 7 >>\n")
            # print(self.rr_wv)
            
            self.rr_wv.pop(0)
            #pause(.001)
            # self.rate.sleep()
            

    @classmethod
    def timeParse(cls, previous_time, msg_split):        
        incoming_time = rospy.Time.now()
        time_step = (incoming_time-previous_time)/(msg_split-1)
        t_dur = int(time_step.nsecs)
        
        t_offset = [rospy.Duration(0,t_dur*i) for i in range(msg_split)]
        t_offset.reverse()
        t_base = [incoming_time for i in range(msg_split)]
        
        return list(map(lambda x,y: x-y, t_base,t_offset)), incoming_time
        
link = None
logger = logging.getLogger(__name__)
if __name__ == '__main__':
    rospy.init_node('zephyr_node', anonymous=True)
    rosargs = rospy.myargv(argv=sys.argv)
    myargs = getArgs(rosargs)  
    # if len(sys.argv) > 1:
    #     myargs.address = str(sys.argv[2])
    # else:
    #     pass
    #     # myargs.address = 'A4:34:F1:F1:67:8F'
    #     #myargs.address = 'test'
    myargs.address = ''
        
    actions = ROSActions()
    
    asyncio.ensure_future(init(actions,myargs))
    loop = asyncio.get_event_loop()
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Ctrl-C pressed.")