Header header
#Respiration Waveform msg with Features

# respiration waveform: The 30s of data used to calculate the below features
# Float32MultiArray  r_wvf

# r_rate: respiration rate
uint16  r_rate

# lf_band: low frequency band
float32 lf_band

# hf_band: low frequency banc
float32 hf_band

# pwr_ratio: power ratio between high and low frequency bands
float32 pwr_ratio
