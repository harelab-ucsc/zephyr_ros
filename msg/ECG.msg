Header header
# ECG Waveform msg with Features

# ecg_wvf: ECG waveform; The 30s of data used to calculate the below features
Float32MultiArray  ecg_wvf

# rmssd: the square root of the mean of the squared 1st order discrete differences
float32 rmssd

# sdsd: the standard deviation of the 1st order discrete differences
float32 sdsd

# rr_mean: mean of the ecg signal
float32 rr_mean

# rr_std: standard deviation of ecg signal
float32 rr_std

# rr_median: median of ecg signal
float32 rr_median

# min: minimum of the ecg signal
float32 min

# max: maximum of the ecg signal
float32 max

# triang: HRV triangular index
float32 triang

# tinn: the triangular interpolation of NN interval histogram
float32 tinn

# pnn10: Percentage of successive RR intervals that differ by more than 10ms
float32 pnn10

# pnn25: Percentage of successive RR intervals that differ by more than 25ms
float32 pnn25

# pnn50: Percentage of successive RR intervals that differ by more than 50ms
float32 pnn50

# sd1: Poincaré plot standard deviation perpendicular the line of identity
float32 sd1

# sd2: Poincaré plot standard deviation along the line of identity
float32 sd2

# sd12: Ratio of SD1-to-SD2
float32 sd12

# sdell: SD1*SD2*pi value of the Poincare' plot of input Inter Beat Intervals
float32 sdell

# dfa1: Detrended fluctuation analysis, which describes short-term fluctuations
# float32 dfa1

# dfa2: Detrended fluctuation analysis, which describes long-term fluctuations
# float32 dfa2

# approx_entr: Approximate entropy, which measures the regularity and complexity of a time series
# float32 approx_entr

# samp_entr: Sample entropy, which measures the regularity and complexity of a time series
# float32 samp_entr

# vlf_band: very low frequency band
float32 vlf_band

# lf_band: low frequency band
float32 lf_band

# hf_band: high frequency band
float32 hf_band

# total_pwr: total power in the band
float32 total_pwr
