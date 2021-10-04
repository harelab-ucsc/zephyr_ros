Header header
# EDA Waveform msg with Features

# eda_wvf: EDA waveform; The 30s of data used to calculate the below features
Float32MultiArray  eda_wvf

# p_wvf: phasic waveform
Float32MultiArray  p_wvf

# p_mean: phasic mean
float32  p_mean

# p_std: phasic standard deviation
float32 p_std

# p_range: phasic range
float32 p_range

# pks_max: phasic max peak
float32 pks_max

# pks_min: phasic min peak
float32 pks_min

# pks_mean: phasic mean peak
float32 pks_mean

# pks_n: number of phasic peaks
float32 pks_n

# dur_mean: mean duration of phasic peaks
float32 dur_mean

# slopes_mean: mean slope of phasic peaks
float32 slopes_mean

# p_auc: area under the phasic curve
float32 p_auc

# t_wvf: tonic waveform
Float32MultiArray  t_wvf

# t_mean: tonic mean
float32  t_mean

# t_std: tonic standard deviation
float32 t_std

# t_range: tonic range
float32 t_range

# t_auc: area under the tonic curve
float32 t_auc



