import pandas as pd
import pickle
import sys
#sys.path.insert(0, '/home/sahabi/mo/lib/python')
from denoise.denoise import denoise
def run(steps, num_states, a, b, state_log_df):
    entry_log = []
    denoise_log = []
    for i in range(0, steps):
        counter = i+1
        entry = [0,0,0]
        action = state_log_df.Action[i]
        state = state_log_df['Previous State'][i]
        response = state_log_df['User Response'][i]
        entry[0] = state
        entry[1] = action
        entry[2] = response
        entry_log.append(entry)
        denoise_log.append(denoise(entry_log,a,b,num_states))
    denoise_logging = {'Maxflow': [i[0] for i in denoise_log],
    'Denoised_Image': [i[1] for i in denoise_log],'Final_Denoised_Image': [i[2] for i in denoise_log], 
    'Image_Builder': [i[3] for i in denoise_log]}

    return denoise_logging

state_log = pickle.load( open( "/home/sahabi/StateAction.p", "rb" ))
state_log_df = pd.DataFrame()
state_log_df = state_log_df.from_dict(state_log, orient='columns', dtype=None)
denoise_log_df = pd.DataFrame()
log = run(40,20,.1,.5,state_log_df)
denoise_log_df = denoise_log_df.from_dict(log, orient='columns', dtype=None)
denoise_log_df['Action'] = state_log_df.Action
denoise_log_df['State'] = state_log_df['Previous State']
denoise_log_df['Evaluation'] = state_log_df['User Response']
print denoise_log_df