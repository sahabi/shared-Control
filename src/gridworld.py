#!/usr/bin/env python
import pygame, sys
from pygame.locals import *
import os
import random
from time import sleep
from math import ceil
import pandas as pd
import rospy
from std_msgs.msg import Bool
from random import random
from bisect import bisect
from denoise.denoise import denoise
import pickle
import timeit
from gridsim.gridsim import Simulation

alpha_signal = []
result = 0


def weighted_choice(choices):
    values, weights = zip(*choices)
    total = 0
    cum_weights = []
    for w in weights:
        total += w
        cum_weights.append(total)
    x = random() * total
    i = bisect(cum_weights, x)
    return values[i]


def callback(data):
    global alpha_signal, subscription, subscribed, closed, responded, result
    alpha_signal.append(data.data)

    if len(alpha_signal) == 17:
        subscription.unregister()
        subscribed = False
        closed = make_decision()
        if closed:
            result = 1
        if not closed:
            result = 0
        alpha_signal = []
        responded = True
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", closed)


def make_decision():
    global alpha_signal
    if True in alpha_signal:
        return True
    else:
        return False

def get_action(actions_list):

   # action = random.choice(actions_set)
    action = weighted_choice(actions_list)
    return action

def get_response():
    global subscribed, subscription, responded, result, steps, counter
    subscription = rospy.Subscriber("/openbci/eyes_closed", Bool, callback)
    subscribed = True
    print 'Step {} of {}'.format(counter,steps)
    start = timeit.default_timer()
    while (not responded):
        pass
    stop = timeit.default_timer()
    print stop - start 
    #try:
    #    while (not responded):
    #        pass
    #except KeyboardInterrupt:
    #    pass
    responded = False
    return result

def start_simulation(steps):
    global closed, counter
    rospy.init_node('gridworld', anonymous=True)
    #start_2D_grid(initial_x_state, initial_y_state, num_x_states, num_y_states)
    sim = Simulation('/home/sahabi/mo/lib/python/gridsim/config.txt','hey')
    x_state_log = []
    y_state_log = []
    action_log = []
    response_log = []
    prev_x_state_log = []
    prev_y_state_log = []
    error_log = []
    x_denoise_log = []
    y_denoise_log = []
    x_entry_log = []
    y_entry_log = []
    clock = pygame.time.Clock()
    sim.draw()

    for i in range(0, steps):
        counter = i+1
        x_entry = [0,0,0]
        y_entry = [0,0,0]
        sleep(2)
        action_2D = get_action_2D([('north',15),('east',35),('south',35),('west',15)])
        action_log.append(action_2D)

        if action_2D == 'north':
            y_entry[1] = 0
            x_entry[1] = 0
        elif action_2D == 'east':
            y_entry[1] = 0
            x_entry[1] = 1
        elif action_2D == 'south':
            y_entry[1] = 1
            x_entry[1] = 0
        elif action_2D == 'west':
            y_entry[1] = 0
            x_entry[1] = 0

        blocks = sim.export_state()

        prev_x_state = blocks["agents"][0][0]
        prev_y_state = blocks["agents"][0][1]
        x_entry[0] = blocks["agents"][0][0]
        y_entry[0] = blocks["agents"][0][1]

        prev_x_state_log.append(prev_x_state)
        prev_y_state_log.append(prev_y_state)

        sim.move_agent(0, action_2D)

        blocks = sim.export_state()

        x_state = blocks["agents"][0][0]
        y_state = blocks["agents"][0][1]

        x_state_log.append(x_state)
        y_state_log.append(y_state)

        sim.draw()

        clock.tick(1)

        for event in pygame.event.get():
            #if the user wants to quit
            if event.type == QUIT:
                #end the game and close the window
                pygame.quit()
                sys.exit()

        response = get_response()
        x_entry[2] = response
        y_entry[2] = response
        response_log.append(response)

        x_entry_log.append(x_entry)
        y_entry_log.append(y_entry)

        x_denoise_log.append(denoise(x_entry_log,.3,.5,10))
        y_denoise_log.append(denoise(y_entry_log,.3,.5,10))

        if action_2D == 'east' and response == 1:
            error_log.append(False)
        elif action_2D == 'south' and response == 1:            
            error_log.append(False)
        elif action_2D == 'north' and response == 0:
            error_log.append(False)
        elif action_2D == 'west' and response  == 0:
            error_log.append(False)
        else:
            error_log.append(True)

    x_state_logging = {'Previous_x_State': prev_x_state_log, 'Action': action_log, 
                'New_x_State': x_state_log, 'User_Response': response_log, 'Error': error_log}

    y_state_logging = {'Previous_y_State': prev_y_state_log, 'Action': action_log, 
                'New_y_State': y_state_log, 'User_Response': response_log, 'Error': error_log}

    x_denoise_logging = {'Maxflow': [i[0] for i in x_denoise_log],'Denoised_x_Image': [i[1] for i in x_denoise_log],
    'Final_Denoised_x_Image': [i[2] for i in x_denoise_log],'x_Image': [i[3] for i in x_denoise_log]}
    y_denoise_logging = {'Maxflow': [i[0] for i in y_denoise_log],'Denoised_y_Image': [i[1] for i in y_denoise_log],
    'Final_Denoised_y_Image': [i[2] for i in y_denoise_log],'y_Image': [i[3] for i in y_denoise_log]}

    x_merged_log = x_state_logging.copy()
    x_merged_log.update(x_denoise_logging)
    y_merged_log = y_state_logging.copy()
    y_merged_log.update(y_denoise_logging)

    return (x_merged_log, y_merged_log)


if __name__=="__main__":

    counter = 0
    responded = False
    subscribed = False
    closed = False
    subscription = 0
    steps = int(sys.argv[1])

    log = start_simulation(steps)

    log_x_df = pd.DataFrame()
    log_x_df = log_x_df.from_dict(log[0], orient='columns', dtype=None)
    log_y_df = pd.DataFrame()
    log_y_df = log_y_df.from_dict(log[1], orient='columns', dtype=None)
    #pickle.dump( log[0], open( "StateAction.p", "wb" ) )
    #pickle.dump( log[1], open( "Denoised.p", "wb" ) )
    log_x_df.to_pickle('log_x.p')
    log_x_df.to_pickle('log_y.p')
    print log_x_df.Error.value_counts()