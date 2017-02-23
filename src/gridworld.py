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
from random import random, choice
from bisect import bisect
from denoise.denoise import denoise
import pickle
import timeit
from gridsim.gridsim import Simulation
from action_selection.action_selection import init_probs, pick_action, det_k, update_P, det_all_k, init_k, update_all_P, get_max, get_target, manhattan_dist
import operator

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

    if len(alpha_signal) == 18:
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


def get_autoresponse(old_x,old_y,new_x,new_y,target):
    if manhattan_dist((old_x,old_y),target) <= manhattan_dist((new_x,new_y),target):
        return weighted_choice([(0,70),(1,30)])
    else:
        return weighted_choice([(1,70),(0,30)])

def get_response():
    global subscribed, subscription, responded, result, steps, counter
    subscription = rospy.Subscriber("/openbci/eyes_closed", Bool, callback)
    subscribed = True
    
    start = timeit.default_timer()
    while (not responded):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        pass
    stop = timeit.default_timer()
    print stop - start 
    responded = False
    return result

def start_simulation(x_size, y_size, steps, param, beta, correction=True):
    "this function works"

    global closed, counter, lock_80, lock_90, lock_95, table, lock_80n, lock_90n, lock_95n
    #rospy.init_node('gridworld', anonymous=True)
    #start_2D_grid(initial_x_state, initial_y_state, num_x_states, num_y_states)
    sim = Simulation('gridsim/config.txt','gridsim/matrix.txt')
    x_state_log = []
    y_state_log = []
    action_log = []
    action_x_log = []
    action_y_log = []
    response_log = []
    prev_x_state_log = []
    prev_y_state_log = []
    error_log = []
    error_x_log = []
    error_y_log = []
    x_denoise_log = []
    y_denoise_log = []
    x_entry_log = []
    y_entry_log = []
    current_state_list = []
    time_step_list = []
    evaluation_list = []
    done = sim.update()


    p = init_probs(x_size, y_size)
    s = init_k(x_size, y_size)
    

    current_state = (0, 0)
    current_state_list.append(current_state)
    x_state_logger = []
    y_state_logger = []
    x_state_logger.append(current_state[0])
    y_state_logger.append(current_state[1])
    response_x_log = []
    response_y_log = []
    evaluation_list_log = []
    target = (2, 2)
    goal_value_log = []
    goal_value_log.append(p[target])
    
    actions = ['east','west','north','south']

    for i in range(0, steps):
        counter = i+1
        x_entry = [0,0,0,0,0]
        y_entry = [0,0,0,0,0]
        #sleep(3.01)
        blocks = sim.get_state()
         
        time_step_list.append(counter-1)
        action  =  pick_action(current_state, actions, p, x_size, y_size, counter - 1)
        if action == 'stay':
            break
        #action = get_action([('north',15),('east',35),('south',35),('west',15)])
        action_log.append(action)

        if action == 'north':
            y_entry[1] = 0
            #x_entry[1] = 0
        elif action == 'east':
            #y_entry[1] = 0
            x_entry[1] = 1
        elif action == 'south':
            y_entry[1] = 1
            #x_entry[1] = 0
        elif action == 'west':
            #y_entry[1] = 0
            x_entry[1] = 0

        S_max = get_max(p)
        S_target = get_target(current_state,S_max)

        # for j in range(y_size):
        #     print '\n'
        #     print '\t',
        #     for i in range(x_size):
        #         print "%.3f" % p[(i,j)],
        #         print '\t',
        #     for i in range(x_size):
        #         if (i,j) == current_state:
        #             print " * ",
        #         elif (i,j) == S_target:
        #             print " X ",
        #         elif (i,j) in S_max:
        #             print " O ",
        #         else:
        #             print " _ ",
           
        # print '\n'

        
        prev_x_state = blocks["agents"][0][0]
        prev_y_state = blocks["agents"][0][1]
        
        sim.move_agent(0, action)

        # prev_x_state_log.append(prev_x_state)
        # prev_y_state_log.append(prev_y_state)

        done = sim.update()
        blocks = sim.get_state()
        current_state = blocks["agents"][0]
        x_state_logger.append(current_state[0])
        y_state_logger.append(current_state[1])
        
        x_state = blocks["agents"][0][0]
        y_state = blocks["agents"][0][1]
        current_state_list.append(current_state)
        # x_state_log.append(x_state)
        # y_state_log.append(y_state)  
        # response = get_response()
        response = get_autoresponse(prev_x_state, prev_y_state, x_state, y_state, target)
        #response = choice([1])
        response_log.append(response)

        if done:
            pygame.quit()
            sys.exit()

        #print prev_x_state, prev_y_state, x_state, y_state, target, response

        #print 'Step {} of {}'.format(counter,steps)

        if action == 'west' or action == 'east':
            x_state_log.append(x_state)
            prev_x_state_log.append(prev_x_state)
            response_x = response
            x_entry[0] = prev_x_state
            x_entry[2] = response_x
            x_entry[3] = prev_x_state
            x_entry[4] = counter - 1
            response_x_log.append(response_x)
            evaluation_list.append(response_x)
            x_entry_log.append(x_entry)
            x_denoise_log.append(denoise(x_entry_log,param,beta))
        if action == 'south' or action == 'north':
            y_state_log.append(y_state) 
            prev_y_state_log.append(prev_y_state)
            response_y = response
            y_entry[0] = prev_y_state
            y_entry[2] = response_y
            y_entry[3] = prev_y_state
            y_entry[4] = counter - 1
            response_y_log.append(response_y)
            evaluation_list.append(response_y)
            y_entry_log.append(y_entry)
            y_denoise_log.append(denoise(y_entry_log,param,beta))

        old_ev_list = evaluation_list[:]
        
        if correction:
            for action in action_log:
                if action == 'east' or action == 'west':
                    for index,column in enumerate(x_denoise_log[-1][5]):
                        for x,element in enumerate(column):
                            if type(element) == list:
                                if element[0] != -1:
                                    if element[0] != x_denoise_log[-1][2][index][x] and (action_log[element[1]] == 'east' or action_log[element[1]] == 'west'):
                                        if response_log[element[1]] == 1:
                                            evaluation_list[element[1]] = 0
                                        elif response_log[element[1]] == 0:
                                            evaluation_list[element[1]] = 1
                                    else:
                                        evaluation_list[element[1]] =  response_log[element[1]]            

                elif action == 'south' or action == 'north':
                    for index,column in enumerate(y_denoise_log[-1][5]):
                        for x,element in enumerate(column):
                            if type(element) == list:
                                if element[0] != -1:
                                    if element[0] != y_denoise_log[-1][2][index][x] and (action_log[element[1]] == 'north' or action_log[element[1]] == 'south'):
                                        if response_log[element[1]] == 1:
                                            evaluation_list[element[1]] = 0
                                        elif response_log[element[1]] == 0:
                                            evaluation_list[element[1]] = 1
                                    else:
                                        evaluation_list[element[1]] =  response_log[element[1]]  

 
        diff = sum(map(abs,map(operator.sub, old_ev_list, evaluation_list)))

        s = init_k(x_size, y_size)
        all_K = det_all_k(s,current_state_list[:-1],action_log,evaluation_list)
        
        p = init_probs(x_size, y_size)
        p = update_all_P(p,all_K)
        goal_value_log.append(p[target])

        if action == 'south' or action == 'north':
            action_y_log.append(action)
            if manhattan_dist(current_state_list[-2],target) <= manhattan_dist(current_state_list[-1],target) and response == 1:
                error_y_log.append(1)
                error_log.append(1)
            elif manhattan_dist(current_state_list[-2],target) >= manhattan_dist(current_state_list[-1],target) and response == 0:            
                error_y_log.append(1)
                error_log.append(1)
            else:
                error_y_log.append(0)
                error_log.append(0)

        elif action == 'east' or action == 'west':
            action_x_log.append(action)
            if manhattan_dist(current_state_list[-2],target) <= manhattan_dist(current_state_list[-1],target) and response == 1:
                error_x_log.append(1)
                error_log.append(1)
            elif manhattan_dist(current_state_list[-2],target) >= manhattan_dist(current_state_list[-1],target) and response == 0:            
                error_x_log.append(1)
                error_log.append(1)
            else:
                error_x_log.append(0)
                error_log.append(0)
        error_prob =  sum(error_log)/float(len(error_log))

        #print p[target]

        if (p[(target[0],target[1]+1)] > .80 or p[(target[0],target[1]-1)] > .80 or p[(target[0]+1,target[1])] > .80 or p[(target[0]-1,target[1])] > .80) and not lock_80n:
            lock_80n = True
            table['ts_80n'].append((counter,trial))
            table['er_80n'].append(error_prob)
        elif (p[(target[0],target[1]+1)] > .90 or p[(target[0],target[1]-1)] > .90 or p[(target[0]+1,target[1])] > .90 or p[(target[0]-1,target[1])] > .90) and not lock_90n:
            lock_90n = True
            table['ts_90n'].append(counter)
            table['er_90n'].append(error_prob)
        elif (p[(target[0],target[1]+1)] > .95 or p[(target[0],target[1]-1)] > .95 or p[(target[0]+1,target[1])] > .95 or p[(target[0]-1,target[1])] > .95) and not lock_95n:
            lock_95n = True
            table['ts_95n'].append(counter)
            table['er_95n'].append(error_prob)

        if p[target] > .80 and not lock_80:
            lock_80 = True
            table['ts_80'].append(counter)
            table['er_80'].append(error_prob)
        elif p[target] > .90 and not lock_90:
            lock_90 = True
            table['ts_90'].append(counter)
            table['er_90'].append(error_prob)
        elif p[target] > .95 and not lock_95:
            lock_95 = True
            table['ts_95'].append(counter)
            table['er_95'].append(error_prob)
            table['er'].append(error_prob)
            print '{}/{} \r'.format(trial, trials),
            sys.stdout.flush()
            lock_80 = False
            lock_90 = False
            lock_95 = False
            if not lock_80n:
                table['ts_80n'].append((None,trial))
                table['er_80n'].append(None)
            else:
                lock_80n = False
            if not lock_90n:
                table['ts_90n'].append(None)
                table['er_90n'].append(None)
            else:
                lock_90n = False        
            if not lock_95n:
                table['ts_95n'].append(None)
                table['er_95n'].append(None)   
            else:
                lock_95n = False
            evaluation_list_copy = evaluation_list[:]
            evaluation_list_log.append(evaluation_list_copy)
            break


        if counter == steps:
            table['er'].append(error_prob)
            print '{}/{} \r'.format(trial, trials),
            sys.stdout.flush()
            #print("{}/{} \r".format(trial, trials), end="\r")
            if not lock_80:
                table['ts_80'].append(None)
                table['er_80'].append(None)
            else:
                lock_80 = False
            if not lock_90:
                table['ts_90'].append(None)
                table['er_90'].append(None)
            else:
                lock_90 = False        
            if not lock_95:
                table['ts_95'].append(None)
                table['er_95'].append(None)   
            else:
                lock_95 = False
            if not lock_80n:
                table['ts_80n'].append((None,trial))
                table['er_80n'].append(None)
            else:
                lock_80n = False
            if not lock_90n:
                table['ts_90n'].append(None)
                table['er_90n'].append(None)
            else:
                lock_90n = False        
            if not lock_95n:
                table['ts_95n'].append(None)
                table['er_95n'].append(None)   
            else:
                lock_95n = False

        evaluation_list_copy = evaluation_list[:]
        evaluation_list_log.append(evaluation_list_copy)

    x_state_logging = {'Previous_x_State': prev_x_state_log, 'Action': action_x_log, 
                'New_x_State': x_state_log, 'User_Response': response_x_log, 'Error': error_x_log}

    y_state_logging = {'Previous_y_State': prev_y_state_log, 'Action': action_y_log, 
                'New_y_State': y_state_log, 'User_Response': response_y_log, 'Error': error_y_log}

    x_denoise_logging = {'Maxflow': [i[0] for i in x_denoise_log],'Denoised_x_Image': [i[1] for i in x_denoise_log],
    'Final_Denoised_x_Image': [i[2] for i in x_denoise_log],'x_Image': [i[3] for i in x_denoise_log]}

    y_denoise_logging = {'Maxflow': [i[0] for i in y_denoise_log],'Denoised_y_Image': [i[1] for i in y_denoise_log],
    'Final_Denoised_y_Image': [i[2] for i in y_denoise_log],'y_Image': [i[3] for i in y_denoise_log]}

    x_merged_log = x_state_logging.copy()
    x_merged_log.update(x_denoise_logging)

    y_merged_log = y_state_logging.copy()
    y_merged_log.update(y_denoise_logging)


    return (x_state_logging, x_denoise_logging, y_state_logging, y_denoise_logging, response_log, action_log, x_state_logger[:-1],y_state_logger[:-1], time_step_list, evaluation_list_log ,goal_value_log[:-1],error_log)

# def start_offline_simulation(current_state_list_offline,action_offline_list,prev_x_state_list_offline,prev_y_state_list_offline, response_list_offline, current_x_state_list_offline, current_y_state_list_offline, param, beta, x_size=10, y_size=10):

#     #"this function works"

#     global closed, counter
#     #rospy.init_node('gridworld', anonymous=True)
#     #start_2D_grid(initial_x_state, initial_y_state, num_x_states, num_y_states)
#     #sim = Simulation('/home/sahabi/mo/lib/python/gridsim/config.txt','/home/sahabi/mo/lib/python/gridsim/matrix.txt')
#     x_state_log = []
#     y_state_log = []
#     action_log = []
#     action_x_log = []
#     action_y_log = []
#     response_log = []
#     prev_x_state_log = []
#     prev_y_state_log = []
#     error_log = []
#     error_x_log = []
#     error_y_log = []
#     x_denoise_log = []
#     y_denoise_log = []
#     x_entry_log = []
#     y_entry_log = []
#     current_state_list = []
#     time_step_list = []
#     evaluation_list = []
#     #done = sim.update()
#     p = init_probs(x_size, y_size)
#     s = init_k(x_size, y_size)
#     current_state = (0, 0)
#     current_state_list.append(current_state)
#     response_x_log = []
#     response_y_log = []
#     target = (4,5)
#     actions = ['east','west','north','south']

#     for offline in range(0, len(action_offline_list)):
#         counter = offline+1
#         x_entry = [0,0,0,0]
#         y_entry = [0,0,0,0]
#     #    sleep(4.5)
#     #    blocks = sim.get_state()
         
#         time_step_list.append(counter-1)
#         action  =  action_offline_list[offline]
#         if action == 'stay':
#             break
#         #action = get_action([('north',15),('east',35),('south',35),('west',15)])
#         action_log.append(action)

#         if action == 'north':
#             y_entry[1] = 0
#             x_entry[1] = 0
#         elif action == 'east':
#             y_entry[1] = 0
#             x_entry[1] = 1
#         elif action == 'south':
#             y_entry[1] = 1
#             x_entry[1] = 0
#         elif action == 'west':
#             y_entry[1] = 0
#             x_entry[1] = 0

#         S_max = get_max(p)
#         S_target = get_target(current_state,S_max)
#         #print current_state
#         #sleep(0.5)

#         # for j in range(y_size):
#         #     print '\n'
#         #     print '\t',
#         #     for i in range(x_size):
#         #         print "%.3f" % p[(i,j)],
#         #         print '\t',
#         #     for i in range(x_size):
#         #         if (i,j) == current_state:
#         #             print " * ",
#         #         elif (i,j) == S_target:
#         #             print " X ",
#         #         elif (i,j) in S_max:
#         #             print " O ",
#         #         else:
#         #             print " _ ",
           
#         # print '\n'

#     #    sim.move_agent(0, action)
#         prev_x_state = prev_x_state_list_offline[offline]
#         prev_y_state = prev_y_state_list_offline[offline]
#         x_entry[0] = prev_x_state_list_offline[offline]
#         y_entry[0] = prev_y_state_list_offline[offline]

#         prev_x_state_log.append(prev_x_state)
#         prev_y_state_log.append(prev_y_state)

#         current_state = current_state_list_offline[offline]
#         current_state_list.append(current_state)
#         x_state = current_x_state_list_offline[offline]
#         y_state = current_y_state_list_offline[offline]

#         x_state_log.append(x_state)
#         y_state_log.append(y_state)

#         # print 'Step {} of {}'.format(counter,len(action_offline_list))

#     #    response = get_response()
#         response = response_list_offline[offline]
#         if action == 'west' or action == 'east':
#             x_state_log.append(x_state)
#             prev_x_state_log.append(prev_x_state)
#             response_x = response
#             x_entry[0] = prev_x_state
#             x_entry[2] = response_x
#             x_entry[3] = prev_x_state
#             response_x_log.append(response_x)
#             evaluation_list.append(response_x)
#             x_entry_log.append(x_entry)
#             x_denoise_log.append(denoise(x_entry_log,param,beta,10))
#         if action == 'south' or action == 'north':
#             y_state_log.append(y_state) 
#             prev_y_state_log.append(prev_y_state)
#             response_y = response
#             y_entry[0] = prev_y_state
#             y_entry[2] = response_y
#             y_entry[3] = prev_y_state
#             response_y_log.append(response_y)
#             evaluation_list.append(response_y)
#             y_entry_log.append(y_entry)
#             y_denoise_log.append(denoise(y_entry_log,param,beta,10))
        
        
#         #evaluation_list = response_log[:]
#         #corrected_evaluations = []
#         #print 'original {}'.format(y_denoise_log[counter-1][3]) 
#         #print 'origints {}'.format(y_denoise_log[counter-1][4])
#         #print 'denoised {}'.format(y_denoise_log[counter-1][2])
#         #print 'eval list pre: {}'.format(evaluation_list)
#         #print y_denoise_log
#         #print x_denoise_log

#         for action in action_log:
#             if action == 'east' or action == 'west':
#                 for index,column in enumerate(x_denoise_log[-1][4]):
#                     for x,element in enumerate(column):
#                         if type(element) == list:
#                             if element[0] != -1:
#                                 if element[0] != x_denoise_log[-1][2][index][x] and (action_log[element[1]] == 'east' or action_log[element[1]] == 'west'):
#                                     if evaluation_list[element[1]] == 1:
#                                         evaluation_list[element[1]] = 0
#                                     if evaluation_list[element[1]] == 0:
#                                         evaluation_list[element[1]] = 1                

#         for action in action_log:
#             if action == 'south' or action == 'north':
#                 for index,column in enumerate(y_denoise_log[-1][4]):
#                     for x,element in enumerate(column):
#                         if type(element) == list:
#                             if element[0] != -1:
#                                 if element[0] != y_denoise_log[-1][2][index][x] and (action_log[element[1]] == 'north' or action_log[element[1]] == 'south'):
#                                     if evaluation_list[element[1]] == 1:
#                                         evaluation_list[element[1]] = 0
#                                     if evaluation_list[element[1]] == 0:
#                                         evaluation_list[element[1]] = 1   
        
#         #print 'eval list post: {}'.format(evaluation_list)
        
#         #K = det_k(P,current_state,action,response)
#         s = init_k(x_size, y_size)
#         all_K = det_all_k(s,current_state_list[:-1],action_log,evaluation_list)
        
#         p = init_probs(x_size, y_size)
#         p = update_all_P(p,all_K)

#         #print p
#         #P = update_P(P,K)
#         #print manhattan_dist(current_state_list[-1],target)
#         if action == 'south' or action == 'north':
#             action_y_log.append(action)

#             if manhattan_dist(current_state_list[-2],target) < manhattan_dist(current_state_list[-1],target) and response == 1:
#                 error_y_log.append(True)
#             elif manhattan_dist(current_state_list[-2],target) >= manhattan_dist(current_state_list[-1],target) and response == 0:            
#                 error_y_log.append(True)
#             else:
#                 error_y_log.append(False)
#         elif action == 'east' or action == 'west':
#             action_x_log.append(action)
#             if manhattan_dist(current_state_list[-2],target) < manhattan_dist(current_state_list[-1],target) and response == 1:
#                 error_x_log.append(True)
#             elif manhattan_dist(current_state_list[-2],target) >= manhattan_dist(current_state_list[-1],target) and response == 0:            
#                 error_x_log.append(True)
#             else:
#                 error_x_log.append(False)

#     x_state_logging = {'Previous_x_State': prev_x_state_log, 'Action': action_x_log, 
#                 'New_x_State': x_state_log, 'User_Response': response_x_log, 'Error': error_x_log}

#     y_state_logging = {'Previous_y_State': prev_y_state_log, 'Action': action_y_log, 
#                 'New_y_State': y_state_log, 'User_Response': response_y_log, 'Error': error_y_log}

#     x_denoise_logging = {'Maxflow': [i[0] for i in x_denoise_log],'Denoised_x_Image': [i[1] for i in x_denoise_log],
#     'Final_Denoised_x_Image': [i[2] for i in x_denoise_log],'x_Image': [i[3] for i in x_denoise_log]}

#     y_denoise_logging = {'Maxflow': [i[0] for i in y_denoise_log],'Denoised_y_Image': [i[1] for i in y_denoise_log],
#     'Final_Denoised_y_Image': [i[2] for i in y_denoise_log],'y_Image': [i[3] for i in y_denoise_log]}

#     x_merged_log = x_state_logging.copy()
#     x_merged_log.update(x_denoise_logging)

#     y_merged_log = y_state_logging.copy()
#     y_merged_log.update(y_denoise_logging)

#     return (x_state_logging, x_denoise_logging, y_state_logging, y_denoise_logging, response_log, evaluation_list)

if __name__=="__main__":

    counter = 0
    responded = False
    subscribed = False
    closed = False
    subscription = 0

    # param_list = [.1,.15,.20,.25,.30]
    # beta_list = [.5,.6,.7,.8,.9,1]

    # x_log9 = pd.read_pickle('/home/sahabi/log_x2_5x5_err_0.25_beta_0.5.p')
    # y_log9 = pd.read_pickle('/home/sahabi/log_y2_5x5_err_0.25_beta_0.5.p')
    # action_offline_list = []
    # prev_x_state_list_offline = []
    # prev_y_state_list_offline = []
    # current_state_list_offline = []
    # response_list_offline = []
    # current_x_state_list_offline = []
    # current_y_state_list_offline = []
    # #param = param
    # #beta = beta

    # for row in range(0,len(y_log9)):
    #     j = y_log9.loc[row, 'Previous_y_State']
    #     i = x_log9.loc[row, 'Previous_x_State']
    #     j_c = y_log9.loc[row, 'New_y_State']
    #     i_c = x_log9.loc[row, 'New_x_State']
    #     response = y_log9.loc[row, 'User_Response']
    #     action = x_log9.loc[row, 'Action']
    #     prev_x_state_list_offline.append(i)
    #     prev_y_state_list_offline.append(j)
    #     current_x_state_list_offline.append(i_c)
    #     current_y_state_list_offline.append(j_c)
    #     action_offline_list.append(action) 
    #     response_list_offline.append(response)
    #     current_state_list_offline.append((i,j))


    # for param in param_list:
    #     for beta in beta_list:

    #         log = start_offline_simulation(current_state_list_offline,action_offline_list,prev_x_state_list_offline,prev_y_state_list_offline, response_list_offline, current_x_state_list_offline, current_y_state_list_offline, param, beta)
    #         log_x_state_df = pd.DataFrame()
    #         #print log[0]
    #         log_x_state_df = log_x_state_df.from_dict(log[0], orient='columns', dtype=None)

    #         log_x_denoise_df = pd.DataFrame()
    #         log_x_denoise_df = log_x_denoise_df.from_dict(log[1], orient='columns', dtype=None)

    #         log_y_state_df = pd.DataFrame()
    #         log_y_state_df = log_y_state_df.from_dict(log[2], orient='columns', dtype=None)

    #         log_y_denoise_df = pd.DataFrame()
    #         log_y_denoise_df = log_y_denoise_df.from_dict(log[3], orient='columns', dtype=None)

    # #log_y_df = pd.DataFrame()
    # #log_y_df = log_y_df.from_dict(log[1], orient='columns', dtype=None)

    # log_x_state_df.to_pickle('log_x5_5x5_err_{}_beta_{}_state.p'.format(param,beta))
    # log_x_denoise_df.to_pickle('log_x5_5x5_err_{}_beta_{}_denoise.p'.format(param,beta))

    # log_y_state_df.to_pickle('log_y5_5x5_err_{}_beta_{}_state.p'.format(param,beta))
    # log_y_denoise_df.to_pickle('log_y5_5x5_err_{}_beta_{}_denoise.p'.format(param,beta))


    #############
    param = .3
    beta = .8
    exp = 6

    steps = int(sys.argv[1])
    trials = int(sys.argv[2])
    name = sys.argv[3]
    lock_80 = False
    lock_90 = False
    lock_95 = False
    lock_80n = False
    lock_90n = False
    lock_95n = False
    trial = 0
    table = {'ts_80': [], 'ts_90': [], 'ts_95': [],'er_80': [], 'er_90': [], 'er_95': [],\
    'ts_80n': [], 'ts_90n': [], 'ts_95n': [],'er_80n': [], 'er_90n': [], 'er_95n': [], 'er': []}
    while trial != trials:
        log = start_simulation(5, 5, steps, param, beta, correction=True)
        trial += 1
    #print log[0]
    print table
    df = pd.DataFrame(data = table)
    #print df
    df.to_csv(sep = ',', path_or_buf='/home/sahabi/hasan/denoise/data/{}.csv'.format(name))
    log_x_state_df = pd.DataFrame()
    log_x_state_df = log_x_state_df.from_dict(log[0], orient='columns', dtype=None)

    log_x_denoise_df = pd.DataFrame()
    log_x_denoise_df = log_x_denoise_df.from_dict(log[1], orient='columns', dtype=None)

    log_y_state_df = pd.DataFrame()
    log_y_state_df = log_y_state_df.from_dict(log[2], orient='columns', dtype=None)

    log_y_denoise_df = pd.DataFrame()
    log_y_denoise_df = log_y_denoise_df.from_dict(log[3], orient='columns', dtype=None)
    logging_exp = {'Evaluation': log[4], 'Action': log[5], 
                'x_State': log[6],'y_State': log[7], 'Time_Step': log[8], 'Goal_value': log[-2], 'Error': log[-1],  'Corrected_evaluation': log[-3]}
    
    logging_exp_df = pd.DataFrame()
    logging_exp_df = logging_exp_df.from_dict(logging_exp, orient='columns', dtype=None)

    log_x_state_df.to_csv('log_x{}_5x5_err_{}_beta_{}_state.csv'.format(exp,param,beta))
    log_x_denoise_df.to_csv('log_x{}_5x5_err_{}_beta_{}_denoise.csv'.format(exp,param,beta))

    log_y_state_df.to_csv('log_y{}_5x5_err_{}_beta_{}_state.csv'.format(exp,param,beta))
    log_y_denoise_df.to_csv('log_y{}_5x5_err_{}_beta_{}_denoise.csv'.format(exp,param,beta))

    logging_exp_df.to_csv('log_exp{}_5x5.csv'.format(exp))
