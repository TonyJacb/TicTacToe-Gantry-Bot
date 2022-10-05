#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool
from gcodeclient import Client
import random

class Motor:
    def __init__(self) -> None:
        '''
        Initialise the GCode Client, and the subscriber
        '''
        self.client = Client("/dev/ttyACM0", 115200)
        rospy.loginfo_once("Started Motor Node")
        rospy.wait_for_service('bots_move')
        rospy.Subscriber("/positions", Int16MultiArray, callback=self.play)
        self.result_init = (-1,-1,-1,-1,-1,-1,-1,-1,-1)
        self.score = [-1,-1,-1,-1,-1,-1,-1,-1,-1]
        # print(f'Init : {self.result_init}')
        
        

    def play(self, data):
        '''
        Decides the move to be made based on the positions of the zeros in the play area.
        '''
        result_next = data.data
        print(f'Zeroes: {result_next}')
        if result_next != self.result_init:
            # print(True)
            # print(result_next)
            move = random.randint(0,8)
            indexes = [i for i in range(len(result_next)) if result_next[i] == 0]
            if indexes != []:
                if move not in indexes and self.score[move] != 1:
                    self.score[move] = 1
                    for index in indexes:
                        self.score[index] = 0
                    set_led_state = rospy.ServiceProxy('bots_move', SetBool)
                    set_led_state(False)
                    print(f'Move:{move}')
                    
                    
                    score = f'{str(self.score[6])} | {str(self.score[7])} | {str(self.score[8])} \n =================\n  {str(self.score[3])} | {str(self.score[4])} | {str(self.score[5])} \n ================\n"  {str(self.score[0])} | {str(self.score[1])} | {str(self.score[2])} \n'
                    print(f'ScoreBoard:\n {score}')
                    self.check_winner()
                    self.client.automate_mode(str(move))
                    
                    self.result_init = result_next
        
    def check_winner(self):
        '''
        Checks who won the game.
        '''
        if self.score[0] == self.score[1] == self.score[2]:
            if self.score[0] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
                # rospy.signal_shutdown("Game Over")

            elif self.score[0] == 0:
                print('You have won against a puny bot.')
                self.client.flush()
                # rospy.signal_shutdown("Game Over")

        elif self.score[0] == self.score[4] == self.score[8]:
            if self.score[0] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[0] == 0:
                print('You have won against a puny bot.')
                self.client.flush()  

        elif self.score[0] == self.score[3] == self.score[6]:
            if self.score[0] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[0] == 0:
                print('You have won against a puny bot.')
                self.client.flush()

        elif self.score[1] == self.score[4] == self.score[7]:
            if self.score[1] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[1] == 0:
                print('You have won against a puny bot.')
                self.client.flush()        

        elif self.score[2] == self.score[4] == self.score[6]:
            if self.score[2] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[2] == 0:
                print('You have won against a puny bot.')
                self.client.flush()
        
        elif self.score[2] == self.score[5] == self.score[8]:
            if self.score[2] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[2] == 0:
                print('You have won against a puny bot.')
                self.client.flush()

        elif self.score[3] == self.score[4] == self.score[5]:
            if self.score[3] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[3] == 0:
                print('You have won against a puny bot.')
                self.client.flush()

        elif self.score[6] == self.score[7] == self.score[8]:
            if self.score[6] == 1:
                print('You have been defeated by a Bot')
                self.client.flush()
            elif self.score[6] == 0:
                print('You have won against a puny bot.')
                self.client.flush()


if __name__ == "__main__":
    rospy.init_node("motor_node")
    Motor()
    rospy.spin()
    