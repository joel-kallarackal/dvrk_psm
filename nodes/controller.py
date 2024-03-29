#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point
from pynput.keyboard import Key, Listener


class Controller:
    def __init__(self,x=-0.25,y=0,z=0.65,eps=0.001):
        # Initial location of the end effector with respect to the world frame  
        self.x = x
        self.y = y
        self.z = z
        
        self.eps = eps
        
        # Publishers
        self.endeff_next_pose_pub = rospy.Publisher("/psm/endeff_next_pos", Point, queue_size=10)
    
        with Listener(on_press = self.controller) as listener:   
            listener.join()
        
    def controller(self,key):
        '''
            Controls:
                q: increase x
                a: decrease x
                w: increase y
                s: decrease y
                e: increase z
                d: decrease z
                
        '''
        clicked = True
        
        if key.char == 'q':
            self.x+=self.eps
        elif key.char == 'a':
            self.x-=self.eps
        elif key.char == 'w':
            self.y+=self.eps
        elif key.char == 's':
            self.y-=self.eps
        elif key.char == 'e':
            self.z+=self.eps
        elif key.char == 'd':
            self.z-=self.eps
        elif key == Key.esc:
            return False
        else:
            clicked=False
        
        if clicked:
            new_pos = Point()
            new_pos.x = self.x
            new_pos.y = self.y
            new_pos.z = self.z
            self.endeff_next_pose_pub.publish(new_pos)
            
 
def main():
    rospy.init_node('controller')
    Controller()

if __name__ == "__main__":
    main()
    

        
        