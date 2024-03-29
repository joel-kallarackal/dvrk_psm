from pynput.keyboard import Key, Listener
  
# Initial location of the end effector with respect to the world frame  
x = -0.25
y = 0
z = 0.65

def controller(key):
    '''
        Write code to change x,y,z of the end effector when key pressed
    '''
    pass
 
# Collect all event until released
with Listener(on_press = controller) as listener:   
    listener.join()