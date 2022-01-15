
import rospy

import threading 
import time

no_input = True

def add_up_time():
    print("adding up time...")
    timeTaken=float(0)
    while no_input:
        time.sleep(0.01)
        timeTaken=timeTaken+0.01
    print(timeTaken)


# designed to be called as a thread
def signal_user_input():
    global no_input
    i = input("hit enter to stop things")  
    no_input = False
    # thread exits here


# we're just going to wait for user input while adding up time once...
threading.Thread(target = signal_user_input).start()

add_up_time()

print("done.... we could set no_input back to True and loop back to the previous comment...")

