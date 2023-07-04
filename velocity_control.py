def velocity(dis):
    kp = 1
    safe_dis = 2 
    vel = kp * (dis-safe_dis)
    if vel < 0 : return 0
    else : return vel  

def angle_rotation(angle): #80度 切成8等分
    ang = angle//10
    if ang > 4 :
        return 40*3.14159/180
    elif ang < -4:
        return -40*3.14159/180 
    else: return ang*10*3.14159/180
def twist(vel,ang):        
    return {"linear":     
            {"x": vel,   
            "y": 0.0,      
            "z": 0.0,},    
            "angular":     
            {"x": 0.0,     
            "y": 0.0,      
            "z": ang},     
            }    
''' ROS topic
car_motion = Twist()
car_motion.linear = vel
car_motion.angular =ang
return car_motion
'''          
def move(signal):
    print(signal)
    pass

x = -1
r =  30
move(twist(velocity(x), angle_rotation(r)))