import matplotlib.pyplot as plt
import numpy as np
import random
import math

def plot_vehicle_trajectory(speeds, angles, timestamp,x_point,y_point): 
    x = 0.0
    y = 0.0
    heading = 0.0
    #dt = 0.1  # 模擬時間間隔

    x_values = [x]
    y_values = [y]

    for speed, angle, dt in zip(speeds, angles, timestamp):
        distance = speed * dt
        x += distance * np.cos(heading)
        y += distance * np.sin(heading)
        heading += np.radians(angle)

        x_values.append(x)
        y_values.append(y)
    
    Overlap_rate = rate_superposition(x_values,y_values,x_point,y_point)
    print("重疊率為"+ str(Overlap_rate*100)+"%")
    plt.figure(figsize=(8, 6))
    plt.plot(x_point, y_point,marker = 'x',markersize=1)
    plt.plot(x_values, y_values, marker='o',markersize=1)
    plt.title('Vehicle Trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    
    

def rate_superposition(x1,y1,x2,y2): #計算重疊率
    THRESHOLD = 0.1
    xx1 = np.asarray(x1)
    xx2 = np.asarray(x2)
    yy1 = np.asarray(y1)
    yy2 = np.asarray(y2)
    xx = xx1 - xx2 
    yy = yy1 - yy2
    xy = xx * yy
    xy = np.absolute(xy)
    return np.sum(xy <= THRESHOLD)/len(xx1)
    

if __name__ == '__main__':
  
  ###預估路徑
  velocity = []
  angular = []
  timestamp= []
  ###預計路徑
  x = [0]
  y = [0]
  narray = np.random.rand(100,3)
  for i in range(100): 
      velocity.append(random.random())
      angular.append(random.random())
      timestamp.append(random.random())
      x.append(x[i]+random.random())
      y.append(y[i]+0.01)
  
  plot_vehicle_trajectory(velocity, angular,timestamp,x,y)
  
