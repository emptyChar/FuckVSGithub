import sys
import numpy as np

#get the distance to the 4 known markers and calculates the position of the point 
#test comment
def trilaterate3D(r1, r2, r3, r4):
    p1=np.array([0,0,0])    #known points in 3D, change this based on measurements of the markers
    p2=np.array([1,0,0])
    p3=np.array([0,1,0])       
    p4=np.array([1,1,0])    
    
    e_x=(p2-p1)/np.linalg.norm(p2-p1)
    i=np.dot(e_x,(p3-p1))
    e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    e_z=np.cross(e_x,e_y)
    d=np.linalg.norm(p2-p1)
    j=np.dot(e_y,(p3-p1))
    x=((r1**2)-(r2**2)+(d**2))/(2*d)
    y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    z1=np.sqrt(r1**2-x**2-y**2)
    z2=np.sqrt(r1**2-x**2-y**2)*(-1)
    ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
    ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)

    return ans1
    # dist1=np.linalg.norm(p4-ans1)
    # dist2=np.linalg.norm(p4-ans2)
    # if np.abs(r4-dist1)<np.abs(r4-dist2):
    #     return ans1
    # else: 
    #     return ans2

if __name__ == "__main__":
    x = trilaterate3D(5.477, 4.5825, 5.1962, 4.243)
    print(x)
    

    