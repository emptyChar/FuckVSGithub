# import sys
# import numpy as np

# #get the distance to the 4 known markers and calculates the position of the point 
# #test comment
# def trilaterate3D(r1, r2, r3, r4):
#     p1=np.array([0,0,0])    #known points in 3D, change this based on measurements of the markers
#     p2=np.array([1,0,0])
#     p3=np.array([0,1,0])       
#     p4=np.array([1,1,0])    
    
#     e_x=(p2-p1)/np.linalg.norm(p2-p1)
#     i=np.dot(e_x,(p3-p1))
#     e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
#     e_z=np.cross(e_x,e_y)
#     d=np.linalg.norm(p2-p1)
#     j=np.dot(e_y,(p3-p1))
#     x=((r1**2)-(r2**2)+(d**2))/(2*d)
#     y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
#     z1=np.sqrt(r1**2-x**2-y**2)
#     z2=np.sqrt(r1**2-x**2-y**2)*(-1)
#     ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
#     ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)

#     return ans1
#     # dist1=np.linalg.norm(p4-ans1)
#     # dist2=np.linalg.norm(p4-ans2)
#     # if np.abs(r4-dist1)<np.abs(r4-dist2):
#     #     return ans1
#     # else: 
#     #     return ans2

# if __name__ == "__main__":
#     # (5,2,1)
#     x = trilaterate3D(5.477, 4.5825, 5.1962, 4.243)
#     print(x)
 
import numpy as np
import pandas as pd
from time import time

class MLAT:
    @staticmethod
    def __d(p1, p2):
        return np.linalg.norm(p1 - p2)

    @staticmethod
    def gdescent(anchors_in, ranges_in, bounds_in=((0, 0, 0),),
                 n_trial=100, alpha=0.001, time_threshold=None):
        anchors = np.array(anchors_in, dtype=float)
        n, dim = anchors.shape
        bounds_temp = anchors
        if bounds_in is not None:
            bounds_temp = np.append(bounds_temp, bounds_in, axis=0)
        bounds = np.empty((2, dim))
        for i in range(dim):
            bounds[0, i] = np.min(bounds_temp[:, i])
            bounds[1, i] = np.max(bounds_temp[:, i])

        if time_threshold is None:
            time_threshold = 1.0 / n_trial

        ranges = np.empty(n)
        result = pd.DataFrame(columns=['estimator', 'error'],
                              index=np.arange(n_trial))
        for i in range(n_trial):
            estimator0 = np.empty(dim)
            for j in range(dim):
                estimator0[j] = np.random.uniform(bounds[0, j], bounds[1, j])
            estimator = np.copy(estimator0)

            t0 = time()
            while True:
                for j in range(n):
                    ranges[j] = MLAT.__d(anchors[j, :], estimator)
                error = MLAT.__d(ranges_in, ranges)

                delta = np.zeros(dim)
                for j in range(n):
                    delta += (ranges_in[j] - ranges[j]) / ranges[j] * \
                             (estimator - anchors[j, :])
                delta *= 2 * alpha

                estimator_next = estimator - delta
                for j in range(n):
                    ranges[j] = MLAT.__d(anchors[j, :], estimator_next)
                error_next = MLAT.__d(ranges_in, ranges)
                if error_next < error:
                    estimator = estimator_next
                else:
                    result['estimator'][i] = estimator
                    result['error'][i] = error
                    break
                if time() - t0 > time_threshold:
                    break
        return result

    @staticmethod
    def mlat(anchors_in, ranges_in, bounds_in=((0, 0, 0),),
             n_trial=100, alpha=0.001, time_threshold=None):
        ret = MLAT.gdescent(anchors_in, ranges_in, bounds_in,
                            n_trial, alpha, time_threshold)

        idx = np.nanargmin(ret['error'])
        estimator = ret['estimator'][idx]
        return estimator, ret

W = 1
L = 1
H = 0
anchors = np.array([[0, 0, H],
                    [W, 0, H],
                    [W, L, H],
                    [0, L, H]])
node = np.array([5.477, 4.5825, 5.1962])
# node = np.array([W * np.random.rand(),
#                  L * np.random.rand(),
#                  H * np.random.rand()])
ranges = np.empty(anchors.shape[0])
error = 0.5
ranges_with_error = np.empty(anchors.shape[0])
for i in range(anchors.shape[0]):
    ranges[i] = np.linalg.norm(anchors[i, :] - node)
    ranges_with_error[i] = ranges[i] + np.random.uniform(-error, error)
# TODO: You need to define search space boundary to prevent UNEXPECTED RESULT
# If not, search space boundary is defined as a cube constrained to
# minimum and maximum coordinates of x, y, z of anchors
# If anchors are in the same plane, i.e., all anchors have the same (similar)
# coordinate of at least one axes, you MUST define search space boundary
# So, defining search space boundary is all up to you
bounds = np.zeros((2, anchors.shape[0]));
for i in range(anchors.shape[1]):
    bounds[0, i] = min(anchors[:, i]); # minimum boundary of ith axis
    bounds[1, i] = max(anchors[:, i]); # maximum boundary of ith axis
# hard coded minimum height (0 m) of search boundary
bounds[0, -1] = 0;

estimator, result = MLAT.mlat(anchors, ranges_with_error)

print('Anchors')
print(anchors)
print('Node:', node)
print('Ranges:')
print('   ', ranges)
print('Ranges with error:')
print('   ', ranges_with_error)
print('Estimator')
print(estimator)
print('Full result')
print(result)
    