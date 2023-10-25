import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
import matplotlib as plt

from .graph_search import graph_search
from flightsim.world import World

class WorldTraj(object):
    """

    """
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """
        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.61
        self.alpha = 0.99 # takes [0,1] towards self.margin
        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)
        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        self.points = np.zeros((1,3)) # shape=(n_pts,3)
        # self.remove_collinear_prox()

        self.points = self.path #####______USE WHEN ONLY USING STRAIGHTEN()______########
        self.pruner(world)


        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.vel = 8
        dist_vec = np.diff(self.points, n=1, axis=0)
        self.dist = np.linalg.norm(dist_vec, axis=1)
        self.t_i = self.dist/self.vel
        self.t_i = self.t_i.reshape(-1, 1)
        if self.t_i[-1] <= 0.05*7.5/self.vel:
            self.points = np.delete(self.points, self.points.shape[0]-2, axis=0)
            self.t_i[-2] += self.t_i[-1]
            self.t_i = np.delete(self.t_i, self.t_i.shape[0]-1, axis=0)
        # if self.t_i.shape[0] == 9:
        #     # self.points = np.delete(self.points, 1+1, axis=0)
        #     # self.t_i[1] += self.t_i[1+1]
        #     # self.t_i = np.delete(self.t_i, 1+1, axis=0)
        #     self.points = np.delete(self.points, 1 + 5, axis=0)
        #     self.t_i[5] += self.t_i[1 + 5]
        #     self.t_i = np.delete(self.t_i, 1+5, axis=0)


        # ###---Add Points Here---###
        # self.add_points(world)
        # dist_vec = np.diff(self.points, n=1, axis=0)
        # self.dist = np.linalg.norm(dist_vec, axis=1)
        # self.t_i = self.dist / self.vel
        # self.t_i = self.t_i.reshape(-1, 1)

        self.m = self.t_i.shape[0]
        ti_cp = self.t_i*1
        print("T_i BEFORE Dilation: ", self.t_i)



        #End-point velocity capping ()slowing
        corner_thresh = 0.3*6/self.vel

        dilate_factor = 2.5*self.vel/7.5
        # dilate_factor = 2.5*(7.5/self.vel)
        self.t_i[0] *= dilate_factor
        self.t_i[-1] *= dilate_factor/1.2
        # # self.t_i[-2] *= 1.1*7.5/self.vel
        self.t_i[-2] *= dilate_factor/2.2727
        # self.t_i[1] *= dilate_factor/2.2727

        # self.t_i[1] *= (dilate_factor / (1.2 * 2)) ** (1 / (2))

        # for i in range(1,3):
        #     self.t_i[-i] *= (dilate_factor/(1.2*i))**(1/i)
        #     # self.t_i[-i] *= dilate_factor / ((1.2) * (2**i))
        # self.t_i[-2] *= 1.1 * self.vel/7.5
        # no_corner_idx = self.t_i >= corner_thresh
        corner_idx = np.where(self.t_i < corner_thresh)[0]
        corner_set_diff = 0.2*8/self.vel
        # corner_set_diff = 0

        self.t_i[self.t_i < corner_thresh] = corner_thresh+corner_set_diff
        # self.t_i[self.t_i < corner_thresh] *= (1 + (0.5/self.vel))


        # print("ti_cp: ", ti_cp)

        print("T_i WHILE Dilation1: ", self.t_i)

        #########______Slowing at the adjacent points to corner______##############
        nc_set = 0.05 * 6 / self.vel
        # fac = 1*(7.5/self.vel)**2
        fac = 1
        # visited = [self.t_i.shape[0]-2]
        visited = []
        visited = np.array(visited)
        no_corner_idx = np.union1d(corner_idx+1, corner_idx-1)
        for i, id in enumerate(corner_idx):
            if id > 0 and id < self.t_i.shape[0]-1:
                if np.isin(visited, id+1).any() == False:
                    # self.t_i[id+1] += corner_set_diff * self.t_i[id+1] / self.t_i[id]
                    # print(self.t_i[id] - ti_cp[id])
                    # self.t_i[id + 1] += (self.t_i[id] - ti_cp[id]) * self.t_i[id + 1] / ti_cp[id]
                    # print(id+1)
                    self.t_i[id + 1] += fac*(self.t_i[id] - ti_cp[id]) * self.t_i[id + 1] / (self.t_i[id])
                    # self.t_i[id+1] *= (1 + (0.5 / self.vel))
                    visited = np.append(visited, id+1)
                if np.isin(visited, id-1).any() == False:
                    # print(1)
                    # self.t_i[id-1] += corner_set_diff * self.t_i[id-1] / self.t_i[id]
                    # self.t_i[id - 1] += (self.t_i[id] - ti_cp[id]) * self.t_i[id - 1] / ti_cp[id]
                    self.t_i[id - 1] += fac*(self.t_i[id] - ti_cp[id]) * self.t_i[id - 1] / (self.t_i[id])
                    # self.t_i[id - 1] *= (1 + (0.5 / self.vel))
                    visited = np.append(visited, id-1)

        # self.t_i *= 7.5 / self.vel



        # post_corner_check = np.where(self.t_i[corner_idx+1].T - nc_set - self.t_i[corner_idx].T < 0)[0]
        # pre_corner_check = np.where(self.t_i[corner_idx-1].T - nc_set - self.t_i[corner_idx].T < 0)[0]
        # print("post_corner: ", post_corner_check)
        # print("pre_corner: ", pre_corner_check)
        # post_check_corner = corner_idx[post_corner_check]
        # print("post_corner: ", post_check_corner)
        # pre_check_corner = corner_idx[pre_corner_check]
        # self.t_i[post_check_corner+1] += self.t_i[post_check_corner] - self.t_i[post_check_corner+1] + nc_set
        # self.t_i[pre_check_corner-1] += self.t_i[pre_check_corner] - self.t_i[pre_check_corner-1] + nc_set

        # self.t_i[no_corner_idx] += corner_set_diff
        print("T_i WHILE Dilation2: ", self.t_i)


        # dilate_factor = 3
        # self.t_i[0] *= dilate_factor
        # self.t_i[-1] *= dilate_factor/2
        # self.t_i[self.t_i < 0.3] = 0.32

        # dilate_factor = 3.
        # self.t_i[0] *= dilate_factor*3/2
        # self.t_i[-1] *= dilate_factor*3/8
        # self.t_i[self.t_i < 0.3] = 0.32
        # print("T_i AFTER Dilation: ", self.t_i)

        #Slowing around the corners


        #Faster for longer durations (nonlinear time scaling)
        # self.t_i *= 1.1*np.sqrt(1.3/self.t_i)
        # self.t_i *= (7.5/self.vel) * np.sqrt(1.65/self.t_i)
        # self.t_i *= (1.65 / self.t_i)**(1/3)
        self.t_i *= np.sqrt(1.65 / self.t_i)
        self.t_i[self.t_i < 0.3] = 0.3
        print("T_i AFTER Dilation: ", self.t_i)
        print("t_i_shape: ", self.t_i.shape[0])

        self.t_start = np.cumsum(np.vstack((np.zeros((1, 1)), self.t_i)), axis=0)

        #Solve AC=P
        self.C = self.min_snap()

    def add_points(self, world):
        points_edit = self.points*1
        robot_radius = 0.25
        if self.margin > robot_radius:
            robot_radius = (self.alpha * self.margin) + (1 - self.alpha) * robot_radius
        dist_vec = np.diff(self.points, n=1, axis=0)
        distances = np.linalg.norm(dist_vec, axis=1)
        mean_dist = np.mean(distances)
        for i in range(distances.shape[0]):
            if distances[i] >= mean_dist:
                check = True
                k = 1
                while check:
                    seg = np.linspace(self.points[i], self.points[i+1], num=2+k, endpoint=True,
                                                retstep=False, axis=0)
                    for j in range(seg.shape[0]-1):
                        num = np.ceil(np.max(np.abs(seg[j] - seg[j+1])) / np.min(self.resolution))
                        print("num: ", num)
                        nump = np.linspace(seg[j], seg[j+1], num=num.astype(int), endpoint=True,
                                                    retstep=False, axis=0)

                        if nump.shape[0] <= 1:
                            check = False
                            continue
                        # collision_pts = world.path_collisions(seg, self.margin)
                        collision_pts = world.path_collisions(nump, robot_radius)
                        if collision_pts.size != 0:
                            k += 1
                            check = True
                            break
                        else:
                            check = False
                            continue

                    if check == False:
                        points_edit = np.insert(points_edit, i+1, seg[1:-1], axis=0)
                        break
        self.points = points_edit






    def cont_var_matrix(self, t):

        t = t ** np.arange(0, 8)
        r0 = np.array([0, 0, 0, 0, 0, 0, 0, 1])
        r1 = np.array([t[7], t[6], t[5], t[4], t[3], t[2], t[1], 1])
        r2 = np.array([7*t[6], 6*t[5], 5*t[4], 4*t[3], 3*t[2], 2*t[1], 1, 0])
        r3 = np.array([42*t[5], 30*t[4], 20*t[3], 12*t[2], 6*t[1], 2, 0, 0])
        r4 = np.array([210*t[4], 120*t[3], 60*t[2], 24*t[1], 6, 0, 0, 0])
        r5 = np.array([840*t[3], 360*t[2], 120*t[1], 24, 0, 0, 0, 0])
        r6 = np.array([2520*t[2], 720*t[1], 120, 0, 0, 0, 0, 0])
        r7 = np.array([5040*t[1], 720, 0, 0, 0, 0, 0, 0])

        return np.vstack((r0, r1, r2, r3, r4, r5, r6, r7))


    def min_snap(self):

        # A matrix and P matrix:
        A = np.zeros((3, 8*self.m))
        A[[0,1,2], [6,5,4]] = [1, 2, 6]
        P = np.zeros((8 * self.m, 3))
        cont_const = np.zeros((8, 8))
        cont_const[[2, 3, 4, 5, 6, 7], [6, 5, 4, 3, 2, 1]] = [-1, -2, -6, -24, -120, -720]
        for i, t in enumerate(self.t_i):
            cont_var = self.cont_var_matrix(t)
            row_t = np.zeros((8, 8*self.m))
            id = 8*i
            P[3 + id] = self.points[i]
            P[4 + id] = self.points[i + 1]
            if i == self.t_i.shape[0]-1:
                row_t = np.zeros((5, 8*self.m))
                row_t[:, id:id + 8] = cont_var[0:5, :]
                A = np.vstack((A, row_t))
            else:
                row_t[:, id:id+8] = cont_var
                row_t[:, id+8:id+16] = cont_const
                A = np.vstack((A, row_t))
        sA = sparse.csc_matrix(A)
        sP = sparse.csc_matrix(P)
        C = spsolve(sA, sP)
        return C.toarray()


    def remove_collinear_prox(self):
        # path_cp = self.path
        # # path_cp1 = self.path
        # temp = np.delete(path_cp, -1, axis=0)
        # temp1 = np.delete(path_cp, 0, axis=0)
        #
        # diff = temp1 - temp
        # diff1 = np.delete(diff, 0, axis=0)
        # diff = np.delete(diff, -1, axis=0)
        #
        # cross = np.cross(diff1, diff)
        # cross = np.linalg.norm(cross, axis=1)
        #
        # index = []
        # index_1 = [0]
        # for i, value in enumerate(cross):
        #     if value == 0 and cross[i + 1] == 0:
        #         # if value == 0:
        #         index.append(i + 1)
        #     if value != 0:
        #         index_1.append(i + 1)
        # index_1.append(self.path.shape[0] - 1)
        #
        # # self.path = np.delete(self.path, index, axis=0)
        # self.path = self.path[index_1]
        # path_n = self.path
        # # dist_norm = np.linalg.norm(self.path[1:] - self.path[0:-1], axis=1)
        # path_n_lastp = path_n[-1]
        # while i < path_n.shape[0]-1:
        #     dist_norm = np.linalg.norm(path_n[i+1] - path_n[i])
        #     if dist_norm <= np.linalg.norm(self.resolution):
        #         path_n = np.delete(path_n, i+1, 0)
        #         i -= 1
        #     i += 1
        # if path_n[-1][0] != path_n_lastp[0] and path_n[-1][1] != path_n_lastp[1] and path_n[-1][2] != path_n_lastp[2]:
        #     path_n = np.append(path_n, path_n_lastp.reshape(1,3), axis=0)
        # self.points = path_n
        # print(self.points.shape[0])

        path_cp = self.path
        path_cp_lastp = path_cp[-1]
        i = 0
        while i < path_cp.shape[0]-2:
            cross_norm = np.linalg.norm(np.cross(path_cp[i+2]-path_cp[i+1], path_cp[i+1]-path_cp[i]))
            dist_norm = np.linalg.norm(path_cp[i + 1] - path_cp[i])
            if cross_norm == 0:
                path_cp = np.delete(path_cp, i+1, 0)
                i -= 1
            # elif dist_norm > 0.01:
            elif dist_norm <= np.linalg.norm(self.resolution):
                print("DELETED")
                path_cp = np.delete(path_cp, i, 0)
                # i -= 1
            i += 1
            if path_cp[-1][0] != path_cp_lastp[0] or path_cp[-1][1] != path_cp_lastp[1] or path_cp[-1][2] != \
                    path_cp_lastp[2]:
                path_cp = np.append(path_cp, path_cp_lastp.reshape(1, 3), axis=0)
        self.points = path_cp
        print("before proximity check: ", self.points.shape[0])

        dist_norms = np.linalg.norm(path_cp[1:] - path_cp[:-1], axis=1)
        keep_idx = np.where(dist_norms > np.linalg.norm(self.resolution))[0]
        if keep_idx[0] == 0:
            keep_idx = np.append(keep_idx, path_cp.shape[0] - 1)
        else:
            keep_idx = np.append(np.zeros(1), keep_idx)
            keep_idx = np.append(keep_idx, path_cp.shape[0] - 1)

        prox_num = (keep_idx[1:]-keep_idx[:-1])
        prox_idx = np.where((keep_idx[1:]-keep_idx[:-1]) > 3)[0]
        prox_center = np.ceil(prox_num[prox_idx]/2)
        prox_center += keep_idx[prox_idx]
        keep_idx = np.sort(np.append(keep_idx, prox_center))
        path_cp = path_cp[keep_idx.astype(int)]
        self.points = path_cp
        print("after proximity check: ", self.points.shape[0])





##########________WORKING________################
    def pruner(self, world):
        robot_radius = 0.25
        if self.margin > robot_radius:
            robot_radius = (self.alpha*self.margin) + (1-self.alpha)*robot_radius
        points_edit = self.points*1
        del_idx = []
        del_idx = np.array(del_idx)
        i = 0
        while i < self.points.shape[0]-2:
            for j in range(i+1, self.points.shape[0]):
                num = np.ceil(np.max(np.abs(self.points[j] - self.points[i]))/np.min(self.resolution))
                print("num: ", num)
                seg, ret_step = np.linspace(self.points[i], self.points[j], num=num.astype(int), endpoint=True, retstep=True, axis=0)
                # seg, ret_step = np.linspace(self.points[i], self.points[j], num=num.astype(int), endpoint=False, retstep=True)
                # seg = np.append(seg, self.points[j].reshape(1,3), axis=0)

                # print("Segment_shape: ", seg)
                # print(self.points[i], "\n", self.points[j])
                if seg.shape[0] <= 1:
                    continue
                # collision_pts = world.path_collisions(seg, self.margin)
                collision_pts = world.path_collisions(seg, robot_radius)
                if collision_pts.size == 0:
                    print("i,j: ", i,j)
                    # print("NO COLLISION")
                    if j == self.points.shape[0]-1:
                        # if i+1 != j-1:
                        if j-i >= 2:
                            del_idx = np.append(del_idx, np.arange(i + 1, j - 1))
                            del_idx = np.append(del_idx, j - 1)
                        i = j
                        break
                        # else:
                        #     del_idx = np.append(del_idx, i + 1)
                        #     i = j
                        #     break
                    continue
                if collision_pts.size != 0 or j == self.points.shape[0]-1:
                    print("COLLISION i,j: ", i,j)
                    if j-i <= 1:
                        if j == self.points.shape[0]-1:
                            i = j
                            break
                        # i = j
                        continue
                    if i+1 == j-1:
                        # del_idx = np.append(del_idx, i+1)
                        if np.linalg.norm(self.points[i]-self.points[j]) <= 1:
                            # i = j
                            i += 1
                            break
                        # else:
                        #     del_idx = np.append(del_idx, i + 1)
                        #     i = j
                        #     break
                        # points_edit = np.delete(points_edit, i+1, axis=0)
                    del_idx = np.append(del_idx, np.arange(i+1, j-1))
                    # points_edit = np.delete(points_edit, np.arange(i+1, j-1), axis=0)
                    if j >= self.points.shape[0]-1:
                        i = j
                        break
                    i = j-1
                    break
        # print("#######NORM#########\n", np.linalg.norm(self.points[6]-self.points[8]))
        self.points = np.delete(self.points, del_idx.astype(int), axis=0)

    #############------------PREVIOUS------------##############
    # def straighten(self, world):
    #     robot_radius = 0.25
    #     points_edit = self.points
    #     del_idx = []
    #     del_idx = np.array(del_idx)
    #     i = 0
    #     while i < self.points.shape[0] - 2:
    #         for j in range(i + 1, self.points.shape[0]):
    #             num = np.ceil(np.max(np.abs(self.points[j] - self.points[i])) / np.min(self.resolution))
    #             print("num: ", num)
    #             seg, ret_step = np.linspace(self.points[i], self.points[j], num=num.astype(int), endpoint=True,
    #                                         retstep=True, axis=0)
    #             # seg, ret_step = np.linspace(self.points[i], self.points[j], num=num.astype(int), endpoint=False, retstep=True)
    #             # seg = np.append(seg, self.points[j].reshape(1,3), axis=0)
    #
    #             # print("Segment_shape: ", seg)
    #             # print(self.points[i], "\n", self.points[j])
    #             if seg.shape[0] <= 1:
    #                 continue
    #             collision_pts = world.path_collisions(seg, self.margin)
    #             if collision_pts.size == 0:
    #                 print("i,j: ", i, j)
    #                 print("NO COLLISION")
    #                 # if j == self.points.shape[0] - 1:
    #                 #     i = j
    #                 continue
    #             if collision_pts.size != 0 or j == self.points.shape[0] - 1:
    #                 print("i,j: ", i, j)
    #                 if j - i <= 1:
    #                     # i = j
    #                     continue
    #                 if i + 1 == j - 1:
    #                     # del_idx = np.append(del_idx, i+1)
    #                     if np.linalg.norm(self.points[i] - self.points[j]) <= 1:
    #                         # i = j
    #                         i += 1
    #                         break
    #                     # points_edit = np.delete(points_edit, i+1, axis=0)
    #                 del_idx = np.append(del_idx, np.arange(i + 1, j - 1))
    #                 # points_edit = np.delete(points_edit, np.arange(i+1, j-1), axis=0)
    #                 i = j - 1
    #                 break
    #     # print("#######NORM#########\n", np.linalg.norm(self.points[6]-self.points[8]))
    #     self.points = np.delete(self.points, del_idx.astype(int), axis=0)

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        # print(vel)
        # for idx, time in enumerate(self.t_start):
        #     if t >= self.t_start[-1]:
        #         x = self.points[-1]
        #         x_dot = np.zeros((3,))
        #     elif time < t < self.t_start[idx + 1]:
        #         x = self.points[idx] + self.vel[idx] * (t - time)
        #         x_dot = self.vel[idx]
        #     elif t == time:
        #         x = self.points[idx]
        #         x_dot = np.zeros((3,))
        #     else:
        #         pass
        # # print(x)
        # # print(x,x_dot)

        if t <= 0:
            x = self.points[0]
        # elif t > self.t_start[-1]:
        elif t >= self.t_start[-1]:
            x = self.points[-1]
        else:
            seg_id = np.where((t-self.t_start) > 0)[0][-1]
            C_seg = self.C[8*seg_id:(8*seg_id)+8]
            t_seg = t - self.t_start[seg_id]
            T_mat = self.cont_var_matrix(t_seg)[1:6, :]

            states = T_mat @ C_seg
            x = states[0]
            x_dot = states[1]
            x_ddot = states[2]
            x_dddot = states[3]
            x_ddddot = states[4]


        # STUDENT CODE END
        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
