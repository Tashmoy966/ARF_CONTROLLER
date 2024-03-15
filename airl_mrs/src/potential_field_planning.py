import numpy as np
import math
from numpy import linalg as la
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

OBSLEN = 5
AREA_WIDTH = 30.0  #
grid_size = 1.0  # potential grid size [m]
show_animation = True

class ARF(object):
    def __init__(self,curr_pos,goal_pos):
        
        self._ox = [15]  # obstacle x position list [m]
        self._oy = [15]  # obstacle y position list [m]
        self._curr_pos=curr_pos
        self._goal_pos=goal_pos
    def __del__(self):
        pass
    def set_obstacle_potential(self,xw, yw, ox, oy, res):
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]
        leni = int(round(OBSLEN / res))
        sidelen = int(round(leni / 2))
        for o in range(len(ox)):
            xval = int(ox[o] / res)
            yval = int(oy[o] / res)
            for l in range(-sidelen, sidelen):
                for w in range(-sidelen, sidelen):
                    pmap[w + yval][l + xval] = 10

        return gaussian_filter(pmap, sigma=1)


    def get_motion_model(self):
        # dx, dy
        motion = [[1, 0],
                [0, 1],
                [-1, 0],
                [0, -1],
                [-1, -1],
                [-1, 1],
                [1, -1],
                [1, 1]]

        return motion


    def calc_goal_potential(self,pos, goal):
        return 3 * math.exp(la.norm(np.array(pos) - np.array(goal)) / 30)


    def calc_static_potential_field(self,ox, oy, gx, gy, res):
        minx = 0
        miny = 0
        maxx = AREA_WIDTH
        maxy = AREA_WIDTH
        xw = int(round((maxx - minx) / res))
        yw = int(round((maxy - miny) / res))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        for ix in range(xw):
            x = ix * res + minx
            for iy in range(yw):
                y = iy * res + miny
                pmap[iy][ix] = self.calc_goal_potential([x, y], [gx, gy])

        pmap = np.array(pmap)
        #op = set_obstacle_potential(xw, yw, ox, oy, res)

        return pmap , minx, miny, maxx, maxy


    def get_interaction_potential(self,posA, posB):
        a = 0.2
        b = 2
        ca = 8
        cr = 1
        # print(np.array(posA).shape)
        # print(np.array(posB).shape)
        dis = la.norm(np.array(posA) - np.array(posB))   
        att_pot = -a * math.exp(-dis / ca)
        rep_pot = b * math.exp(-dis / cr)
        return att_pot + rep_pot


    def cal_dynamic_potential_field(self, id, xw, yw, res):
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]
        for i in range(yw):
            for j in range(xw):
                for r in range(len(self._curr_pos)):
                    if r == id:
                        continue
                    pmap[j][i] += 3 * self.get_interaction_potential([i, j], np.array((self._curr_pos[r] / res))) + 2    
        return pmap


    def calc_next_position(self,id, res, pmap, xvals, yvals):
        minx = xvals[0]
        maxx = xvals[1]
        miny = yvals[0]
        maxy = yvals[1]

        xw = int(round((maxx - minx) / res))
        yw = int(round((maxy - miny) / res))

        pdmap = self.cal_dynamic_potential_field(id, xw, yw, res)
        synthesized_map = np.array(pmap) + np.array(pdmap)

        ix = round((self._curr_pos[id][0] - minx) / res)
        iy = round((self._curr_pos[id][1] - miny) / res)
        # if show_animation:
        #     if id == 0:
        #         colstr = "*r"
        #     elif id == 1:
        #         colstr = "*g"
        #     elif id == 3:
        #         colstr = "*b"
        #     elif id == 4:
        #         colstr = "*y"
        #     else:
        #         colstr = "*m"
            # plt.plot(ix, iy, colstr)

        motion = self.get_motion_model()
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = synthesized_map[iny][inx]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * res + minx
        yp = iy * res + miny
        self._curr_pos[id] = [xp, yp]

        # if show_animation:
        #     plt.pause(0.0001)
        #     if id == len(curr_pos)-1:
        #         plt.savefig('data/im_'+str(it))
        return self._curr_pos


    def draw_heatmap(self,data):
        data = np.array(data)
        plt.pcolor(data, vmax=20.0, cmap=plt.cm.Blues)

    
    def potential(self):
        for r in range(len(self._curr_pos)):
            pmap, minx, miny, maxx, maxy = self.calc_static_potential_field(self._ox, self._oy, self._goal_pos[r][0], self._goal_pos[r][1], grid_size)
            xvals = [minx, maxx]
            yvals = [miny, maxy]
            self._curr_pos = self.calc_next_position(r, grid_size, pmap, xvals, yvals)
        return np.array(self._curr_pos)
   
        


# if __name__ == '__main__':
#     files = glob.glob('./data/*')
#     for f in files:
#         os.remove(f)
#     print(__file__ + " start!!")
#     main()
#     print(__file__ + " Done!!")
