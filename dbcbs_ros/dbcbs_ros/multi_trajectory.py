#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    trajs = []
    n = 1  # number of distinct trajectories

    # enable logging
    allcfs.setParam('usd.logging', 1)
    ids = [0]
    for i in range(n):
        traj = Trajectory()
        # traj.loadcsv(Path(__file__).parent / f'data/multi_trajectory/traj{i}.csv')
        traj.loadcsv(Path(__file__).parent / f'data/drone.csv')
        trajs.append(traj)

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for idx, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajs[idx % len(trajs)])

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(3.0)

        # go to initial state
        for idx, cf in enumerate(allcfs.crazyflies):
            traj = trajs[idx % len(trajs)]
            initial_pos = traj.eval(0.0).pos
            cf.goTo(initial_pos, 0, 2.0)
        timeHelper.sleep(2.5)
        
        allcfs.startTrajectory(0, timescale=TIMESCALE, relative=False)
        timeHelper.sleep(max([t.duration for t in trajs]) * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

    # disable logging
    allcfs.setParam('usd.logging', 0)


if __name__ == '__main__':
    main()
