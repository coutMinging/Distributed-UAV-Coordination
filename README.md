# Distributed-UAV-Coordination
We use CORE(Common Open Research Emulator) to emulate the UAV network

## Overview
In this project, we explore distributed UAV coordination by developing a robust communication system using the User Datagram Protocol (UDP). Our aim is to implement an agreement protocol that enables each UAV node to dynamically and uniquely lock onto a target once it enters their operating range.

## Project Details
Sample Mission Scenario:
- Each UAV can track only one target at a time
- Once tracking a target, a UAV “locks” on it
- Does not switch targets unless the current one gets out of range Simple Algorithm:
  - Find a target
  - If the target is already tracked, then search for another one
  - If the target is not tracked, then advertise to others that the target is now tracked
  - Start tracking the target
Assumptions:
- Equal number of UAV nodes and targets. For this scenario (see uav8-notrack-new-gui.xml), there are 8 UAVs and 8 targets.
- Each UAV nodes should track a unique target. No two nodes should track the same target.
- Once a UAV selects a target to track, the decision cannot be changed until the target is moved out of range.
- Up to 20% packet loss and 200 ms delay.
- UAVs should be within range of each other for communication. Do not need to account for loss of range among UAVs.
- UAVs are pretty close to one another. So, multiple UAVs can detect the same target within its range.
- An agreement protocol must be established on which UAV should take that target. Although each UAV node may know of the other UAV and targets node ids, avoid assigning arbitrary numbering to match UAV node to target. (Ex. uavnode n1 always matches with targetnode n11.) The UAV nodes should communicate using an agreement protocol to decide which UAV should take a target.

The goal is to have an agreement protocol among the UAV nodes to avoid them tracking the same target. Currently, the code implementation has UAV nodes in a multicast group, but each node is unaware of what other potential UAVs are in the group.
