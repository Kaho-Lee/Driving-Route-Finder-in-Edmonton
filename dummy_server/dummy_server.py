import sys
from cs_message import *
from adjacencygraph import AdjacencyGraph
import math


"""Library import"""

"""
A very basic test server that can exercise the route finding client,
as well as display the diagnostic messages that it generates.

You can use this to test your client's basic functionality, and to
see if a large path that consumes too much memory will be properly
handled.

Typically run with
    python3 dummy_server.py -s /dev/ttyACM0

optional arguments:
  -h, --help            show this help message and exit
  -d0                   Debug off
  -s [SERIAL_PORT_NAME]
                        Set serial port for protocol

"""
g = AdjacencyGraph()
geo_info = dict()
weights = dict()
cost  = lambda x,y : weights.get((x,y),float("inf"))


class MinHeap:

    def __init__(self):
        self._array = []

    def add(self, key, value):
        self._array.append((key, value))
        self.fix_heap_up(len(self._array)-1)

    def pop_min(self):
        if not self._array:
            raise RuntimeError("Attempt to call pop_min on empty heap")
        retval = self._array[0]
        self._array[0] = self._array[-1]
        del self._array[-1]
        if self._array:
            self.fix_heap_down(0)
        return retval

    def fix_heap_up(self, i):
        if self.isroot(i):
            return
        p = self.parent(i)
        if self._array[i][0] < self._array[p][0]:
            self.swap(i, p)
            self.fix_heap_up(p)

    def swap(self, i, j):
        self._array[i], self._array[j] = \
            self._array[j], self._array[i]

    def isroot(self, i):
        return i == 0

    def isleaf(self, i):
        return self.lchild(i) >= len(self._array)

    def lchild(self, i):
        return 2*i+1

    def rchild(self, i):
        return 2*i+2

    def parent(self, i):
        return (i-1)//2

    def min_child_index(self, i):
        l = self.lchild(i)
        r = self.rchild(i)
        retval = l
        if r < len(self._array) and self._array[r][0] < self._array[l][0]:
            retval = r
        return retval

    def isempty(self):
        return len(self._array) == 0

    def length(self):
        return len(self._array)

    # So the len() function will work.
    def __len__(self):
        return len(self._array)

    def fix_heap_down(self, i):
        if self.isleaf(i):
            return

        j = self.min_child_index(i)
        if self._array[i][0] > self._array[j][0]:
            self.swap(i, j)
            self.fix_heap_down(j)

def test_minheap():
    # testing against the heapq module
    import random
    import heapq
    tracing = False

    def test_pop(heap, ourheap):
        if len(heap) == 0 and ourheap.isempty():
            return
        item = heapq.heappop(heap)
        item2 = our_heap.pop_min()
        if item != item2[0]:
            raise RuntimeError("Unequal elements %s!=%s" % (item, item2[0]))
        tracing and print("Popped", item)

    for n in range(100):  # length
        print("Testing for length %s" % n)
        for i in range(100):  # random instances
            s = [random.randrange(-10, 10) for j in range(n)]
            heap = []
            our_heap = MinHeap()
            for x in s:
                heapq.heappush(heap, x)
                our_heap.add(x, None)
                tracing and print("Added ", x)
                if random.uniform(0, 1) < 0.2:
                    tracing and print("Popping")
                    test_pop(heap, our_heap)
            while heap:
                tracing and print("Popping")
                test_pop(heap, our_heap)
            if not our_heap.isempty():
                raise RuntimeError("Too few elements")
    print("Test finished")

def least_cost_path(g, start, dest, cost):
    """
    Use Dijkstra's algorithm to compute a min cost path from root
    to all vertices reachable from root by a directed path.
    Args:
    g=(V, E) specifies the graph

    start the vertice of the stariting position

    dest the vertice of the destination position

    cost(x, y) is a function which takes an edge and returns the cost of
        following edge x -> y. cost returns None if there is no edge.

    Yields:

    a list of the vertice which having lowest cost from the starting
    psition to destination position
    """
    time = 0
    result_list = []
    reached = dict()#reached[v_to]=(cost, v_from)
    runner = MinHeap()
    runner.add(0,(0,start,start)) #(key,(cost, start, destination))
    while len(runner)>0:
        (key, runner_go) = runner.pop_min()
        if runner_go[2] not in reached.keys() or runner_go[0] < reached[runner_go[2]][0]:
            reached[runner_go[2]] = (runner_go[0], runner_go[1])
        else:
            continue
        for items in g.neighbours(runner_go[2]):
            if items not in reached.keys() or (cost(runner_go[2], items) + runner_go[0]) < reached[items][0]:
                time = cost(runner_go[2],items)
                runner.add(runner_go[0], (runner_go[0]+time, runner_go[2], items))
            else:
                continue
    if len(runner) == 0 and dest not in reached.keys():
        return []
    #print(reached)
    list_info = reached[dest]
    result_list.append(dest)
    stops = list_info[1]
    while  stops != start:
        result_list.append(stops)
        list_info = reached[stops]
        stops = list_info[1]
    result_list.append(start)
    result_list = result_list[::-1]
    #print(cost)
    # print(result_list)
    if result_list[0] == result_list[1]:
        result_list = []
    return result_list

def dataprocess(f_name):
    """
    This fucniton is create a graph from a text file
    Args:
    The name of the filed(include .type)

    Yiled: Graph , geo_info , weights
    Graph:
    The Graph stored the information form the
    Text file

    geo_info:
    return all the dictionary which contains
    verticee and it's latitude and lontitue
    ['keys':latitude,lontitue]

    weights:
    return a dictionary which contains all the edge and
    edges' distance

    """
    data_list = list()
    input_file = open(f_name,'r')
    line = input_file.readlines()
    for data in line:
        data = data.strip().split(',')
        data_list.append(data)
        #print(data)
    for a in data_list:
        if a[:][0] == 'V':
            g.add_vertex(int(a[:][1]))
            geo_info[(a[:][1])] = a[:][2],a[:][3]
        elif a[:][0] == 'E':
            g.add_edge((int(a[:][1]),int(a[:][2])))
            weights[int(a[:][1]),int(a[:][2])] = cost_distance(a[:][1],a[:][2])
    return(g)

def cost_distance(u,v):
    """
    The cost_distance fucntion is for calculating the geological
    distance betwwen two veritice
    Args:(u,v)
    is the vertices, both of them
    return:distance
    The distance between two vertices
    """
    u = str(u)
    v = str(v)
    u_pos1 = int(float(geo_info[u][0])*100000)
    u_pos2 = int(float(geo_info[u][1])*100000)
    v_pos1 = int(float(geo_info[v][0])*100000)
    v_pos2 = int(float(geo_info[v][1])*100000)
    distance = ((u_pos1 - v_pos1)**2 + (u_pos2 - v_pos2)**2)**(1/2)
    return distance

def data_extra(lat_s,lon_s,lat_e,lon_e):
    """
    data_extra() is for displaying the test in the terminal and
    finding the closest vertices by given arbitrary latitude and lontitue

    """
    count2 = 0
    geo_info['start'] = (str(float(lat_s)/100000),str(float(lon_s)/100000))
    geo_info['end'] = (str(float(lat_e)/100000), str(float(lon_e)/100000))
    """
    receieving the data from user input
    """
    near_startlist = MinHeap()
    near_endlist = MinHeap()
    """
    calcualting all the distance from staring position and destination
    position to all the vertice
    adding these imformation into MinHeap and poping the smallest
    distance from staring position and destination position
    """
    for i in geo_info.keys():
        if i != 'start' and  i != 'end':
            dist = cost_distance('start',i)
            near_startlist.add(dist, i)
            dist = cost_distance('end',i)
            near_endlist.add(dist, i)
    (near_start, startindex) = near_startlist.pop_min()
    (near_end, endindex) = near_endlist.pop_min()

    final_result = least_cost_path(g,int(startindex),int(endindex),cost)
    """
    calling least_cost_path() function to get the path with lowest cost
    """
    # waypoints = len(final_result)
    """
    The length of the path is equalt to waypoints
    """
    return final_result


def protocol(serial_in, serial_out):
    # simple echo protocol
    while True:
        while True:
            msg = receive_msg_from_client(serial_in)
            log_msg(msg)
            if msg[0] == "R":
                print("the received msg :",msg)
                break

        # Hope that it's a properly formatted R message
        coords = msg[2:].split()

        if len(coords) != 4:
            continue

        (lat_s, lon_s, lat_e, lon_e) = coords
        lat_s = int(lat_s)
        lon_s = int(lon_s)
        lat_e = int(lat_e)
        lon_e = int(lon_e)
        # print("coordinate :" , lat_s,lon_s,lat_e,lon_e)
        final_result = list()
        final_result = data_extra(lat_s,lon_s,lat_e,lon_e)
        # if final_result == []:
        #     print("stuck")
        #     continue
        n = len(final_result)
        # print("N value: ",n)
        #print(final_result)
        # delta_lat = lat_e - lat_s
        # delta_lon = lon_e - lon_s
        # write a fake waypoint path to client, make n large to break the
        # client.
        print("N")
        send_msg_to_client(serial_out, "N {}" .format(n))

        # while True :
        #     ack = receive_msg_from_client(serial_in)
        #     log_msg(ack)
        #     # print("ack :",ack)
        #     if ack[0] == "A":
        #         ##print("received ack")
        #         break

        for i in range(0,n):
            while True :
                ack = receive_msg_from_client(serial_in)
                log_msg(ack)
                if ack[0] == "A":
                    # print("received ack")
                    break
            lat = int(float(geo_info[str(final_result[i])][0])*100000)
            lon = int(float(geo_info[str(final_result[i])][1])*100000)
            send_msg_to_client(serial_out, "W {} {}" .format(lat, lon))
            # msg = receive_msg_from_client(serial_in)
            # log_msg(msg)
        while True :
            ack = receive_msg_from_client(serial_in)
            log_msg(ack)
            if ack[0] == "A":
                # print("received ack")
                break
        send_msg_to_client(serial_out, "E")


def main():

    import argparse
    parser = argparse.ArgumentParser(
        description='Client-server message test.',
        formatter_class=argparse.RawTextHelpFormatter,
        )

    parser.add_argument("-d0",
        help="Debug off",
        action="store_false",
        dest="debug")

    parser.add_argument("-s",
        help="Set serial port for protocol",
        nargs="?",
        type=str,
        dest="serial_port_name",
        default="/dev/ttyACM0")

    args = parser.parse_args()

    debug = args.debug

    set_logging(debug)

    # this imports serial, and provides a useful wrapper around it
    import textserial

    serial_port_name = args.serial_port_name;
    log_msg("Opening serial port: {}".format(serial_port_name))

    # Open up the connection
    baudrate = 9600  # [bit/seconds] 115200 also works

    # Run the server protocol forever

    # The with statment ensures that if things go bad, then ser
    # will still be closed properly.
    # errors='ignore' allows any 1 byte character, not just the usual
    # ascii range [0,127]

    with textserial.TextSerial(
        serial_port_name, baudrate, errors='ignore', newline=None) as ser:
        protocol(ser, ser)

if __name__ == "__main__":
    dataprocess('edmonton-roads-2.0.1.txt')
    main()
