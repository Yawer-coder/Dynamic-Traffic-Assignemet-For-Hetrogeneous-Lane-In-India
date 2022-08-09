import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()

def create_origin_pair(m) :
    origin_pair = []
    for i in range(m):
        origin = int(input("Please input an origin: "))
        
        origin_pair.append(origin)
    return origin_pair
def create_destination_pair(m):
    destination_pair = []
    for i in range(m):
        destination = int(input("Please input an destination: "))
        destination_pair.append(destination)
    return destination_pair

def create_o_d_pair(arr1,arr2) :
    o_d_pair = []
    for i in range(len(arr1)):
        for j in range(len(arr2)) :
            o_d_pair.append((arr1[i], arr2[j]))
    return o_d_pair
def generate_edge_list(n):
    edge_list = []
    for i in range(n):

        e = int(input("Please input an edge: "))
        u = int(input("Please input an connected edge: "))
        edge_list.append((e, u))
    return edge_list


nodes = int(input("Please input the number of nodes: "))
edges = int(input("Please input the number of edges: "))
no_of_orgin = int(input("Please input the number of orgin: "))

edges_list = generate_edge_list(edges)

G.add_edges_from(edges_list)
pos = nx.spring_layout(G)

nx.draw_networkx_nodes(G, pos, node_size=500)
nx.draw_networkx_edges(G, pos, edgelist=G.edges(), width=2, edge_color='r')
nx.draw_networkx_labels(G, pos, font_size=10, font_family='sans-serif')
plt.show()


def input_info():
    a = int(input('enter no. of vehicles: '))
    initial_speed = [60, 25, 54]
    types_of_vehical = ['Car', 'Scooter,Bike', 'Truck,Bus']
    dict1 = {

        "type_of_vehicles": types_of_vehical,
        "speed": initial_speed, }
   
    for i in range(a):
        print("The keys are :- speed, new_vehicle")
        k = input("Please input a key: ")
        
        v = input("Please input a value: ")

        if k == "speed":
            v = int(v)
            initial_speed.append(v)

            dict1[k] = initial_speed
            print(dict1)
        elif k=="new_vehicle":
            types_of_vehical.append(v)
            dict1["type_of_vehicles"] = types_of_vehical
            print(dict1)
        else :
            print("unable to find the data related to key please verify.")


input_info()
 