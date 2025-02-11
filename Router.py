from Module.DB import *
from Device_Router.GridGraph import GridGraph
from Device_Router.Maze_Algorithm import *
from Device_Router.LayoutProcess import Preprocess
import rdp

def maze_routing(tech: Tech, circuit: Circuit, routing_layers: int) -> None:
    """
    @brief      Maze routing algorithm
    @param      tech     The technology
    @param      circuit  The circuit
    """
    # Initialize 
    circuit.group["routing"] = Group()
    circuit.group["routing"].shape = {"poly": [], "metal1": [], "metal2": [], "metal3": [], "metal4": [], "metal5": [], "metal6": [],
                               "contact": [], "via12": [], "via23": [], "via34": [], "via45": [], "via56": []}
    route = Preprocess(tech)

    # pin and port grouping
    print("Pin Port Grouping")
    combine_pin_port = route.pin_port_grouping2(circuit)

    # find the points to route
    print("Pin Port Find Points")
    routing_net = route.pin_port_find_points2(tech, combine_pin_port)

    # route for each net
    print("Maze Routing for each Net")
    for name in routing_net:
        
        # create grid graph 
        grid_div = 1
        while True:
            print("\nNET "+name)
            print(">> Create Grid Graph")
            grid = GridGraph(tech, routing_layers)
            grid.create_grid_graph(routing_net[name], grid_div)

            # obstacle mapping
            print(">> Obstacle Mapping")
            route.diffusion_blockage(tech, circuit, grid)
            route.route_path_blockage(tech, circuit, grid)
            route.poly_pin_blockage2(tech, circuit, grid, name)
            route.metal_pin_blockage(tech, circuit, grid, name)

            # maze routing
            print(">> Grid Connection")
            grid.grid_connections()
            
            # group pin list
            netlist = []
            for net in routing_net[name]:
                # print(net)
                pinlist = []
                for pin in net:
                    node = grid.get_grid_node(pin)
                    # block vertical routing
                    node.vertical_block = True
                    pinlist.append(node)
                netlist.append(pinlist)

            print(">> Route Multiple Pins Group")
            paths = route_multi_pins_group(grid.grid3d, netlist)
            if paths == None and grid_div < 3:
                print("No path found")
                grid_div += 1
                continue
            
            break
        
        if paths:
            trim_paths = trim_path(paths)

            # layout
            print(">> Layout Generation")
            route.path_layout(tech, circuit.group["routing"], trim_paths)


def trim_path(paths: list) -> list:
    # preprocess the path
    process_paths = []
    for path in paths:
        tmp = []
        for node in path:
            tmp.append((node.x, node.y, node.z))
        process_paths.append(tmp)

    # remove the redundant nodes
    trim_path = []
    for path in process_paths:
        rdppath = rdp.rdp(path, epsilon=0.2)
        trim_path.append(rdppath)

    return trim_path


def port_placement(tech: Tech, circuit: Circuit, routing_layers: int) -> None:
    # each port
    for port_id in circuit.port:
        curr_port = circuit.port[port_id]
        # print("Port:", curr_port.name)

        # check the existence of the port
        port_exist = False
        label = {}
        for layer in range(1, routing_layers+1, 1):
            label["metal"+str(layer)] = "m"+str(layer)+"_text"
            if "m"+str(layer)+"_text" in curr_port.shape:
                # print("Layer:", "m"+str(layer)+"_text")
                # for shape in curr_port.shape["m"+str(layer)+"_text"]:
                #     print("Coordinates:", shape.x, shape.y)
                port_exist = True
                break
        # print()
        
        if not port_exist:
           # create port text from pin
            for group_id in circuit.group:
                curr_group = circuit.group[group_id]
                
                for pin in curr_group.pin:
                    if pin.net == curr_port.name:
                        if pin.layer == "poly":
                            continue

                        # create port text
                        layer = label[pin.layer]
                        x = (pin.pt1[0] + pin.pt2[0])/2
                        y = (pin.pt1[1] + pin.pt2[1])/2

                        circuit.port[port_id].shape[layer] = [Text(layer, [x, y], curr_port.name)]
