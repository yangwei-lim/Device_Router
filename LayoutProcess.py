from Module.DB import *
from Device_Router.GridGraph import GridGraph
from Device_Generator.engineering_notation import EngNumber as eng
import math

class Preprocess:
    def __init__(self, tech: Tech, total_layers: int=7) -> None:
        self.total_layers = total_layers
        self.get_design_rule(tech)

    def get_design_rule(self, tech: Tech) -> None:
        self.pitch = []
        self.rt_layer2int = {}
        self.int2rt_layer = {}
        self.via_layer2int = {}
        self.int2via_layer = {}

        for layer_idx in range(self.total_layers):
            if layer_idx == 0:
                min_width = tech.min_width_rule["poly"]
                min_spacing = tech.min_spacing_rule[("poly","poly")]

                self.rt_layer2int["poly"] = layer_idx
                self.int2rt_layer[layer_idx] = "poly"
                self.via_layer2int["contact"] = layer_idx
                self.int2via_layer[layer_idx] = "contact"

            else:
                # check the min width and min spacing exist
                if "metal"+str(layer_idx) not in tech.min_width_rule or ("metal"+str(layer_idx),"metal"+str(layer_idx)) not in tech.min_spacing_rule:
                    print("Error: metal"+str(layer_idx)+" min width or min spacing is not defined.")
                    exit(1)

                min_width = tech.min_width_rule["metal"+str(layer_idx)]
                min_spacing = tech.min_spacing_rule[("metal"+str(layer_idx),"metal"+str(layer_idx))]
                self.rt_layer2int["metal"+str(layer_idx)] = layer_idx
                self.int2rt_layer[layer_idx] = "metal"+str(layer_idx)
                if layer_idx < self.total_layers:
                    self.via_layer2int["via"+str(layer_idx)+str(layer_idx+1)] = layer_idx
                    self.int2via_layer[layer_idx] = "via"+str(layer_idx)+str(layer_idx+1)

            self.pitch.append(min_width + min_spacing)


    def pin_port_grouping2(self, circuit: Circuit) -> dict:
        """
        Combine the circuit's ports and the group's pins together into the same net dictionary,
        because the ports are the input/output of the circuit, and the pins are the internal connections of the group,
        and they need to be connected together to form a net.
        """
        # get pin and port from circuit
        nets = {}

        # append the port into the nets (with the name)
        for name in circuit.port:
            if "metal1" in circuit.port[name].shape:
                m1_shape = circuit.port[name].shape["metal1"]

                pt1 = [m1_shape[0].x[0], m1_shape[0].y[0]]
                pt2 = [m1_shape[0].x[1], m1_shape[0].y[1]]
                nets[name] = [Pin(name, "metal1", pt1, pt2)]

        # append the pin from the group into the nets (with the name)
        for inst in circuit.group:
            # pin in the group
            for pin in circuit.group[inst].pin:
                if pin.net in nets:
                    nets[pin.net].append(pin)
                else:
                    nets[pin.net] = [pin]

        # group
        group_nets = {}
        for name in nets:
            group_nets[name] = self.find_groups(nets[name])
            
        return group_nets            


    def pin_port_find_points2(self, tech: Tech, group_nets: dict) -> dict:
        """
        Find the points to route the net from the pin and port dictionary.
        Use the points to generate grid graph for routing later.
        """
        # get the point to route
        routing_net = {}
        for name in group_nets:
            routing_net[name] = []

            for net in group_nets[name]:
                points = []
                for pin in net:
                    point = self.get_pin_points(tech, pin)
                    points += point
                    pin.grid = point
                    
                routing_net[name].append(points) if points else None

        return routing_net
    

    def check_interaction(self, pin1: Pin, pin2: Pin) -> bool:
        if pin1.layer != pin2.layer:
            return False
        
        interact = (
            pin1.pt1[0] <= pin2.pt2[0] and pin1.pt2[0] >= pin2.pt1[0] and  # Horizontal overlap or touch
            pin1.pt1[1] <= pin2.pt2[1] and pin1.pt2[1] >= pin2.pt1[1]     # Vertical overlap or touch
        )

        return interact


    def find_groups(self, nets: list) -> list:
        n = len(nets)
        parent = list(range(n))

        def find(x):
            if parent[x] != x:
                parent[x] = find(parent[x])
            return parent[x]
        
        def union(x, y):
            root_x = find(x)
            root_y = find(y)
            if root_x != root_y:
                parent[root_y] = root_x

        for i in range(n):
            for j in range(i+1, n):
                if self.check_interaction(nets[i], nets[j]):
                    union(i, j)

        groups = {}
        for i in range(n):
            root = find(i)
            if root not in groups:
                groups[root] = []
            groups[root].append(i)

        return [[nets[i] for i in group] for group in groups.values()]


    def get_pin_points(self, tech: Tech, pin: Pin) -> list:
        z = self.rt_layer2int[pin.layer]
        route_hw = tech.min_width_rule[pin.layer] / 2
        points = []

        # get the four end points
        x0 = round(pin.pt1[0]/tech.unit["user"]) + int(route_hw/tech.unit["user"])
        x1 = round(pin.pt2[0]/tech.unit["user"]) - int(route_hw/tech.unit["user"])
        y0 = round(pin.pt1[1]/tech.unit["user"]) + int(route_hw/tech.unit["user"])
        y1 = round(pin.pt2[1]/tech.unit["user"]) - int(route_hw/tech.unit["user"])

        # single end point
        if x0 == x1 and y0 == y1:
            points.append((x0, y0, z))
        else:
            # vertical pin (two end points)
            if x0 == x1:
                points.append((x0, y0, z))
                points.append((x0, y1, z))
            # horizontal pin (two end points)
            elif y0 == y1:
                points.append((x0, y0, z))
                points.append((x1, y0, z))
            # four end points
            else:
                points.append((x0, y0, z))
                points.append((x1, y1, z))
                points.append((x0, y1, z))
                points.append((x1, y0, z))

        return points
    

    def diffusion_blockage(self, tech: Tech, circuit: Circuit, graph: GridGraph):
        print("   >> Diffusion Blockage")
        # find the diffusion blockage
        for diff_layer in ["ndiffusion", "pdiffusion"]:
            # design rules
            df_spc_po = int(tech.min_spacing_rule[(diff_layer,"poly")]/tech.unit["user"])
            # po_hw = int(tech.min_width_rule["poly"]/2 /tech.unit["user"])                                                 # poly width
            po_hw = int((tech.min_size_rule["contact"] + tech.min_enclosure_rule["poly","contact"])/2 /tech.unit["user"])   # poly-cut width

            for inst in circuit.group:
                if diff_layer not in circuit.group[inst].shape:
                    continue

                for diff in circuit.group[inst].shape[diff_layer]:
                    # convert box to grid unit (db -> user)
                    df_x0 = round(diff.x[0] / tech.unit["user"])
                    df_x1 = round(diff.x[1] / tech.unit["user"])
                    df_y0 = round(diff.y[0] / tech.unit["user"])
                    df_y1 = round(diff.y[1] / tech.unit["user"])

                    # add spacing rules
                    df_x0 = df_x0 - df_spc_po - po_hw
                    df_x1 = df_x1 + df_spc_po + po_hw
                    df_y0 = df_y0 - df_spc_po - po_hw
                    df_y1 = df_y1 + df_spc_po + po_hw

                    # add blockage on the poly grid
                    for row in graph.grid3d[0]:
                        for node in row:
                            if node.x >= df_x0 and node.x <= df_x1 and node.y >= df_y0 and node.y <= df_y1:
                                node.obstacle = True
                                node.vertical_block = True


    def poly_pin_blockage2(self, tech: Tech, circuit: Circuit, graph: GridGraph, pin_name: str=""):
        print("   >> Poly Pin Blockage")
        for inst in circuit.group:
            for pin in circuit.group[inst].pin:
                if pin.layer == "poly":
                    # design rules
                    po_spc_po = int(tech.min_spacing_rule[("poly","poly")]/tech.unit["user"])    # poly space poly
                    po_hw = int(tech.min_width_rule["poly"]/2 /tech.unit["user"])                # poly half width
                    po_enc_co = int(tech.min_enclosure_rule["poly","contact"]/tech.unit["user"]) # poly enclosure contact
                    co_hs = int(tech.min_size_rule["contact"]/2 /tech.unit["user"])              # contact halfsize

                    # convert box to grid unit (db -> user)
                    po_x0 = round(pin.pt1[0] / tech.unit["user"])
                    po_x1 = round(pin.pt2[0] / tech.unit["user"])
                    po_y0 = round(pin.pt1[1] / tech.unit["user"])
                    po_y1 = round(pin.pt2[1] / tech.unit["user"])

                    # add spacing rules (block in the same layer)
                    po_x0_1 = po_x0 - po_spc_po - po_hw
                    po_x1_1 = po_x1 + po_spc_po + po_hw
                    po_y0_1 = po_y0 - po_spc_po - po_hw
                    po_y1_1 = po_y1 + po_spc_po + po_hw

                    # add spacing rules (block to the vertical layers)
                    po_x0_2 = po_x0 - po_spc_po - po_enc_co - co_hs
                    po_x1_2 = po_x1 + po_spc_po + po_enc_co + co_hs
                    po_y0_2 = po_y0 - po_spc_po - po_enc_co - co_hs
                    po_y1_2 = po_y1 + po_spc_po + po_enc_co + co_hs

                    # block the poly pin
                    for row in graph.grid3d[0]:
                        for node in row:
                            if node.x >= po_x0_1 and node.x <= po_x1_1 and node.y >= po_y0_1 and node.y <= po_y1_1:
                                node.obstacle = True

                            if node.x >= po_x0_2 and node.x <= po_x1_2 and node.y >= po_y0_2 and node.y <= po_y1_2:
                                node.vertical_block = True

                    # unblock the poly points in the grid if it is the current pin
                    if pin_name == pin.net:
                        for row in graph.grid3d[0]:
                            for node in row:

                                # within the poly shape
                                if node.x >= po_x0 and node.x <= po_x1 and node.y >= po_y0_1 and node.y <= po_y1_1:
                                    for point in pin.grid:

                                        # unblock the points in the same x-coordinates as the routing points
                                        if node.x == point[0] or node.y == point[1]:
                                            node.obstacle = False



    def metal_pin_blockage(self, tech: Tech, circuit: Circuit, graph: GridGraph, pin_name: str=""):
        print("   >> Metal Pin Blockage")
        metal_layer = {"poly": 0, "metal1": 1, "metal2": 2, "metal3": 3, "metal4": 4, "metal5": 5, "metal6": 6}
        via_layer = {0: "contact", 1: "via12", 2: "via23", 3: "via34", 4: "via45", 5: "via56", 6: "via56"}

        for inst in circuit.group:
            for pin in circuit.group[inst].pin:
                if pin.layer in ["metal1", "metal2", "metal3", "metal4", "metal5", "metal6"]:
                    # design rules
                    mx_spc_mx = int(tech.min_spacing_rule[(pin.layer,pin.layer)]/tech.unit["user"])
                    mx_hw = int(tech.min_width_rule[pin.layer]/2 /tech.unit["user"])
                    mx_enc_vx = int(tech.min_enclosure_rule[pin.layer,via_layer[metal_layer[pin.layer]]]/tech.unit["user"])
                    vx_hs = int(tech.min_size_rule[via_layer[metal_layer[pin.layer]]]/2 /tech.unit["user"])

                    # convert box to grid unit (db -> user)
                    mx_x0 = round(pin.pt1[0] / tech.unit["user"])
                    mx_x1 = round(pin.pt2[0] / tech.unit["user"])
                    mx_y0 = round(pin.pt1[1] / tech.unit["user"])
                    mx_y1 = round(pin.pt2[1] / tech.unit["user"])

                    # unblock the metal pin if it is the current pin
                    if pin_name == pin.net:
                        for row in graph.grid3d[metal_layer[pin.layer]]:
                            for node in row:
                                if node.x >= mx_x0 and node.x <= mx_x1 and node.y >= mx_y0 and node.y <= mx_y1:
                                    node.obstacle = False

                    # block the metal pin if it is not the current pin
                    else:
                        # add spacing rules
                        mx_x0_1 = mx_x0 - mx_spc_mx - mx_hw
                        mx_x1_1 = mx_x1 + mx_spc_mx + mx_hw
                        mx_y0_1 = mx_y0 - mx_spc_mx - mx_hw
                        mx_y1_1 = mx_y1 + mx_spc_mx + mx_hw

                        # add spacing rules
                        mx_x0_2 = mx_x0 - mx_spc_mx - mx_enc_vx - vx_hs
                        mx_x1_2 = mx_x1 + mx_spc_mx + mx_enc_vx + vx_hs
                        mx_y0_2 = mx_y0 - mx_spc_mx - mx_enc_vx - vx_hs
                        mx_y1_2 = mx_y1 + mx_spc_mx + mx_enc_vx + vx_hs

                        # block the metal pin
                        for row in graph.grid3d[metal_layer[pin.layer]]:
                            for node in row:
                                if node.x >= mx_x0_1 and node.x <= mx_x1_1 and node.y >= mx_y0_1 and node.y <= mx_y1_1:
                                    node.obstacle = True

                                if node.x >= mx_x0_2 and node.x <= mx_x1_2 and node.y >= mx_y0_2 and node.y <= mx_y1_2:
                                    node.vertical_block = True

        for port in circuit.port:
            if "metal1" in circuit.port[port].shape:
                for shape in circuit.port[port].shape["metal1"]:
                    pt1 = [shape.x[0], shape.y[0]]
                    pt2 = [shape.x[1], shape.y[1]]

                    # design rules
                    mx_spc_mx = int(tech.min_spacing_rule[("metal1","metal1")]/tech.unit["user"])
                    mx_hw = int(tech.min_width_rule["metal1"]/2 /tech.unit["user"])
                    mx_enc_vx = int(tech.min_enclosure_rule["metal1","via12"]/tech.unit["user"])
                    vx_hs = int(tech.min_size_rule["via12"]/2 /tech.unit["user"])

                    # convert box to grid unit (db -> user)
                    mx_x0 = round(pt1[0] / tech.unit["user"])
                    mx_x1 = round(pt2[0] / tech.unit["user"])
                    mx_y0 = round(pt1[1] / tech.unit["user"])
                    mx_y1 = round(pt2[1] / tech.unit["user"])

                    # unblock the metal pin if it is the current pin
                    if pin_name == port:
                        for row in graph.grid3d[metal_layer["metal1"]]:
                            for node in row:
                                if node.x >= mx_x0 and node.x <= mx_x1 and node.y >= mx_y0 and node.y <= mx_y1:
                                    node.obstacle = False

                    # block the metal pin if it is not the current pin
                    else:
                        # add spacing rules
                        mx_x0_1 = mx_x0 - mx_spc_mx - mx_hw
                        mx_x1_1 = mx_x1 + mx_spc_mx + mx_hw
                        mx_y0_1 = mx_y0 - mx_spc_mx - mx_hw
                        mx_y1_1 = mx_y1 + mx_spc_mx + mx_hw

                        # add spacing rules
                        mx_x0_2 = mx_x0 - mx_spc_mx - mx_enc_vx - vx_hs
                        mx_x1_2 = mx_x1 + mx_spc_mx + mx_enc_vx + vx_hs
                        mx_y0_2 = mx_y0 - mx_spc_mx - mx_enc_vx - vx_hs
                        mx_y1_2 = mx_y1 + mx_spc_mx + mx_enc_vx + vx_hs

                        # block the metal pin
                        for row in graph.grid3d[metal_layer["metal1"]]:
                            for node in row:
                                if node.x >= mx_x0_1 and node.x <= mx_x1_1 and node.y >= mx_y0_1 and node.y <= mx_y1_1:
                                    node.obstacle = True 

                                if node.x >= mx_x0_2 and node.x <= mx_x1_2 and node.y >= mx_y0_2 and node.y <= mx_y1_2:
                                    node.vertical_block = True


    def route_path_blockage(self, tech: Tech, circuit: Circuit, graph: GridGraph):
        print("   >> Route Path Blockage")
        route = {"poly": 0, "metal1": 1, "metal2": 2, "metal3": 3, "metal4": 4, "metal5": 5, "metal6": 6}
        via = {0: "contact", 1: "via12", 2: "via23", 3: "via34", 4: "via45", 5: "via56", 6: "via56"}

        for layer in route:
            # design rules
            rt_spc_rt = int(tech.min_spacing_rule[(layer,layer)]/tech.unit["user"])
            rt_hw = int(tech.min_width_rule[layer]/2 /tech.unit["user"])
            rt_enc_vx = int(tech.min_enclosure_rule[layer,via[route[layer]]]/tech.unit["user"])
            vx_hs = int(tech.min_size_rule[via[route[layer]]]/2 /tech.unit["user"])

            for shp in circuit.group["routing"].shape[layer]:
                # convert box to grid unit (db -> user)
                x0 = round(shp.x[0] / tech.unit["user"])
                x1 = round(shp.x[1] / tech.unit["user"])
                y0 = round(shp.y[0] / tech.unit["user"])
                y1 = round(shp.y[1] / tech.unit["user"])

                # add spacing rules
                x0_1 = x0 - rt_spc_rt - rt_hw
                x1_1 = x1 + rt_spc_rt + rt_hw
                y0_1 = y0 - rt_spc_rt - rt_hw
                y1_1 = y1 + rt_spc_rt + rt_hw

                # add spacing rules
                x0_2 = x0 - rt_spc_rt - rt_enc_vx - vx_hs
                x1_2 = x1 + rt_spc_rt + rt_enc_vx + vx_hs
                y0_2 = y0 - rt_spc_rt - rt_enc_vx - vx_hs
                y1_2 = y1 + rt_spc_rt + rt_enc_vx + vx_hs

                # add blockage on the grid
                for row in graph.grid3d[route[layer]]:
                    for node in row:
                        if node.x >= x0_1 and node.x <= x1_1 and node.y >= y0_1 and node.y <= y1_1:
                            node.obstacle = True

                        if node.x >= x0_2 and node.x <= x1_2 and node.y >= y0_2 and node.y <= y1_2:
                            node.vertical_block = True


    def path_layout(self, tech: Tech, group: Group, paths: list):
        metal = {0: "poly", 1: "metal1", 2: "metal2", 3: "metal3", 4: "metal4", 5: "metal5", 6: "metal6"}
        via = {(0,1): "contact", (1,2): "via12", (2,3): "via23", (3,4): "via34", (4,5): "via45", (5,6): "via56",
            (1,0): "contact", (2,1): "via12", (3,2): "via23", (4,3): "via34", (5,4): "via45", (6,5): "via56"}

        for path in paths:

            ##### Poly or Metal Route Layout #####
            for i in range(len(path)-1):
                # same layer for current and next node
                if path[i][2] == path[i+1][2]:      
                    # add metal
                    metal_layer = metal[path[i][2]]
                
                    # X-coordinates
                    if path[i][0] == path[i+1][0]:
                        # poly path
                        if path[i][2] == 0:
                            # print("poly path ver (x-coor):", path[i][0], path[i+1][0])
                            x0 = (path[i][0] * tech.unit["user"]) - tech.min_width_rule["poly"] / 2
                            x1 = (path[i][0] * tech.unit["user"]) + tech.min_width_rule["poly"] / 2
                        # metal path
                        else:
                            x0 = (path[i][0] * tech.unit["user"]) - tech.min_width_rule[metal_layer] / 2
                            x1 = (path[i][0] * tech.unit["user"]) + tech.min_width_rule[metal_layer] / 2
                    else:
                        # poly path
                        if path[i][2] == 0:
                            # print("poly path hor (x-coor)", path[i][0], path[i+1][0])
                            x0 = min(path[i][0], path[i+1][0]) * tech.unit["user"] - tech.min_width_rule["poly"] / 2
                            x1 = max(path[i][0], path[i+1][0]) * tech.unit["user"] + tech.min_width_rule["poly"] / 2
                        # metal path
                        else:
                            x0 = min(path[i][0], path[i+1][0]) * tech.unit["user"] - tech.min_width_rule[metal_layer] / 2
                            x1 = max(path[i][0], path[i+1][0]) * tech.unit["user"] + tech.min_width_rule[metal_layer] / 2

                    # Y-coordinates
                    if path[i][1] == path[i+1][1]:
                        # poly path
                        if path[i][2] == 0:
                            # print("poly path hor (y-coor)", path[i][1], path[i+1][1])
                            y0 = (path[i][1] * tech.unit["user"]) - tech.min_width_rule["poly"] / 2
                            y1 = (path[i][1] * tech.unit["user"]) + tech.min_width_rule["poly"] / 2
                        # metal path
                        else:
                            y0 = (path[i][1] * tech.unit["user"]) - tech.min_width_rule[metal_layer] / 2
                            y1 = (path[i][1] * tech.unit["user"]) + tech.min_width_rule[metal_layer] / 2
                    else:
                        # poly path
                        if path[i][2] == 0:
                            # print("poly path ver (y-coor)", path[i][1], path[i+1][1])
                            y0 = min(path[i][1], path[i+1][1]) * tech.unit["user"] - tech.min_width_rule["poly"] / 2
                            y1 = max(path[i][1], path[i+1][1]) * tech.unit["user"] + tech.min_width_rule["poly"] / 2
                        # metal path
                        else:
                            y0 = min(path[i][1], path[i+1][1]) * tech.unit["user"] - tech.min_width_rule[metal_layer] / 2
                            y1 = max(path[i][1], path[i+1][1]) * tech.unit["user"] + tech.min_width_rule[metal_layer] / 2

                    # add the shape
                    group.shape[metal_layer].append(Box(metal_layer, [x0, y0], [x1, y1]))

            
            ##### Via Layout #####
            prev_direction, next_direction = "", ""
            prev_distance, next_distance = 0, 0
            for i in range(len(path)-1):
                # different layer for current and next node
                if path[i][2] != path[i+1][2]:
                    # get previous direction
                    if i > 0:
                        if path[i-1][0] < path[i][0]:
                            prev_direction = "left_to_right"
                            prev_distance = path[i][0] - path[i-1][0]
                        elif path[i-1][0] > path[i][0]:
                            prev_direction = "right_to_left"
                            prev_distance = path[i-1][0] - path[i][0]
                        elif path[i-1][1] < path[i][1]:
                            prev_direction = "down_to_up"
                            prev_distance = path[i][1] - path[i-1][1]
                        elif path[i-1][1] > path[i][1]:
                            prev_direction = "up_to_down"
                            prev_distance = path[i-1][1] - path[i][1]
                    else:
                        prev_direction = "left_to_right"
                        prev_distance = 0

                    # previous layer
                    prev_layer = metal[path[i][2]]
                    prev_via_layer = via[(path[i][2], path[i][2]+1)] if path[i][2] < path[i+1][2] else via[(path[i][2], path[i][2]-1)]
                    prev_route_width = tech.min_size_rule[prev_via_layer] + 2*tech.min_enclosure_rule[prev_layer, prev_via_layer]
                    prev_route_width_eol = tech.min_size_rule[prev_via_layer] + 2*tech.min_enclosure_rule[prev_layer, prev_via_layer, "end"]

                    if prev_direction == "left_to_right":
                        # print("prev_direction: left_to_right","distance:", prev_distance)
                        if (prev_distance * tech.unit["user"]) > prev_route_width_eol + tech.min_spacing_rule[(prev_layer, prev_layer)]:
                            x0 = (path[i][0] * tech.unit["user"]) - prev_route_width_eol / 2
                        else:
                            x0 = (path[i-1][0] * tech.unit["user"]) - tech.min_width_rule[prev_layer] / 2

                        x1 = (path[i][0] * tech.unit["user"]) + prev_route_width_eol / 2
                        y0 = (path[i][1] * tech.unit["user"]) - prev_route_width / 2
                        y1 = (path[i][1] * tech.unit["user"]) + prev_route_width / 2

                    elif prev_direction == "right_to_left":
                        # print("prev_direction: right_to_left","prev_distance:", prev_distance)
                        if (prev_distance * tech.unit["user"]) > prev_route_width_eol + tech.min_spacing_rule[(prev_layer, prev_layer)]:
                            x1 = (path[i][0] * tech.unit["user"]) + prev_route_width_eol / 2
                        else:
                            x1 = (path[i-1][0] * tech.unit["user"]) + tech.min_width_rule[prev_layer] / 2

                        x0 = (path[i][0] * tech.unit["user"]) - prev_route_width_eol / 2
                        y0 = (path[i][1] * tech.unit["user"]) - prev_route_width / 2
                        y1 = (path[i][1] * tech.unit["user"]) + prev_route_width / 2

                    elif prev_direction == "down_to_up":
                        # print("prev_direction: down_to_up","prev_distance:", prev_distance)
                        if (prev_distance * tech.unit["user"]) > prev_route_width_eol + tech.min_spacing_rule[(prev_layer, prev_layer)]: 
                            y0 = (path[i][1] * tech.unit["user"]) - prev_route_width_eol / 2
                        else:
                            y0 = (path[i-1][1] * tech.unit["user"]) - tech.min_width_rule[prev_layer] / 2

                        x0 = (path[i][0] * tech.unit["user"]) - prev_route_width / 2
                        x1 = (path[i][0] * tech.unit["user"]) + prev_route_width / 2
                        y1 = (path[i][1] * tech.unit["user"]) + prev_route_width_eol / 2
                    
                    elif prev_direction == "up_to_down":
                        # print("prev_direction: up_to_down","prev_distance:", prev_distance)
                        if (prev_distance * tech.unit["user"]) > prev_route_width_eol + tech.min_spacing_rule[(prev_layer, prev_layer)]:
                            y1 = (path[i][1] * tech.unit["user"]) + prev_route_width_eol / 2
                        else:
                            y1 = (path[i-1][1] * tech.unit["user"]) + tech.min_width_rule[prev_layer] / 2

                        x0 = (path[i][0] * tech.unit["user"]) - prev_route_width / 2
                        x1 = (path[i][0] * tech.unit["user"]) + prev_route_width / 2
                        y0 = (path[i][1] * tech.unit["user"]) - prev_route_width_eol / 2

                    # add the shape
                    group.shape[prev_layer].append(Box(prev_layer, [x0, y0], [x1, y1]))

                    # via layer
                    via_width = tech.min_size_rule[prev_via_layer]

                    x0 = (path[i][0] * tech.unit["user"]) - via_width / 2
                    x1 = (path[i][0] * tech.unit["user"]) + via_width / 2
                    y0 = (path[i][1] * tech.unit["user"]) - via_width / 2
                    y1 = (path[i][1] * tech.unit["user"]) + via_width / 2

                    # add the shape
                    group.shape[prev_via_layer].append(Box(prev_via_layer, [x0, y0], [x1, y1]))

                    # middle layer
                    # print("path[i][2]:", path[i][2], "path[i+1][2]:", path[i+1][2])
                    if path[i][2] < path[i+1][2]:
                        start = int(path[i][2])+1
                        end = int(path[i+1][2])
                        step = 1
                    else:
                        start = int(path[i][2])-1
                        end = int(path[i+1][2])
                        step = -1
    
                    for mid_layer_idx in range(start, end, step):
                        # print("mid_layer_idx:", mid_layer_idx)
                        mid_layer = metal[mid_layer_idx]
                        mid_via_layer = via[(mid_layer_idx, mid_layer_idx+1)] if mid_layer_idx < path[i][2] else via[(mid_layer_idx, mid_layer_idx-1)]
                        mid_route_width = tech.min_size_rule[mid_via_layer] + 2*tech.min_enclosure_rule[mid_layer, mid_via_layer, "end"]
                        mid_route_width_wide = tech.min_area_rule[mid_layer] / mid_route_width
                        mid_route_width_wide = math.ceil(mid_route_width_wide / 2 / tech.unit["grid"]) * 2 * tech.unit["grid"]

                        if prev_direction == "left_to_right" or prev_direction == "right_to_left":
                            x0 = (path[i][0] * tech.unit["user"]) - mid_route_width_wide / 2
                            x1 = (path[i][0] * tech.unit["user"]) + mid_route_width_wide / 2
                            y0 = (path[i][1] * tech.unit["user"]) - mid_route_width / 2
                            y1 = (path[i][1] * tech.unit["user"]) + mid_route_width / 2
                        elif prev_direction == "down_to_up" or prev_direction == "up_to_down":
                            x0 = (path[i][0] * tech.unit["user"]) - mid_route_width / 2
                            x1 = (path[i][0] * tech.unit["user"]) + mid_route_width / 2
                            y0 = (path[i][1] * tech.unit["user"]) - mid_route_width_wide / 2
                            y1 = (path[i][1] * tech.unit["user"]) + mid_route_width_wide / 2

                        # add the shape
                        group.shape[mid_layer].append(Box(mid_layer, [x0, y0], [x1, y1]))

                        # via layer
                        via_width = tech.min_size_rule[mid_via_layer]

                        x0 = (path[i][0] * tech.unit["user"]) - via_width / 2
                        x1 = (path[i][0] * tech.unit["user"]) + via_width / 2
                        y0 = (path[i][1] * tech.unit["user"]) - via_width / 2
                        y1 = (path[i][1] * tech.unit["user"]) + via_width / 2

                        # add the shape
                        group.shape[mid_via_layer].append(Box(mid_via_layer, [x0, y0], [x1, y1]))

                    # get next direction
                    if i < len(path)-2:
                        if path[i+1][0] < path[i+2][0]:
                            next_direction = "left_to_right"
                            next_distance = path[i+2][0] - path[i+1][0]
                        elif path[i+1][0] > path[i+2][0]:
                            next_direction = "right_to_left"
                            next_distance = path[i+1][0] - path[i+2][0]
                        elif path[i+1][1] < path[i+2][1]:
                            next_direction = "down_to_up"
                            next_distance = path[i+2][1] - path[i+1][1]
                        elif path[i+1][1] > path[i+2][1]:
                            next_direction = "up_to_down"
                            next_distance = path[i+1][1] - path[i+2][1]
                    else:
                        next_direction = "left_to_right"
                        next_distance = 0

                    # next layer
                    next_layer = metal[path[i+1][2]]
                    next_via_layer = via[(path[i+1][2], path[i+1][2]+1)] if path[i+1][2] < path[i][2] else via[(path[i+1][2], path[i+1][2]-1)]
                    next_route_width = tech.min_size_rule[next_via_layer] + 2*tech.min_enclosure_rule[next_layer, next_via_layer]
                    next_route_width_eol = tech.min_size_rule[next_via_layer] + 2*tech.min_enclosure_rule[next_layer, next_via_layer, "end"]

                    if next_direction == "left_to_right":
                        # print("next_direction: left_to_right","next_distance:", next_distance)
                        if (next_distance * tech.unit["user"]) > next_route_width_eol + tech.min_spacing_rule[(next_layer, next_layer)]:
                            x1 = (path[i+1][0] * tech.unit["user"]) + next_route_width_eol / 2
                        else:
                            if i < len(path)-2:
                                x1 = (path[i+2][0] * tech.unit["user"]) + tech.min_width_rule[next_layer] / 2
                            else:
                                x1 = (path[i+1][0] * tech.unit["user"]) + next_route_width_eol / 2
                            # x1 = (path[i+2][0] * tech.unit["user"]) + tech.min_width_rule[next_layer] / 2 

                        x0 = (path[i+1][0] * tech.unit["user"]) - next_route_width_eol / 2
                        y0 = (path[i+1][1] * tech.unit["user"]) - next_route_width / 2
                        y1 = (path[i+1][1] * tech.unit["user"]) + next_route_width / 2

                    elif next_direction == "right_to_left":
                        # print("next_direction: right_to_left","next_distance:", next_distance)
                        if (next_distance * tech.unit["user"]) > next_route_width_eol + tech.min_spacing_rule[(next_layer, next_layer)]:
                            x0 = (path[i+1][0] * tech.unit["user"]) - next_route_width_eol / 2
                        else:
                            if i < len(path)-2:
                                x0 = (path[i+2][0] * tech.unit["user"]) - tech.min_width_rule[next_layer] / 2
                            else:
                                x0 = (path[i+1][0] * tech.unit["user"]) - next_route_width_eol / 2
                            # x0 = (path[i+2][0] * tech.unit["user"]) - tech.min_width_rule[next_layer] / 2

                        x1 = (path[i+1][0] * tech.unit["user"]) + next_route_width_eol / 2
                        y0 = (path[i+1][1] * tech.unit["user"]) - next_route_width / 2
                        y1 = (path[i+1][1] * tech.unit["user"]) + next_route_width / 2

                    elif next_direction == "down_to_up":
                        # print("next_direction: down_to_up","next_distance:", next_distance)
                        if (next_distance * tech.unit["user"]) > next_route_width_eol + tech.min_spacing_rule[(next_layer, next_layer)]:
                            y1 = (path[i+1][1] * tech.unit["user"]) + next_route_width_eol / 2
                        else:
                            y1 = (path[i+2][1] * tech.unit["user"]) + tech.min_width_rule[next_layer] / 2

                        x0 = (path[i+1][0] * tech.unit["user"]) - next_route_width / 2
                        x1 = (path[i+1][0] * tech.unit["user"]) + next_route_width / 2
                        y0 = (path[i+1][1] * tech.unit["user"]) - next_route_width_eol / 2

                    elif next_direction == "up_to_down":
                        # print("next_direction: up_to_down","next_distance:", next_distance)
                        if (next_distance * tech.unit["user"]) > next_route_width_eol + tech.min_spacing_rule[(next_layer, next_layer)]:
                            y0 = (path[i+1][1] * tech.unit["user"]) - next_route_width_eol / 2
                        else:
                            y0 = (path[i+2][1] * tech.unit["user"]) - tech.min_width_rule[next_layer] / 2

                        x0 = (path[i+1][0] * tech.unit["user"]) - next_route_width / 2
                        x1 = (path[i+1][0] * tech.unit["user"]) + next_route_width / 2
                        y1 = (path[i+1][1] * tech.unit["user"]) + next_route_width_eol / 2

                    # add the shape
                    group.shape[next_layer].append(Box(next_layer, [x0, y0], [x1, y1]))