from Module.DB import *
import numpy as np
import matplotlib.pyplot as plt

class GridNode:
    def __init__(self, x: int, y: int, z: int) -> None:
        # x, y and z coordinates
        self.x = x
        self.y = y
        self.z = z

        # neighbors nodes
        self.up = None
        self.down = None
        self.left = None
        self.right = None
        self.top = None
        self.bottom = None

        # obstacle
        self.obstacle = False
        self.vertical_block = False
        self.visited = False
        self.step = None

    def get_neighbors(self) -> list:
        neighbors = []
        if self.up and not self.up.obstacle:
            neighbors.append(self.up)
        if self.down and not self.down.obstacle:
            neighbors.append(self.down)
        if self.left and not self.left.obstacle:
            neighbors.append(self.left)
        if self.right and not self.right.obstacle:
            neighbors.append(self.right)

        if not self.vertical_block:
            if self.top and not self.top.obstacle:
                neighbors.append(self.top)
            if self.bottom and not self.bottom.obstacle:
                neighbors.append(self.bottom)

        return neighbors


class GridGraph:
    def __init__(self, tech: Tech, layers: int=7) -> None:
        self.tech = tech
        self.grid3d = []
        self.total_layers = layers      # include poly (index: 0)
        self.get_design_rule(tech)


    def get_design_rule(self, tech: Tech) -> None:
        self.pitch = []
        self.rt_layer2int = {}
        self.int2rt_layer = {}
        self.via_layer2int = {}
        self.int2via_layer = {}

        for layer_idx in range(self.total_layers):
            # poly layer
            if layer_idx == 0:
                min_width = tech.min_width_rule["poly"]
                min_spacing = tech.min_spacing_rule[("poly","poly")]

                self.rt_layer2int["poly"] = layer_idx
                self.int2rt_layer[layer_idx] = "poly"
                self.via_layer2int["contact"] = layer_idx
                self.int2via_layer[layer_idx] = "contact"

            # metal layer
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

            self.pitch.append((min_width + min_spacing)/tech.unit["user"])

        
    def create_grid_graph(self, nets: list, pitch_adjust: int) -> None:
        # flatten the nets
        flatten_nets = []
        for net in nets:
            flatten_nets += net

        # find the boundary of the nets
        x0 = min([pt[0] for pt in flatten_nets])
        x1 = max([pt[0] for pt in flatten_nets])
        y0 = min([pt[1] for pt in flatten_nets])
        y1 = max([pt[1] for pt in flatten_nets])

        # for each metal layer
        for m in range(self.total_layers):
            num_grid_extend = 5                              # extend the number of grid (hardcoded)
            grid_pitch = self.pitch[m] / pitch_adjust        # grid pitch size: the routing pitch / 2 (hardcoded)

            # extend the boundary
            br_x0 = x0 - num_grid_extend * grid_pitch
            br_x1 = x1 + num_grid_extend * grid_pitch
            br_y0 = y0 - num_grid_extend * grid_pitch
            br_y1 = y1 + num_grid_extend * grid_pitch

            # create the nodes
            grid2d = []
            for y in np.arange(br_y0, br_y1, grid_pitch):
                grid = []
                for x in np.arange(br_x0, br_x1, grid_pitch):
                    grid.append(GridNode(x, y, m))
                grid2d.append(grid)
            self.grid3d.append(grid2d)

        # add nodes on the grid if the point is not align 
        self.extend_grid_node(flatten_nets)


    def extend_grid_node(self, flatten_nets: list) -> None:
        # extend nodes on the grid if the point is not align 
        for pt in flatten_nets:
            x_align = False
            y_align = False

            # check if the point is align in x-axis
            for node in self.grid3d[pt[2]][0]:    # 1st row
                if node.x == pt[0]:
                    x_align = True
                    break
            
            # check if the point is align in y-axis
            for row in self.grid3d[pt[2]]:        # all rows
                if row[0].y == pt[1]:   
                    y_align = True
                    break
            
            # add nodes on the grid if the point is not align in x-axis
            if not x_align:
                # find the nearest 2 nodes in the grid (get their index)
                for i, node in enumerate(self.grid3d[pt[2]][0]):    # 1st row
                    if node.x < pt[0]:
                        prev_node_index = i

                    if node.x > pt[0]:
                        next_node_index = i
                        break

                # add the node in between the 2 nodes at each layer
                for m in range(self.total_layers):
                    for row in self.grid3d[m]:                       # go through all rows
                        prev_node = row[prev_node_index]            # get the prev node (same row)
                        curr_node = GridNode(pt[0], prev_node.y, m)
                        curr_node.vertical_block = True             # set the vertical block

                        row.insert(next_node_index, curr_node)      # insert the new node

            # add nodes on the grid if the point is not align in y-axis
            if not y_align:
                # find the nearest 2 rows in the grid (get their index)
                for i, node in enumerate(self.grid3d[pt[2]]):
                    if node[0].y < pt[1]:
                        prev_row_index = i

                    if node[0].y > pt[1]:
                        next_row_index = i
                        break
                
                # add the row in between the 2 rows at each layer
                for m in range(self.total_layers):
                    prev_row = self.grid3d[m][prev_row_index]        # get the prev row
                    curr_row = []
                    for node in prev_row:                           # go through all nodes in the prev row
                        curr_node = GridNode(node.x, pt[1], m)
                        curr_node.vertical_block = True             # set the vertical block
                        curr_row.append(curr_node)

                    self.grid3d[m].insert(next_row_index, curr_row)  # insert the new row


    def get_grid_node(self, coor: tuple) -> GridNode:
        for row in self.grid3d[coor[2]]:
            for node in row:
                if node.x == coor[0] and node.y == coor[1]:
                    return node 


    def grid_connections(self) -> None:
        # connect the nodes
        for lay in range(len(self.grid3d)):
            for row in range(len(self.grid3d[lay])):
                for col in range(len(self.grid3d[lay][row])):
                    # get the current node
                    curr_node: GridNode   = self.grid3d[lay][row][col] 

                    # if the node is an obstacle, skip
                    if curr_node.obstacle:
                        continue

                    # connect the nodes in the x-y plane
                    if row > 0:
                        up_node: GridNode = self.grid3d[lay][row-1][col]
                        curr_node.up = up_node if not up_node.obstacle else None
                    if row < len(self.grid3d[lay])-1:
                        down_node: GridNode = self.grid3d[lay][row+1][col]
                        curr_node.down = down_node if not down_node.obstacle else None
                    if col > 0:
                        left_node: GridNode = self.grid3d[lay][row][col-1]
                        curr_node.left = left_node if not left_node.obstacle else None
                    if col < len(self.grid3d[lay][row])-1:
                        right_node: GridNode = self.grid3d[lay][row][col+1]
                        curr_node.right = right_node if not right_node.obstacle else None

                    # connect the nodes in the z-axis
                    if lay > 0:
                        bottom_node: GridNode = self.grid3d[lay-1][row][col]
                        if not bottom_node.obstacle and not curr_node.vertical_block and curr_node.x == bottom_node.x and curr_node.y == bottom_node.y:
                            curr_node.bottom = bottom_node
                    if lay < 6:
                        top_node: GridNode = self.grid3d[lay+1][row][col]
                        if not top_node.obstacle and not curr_node.vertical_block and curr_node.x == top_node.x and curr_node.y == top_node.y:
                            curr_node.top = top_node


    def plot_grid(self, circuit: Circuit, nets: list, paths: list=[], mrange: tuple=(0,2), title: str=""):
        # shape layout
        for m in range(mrange[0], mrange[1]+1, 1):
            layer = self.int2rt_layer[m]
            # diffusion layer
            for gid in circuit.group:
                for shplayer in circuit.group[gid].shape:
                    if shplayer == "ndiffusion" or shplayer == "pdiffusion":
                        diff_shape = circuit.group[gid].shape[shplayer]
                        for shape in diff_shape:
                            x0 = shape.x[0]/self.tech.unit["user"]
                            x1 = shape.x[1]/self.tech.unit["user"]
                            y0 = shape.y[0]/self.tech.unit["user"]
                            y1 = shape.y[1]/self.tech.unit["user"]
                            plt.subplot(1, mrange[1]+1, m+1)
                            plt.fill_between([x0, x1], y0, y1, color='green', alpha=0.5)

            # port layout
            for pid in circuit.port:
                if layer in circuit.port[pid].shape:
                    shape = circuit.port[pid].shape[layer][0]
                    x0 = shape.x[0]/self.tech.unit["user"]
                    x1 = shape.x[1]/self.tech.unit["user"]
                    y0 = shape.y[0]/self.tech.unit["user"]
                    y1 = shape.y[1]/self.tech.unit["user"]
                    plt.subplot(1, mrange[1]+1, m+1)
                    plt.fill_between([x0, x1], y0, y1, color='yellow', alpha=0.5)

            # pin layout
            for gid in circuit.group:
                for pin in circuit.group[gid].pin:
                    if layer == pin.layer:
                        x0 = pin.pt1[0]/self.tech.unit["user"]
                        x1 = pin.pt2[0]/self.tech.unit["user"]
                        y0 = pin.pt1[1]/self.tech.unit["user"]
                        y1 = pin.pt2[1]/self.tech.unit["user"]
                        plt.subplot(1, mrange[1]+1, m+1)
                        plt.fill_between([x0, x1], y0, y1, color='yellow', alpha=0.5)

        # empty grid
        for m in range(mrange[0], mrange[1]+1, 1):
            x = []
            y = []
            for row in self.grid3d[m]:
                for node in row:
                    if not node.obstacle:
                        x.append(node.x)
                        y.append(node.y)
            plt.subplot(1, mrange[1]+1, m+1)
            plt.scatter(x, y, marker='.', color='black')

        # obstacle
        for m in range(mrange[0], mrange[1]+1, 1):
            x = []
            y = []
            vx = []
            vy = []
            for row in self.grid3d[m]:
                for node in row:
                    if node.obstacle:
                        x.append(node.x)
                        y.append(node.y)

                    if node.vertical_block:
                        vx.append(node.x)
                        vy.append(node.y)

            # planar obstacle
            plt.subplot(1, mrange[1]+1, m+1)
            plt.scatter(x, y, marker='x', color='red')

            # vertical obstacle
            plt.subplot(1, mrange[1]+1, m+1)
            plt.scatter(vx, vy, marker='.', color='magenta')

        # plot grid connection
        # for m in range(mrange[0], mrange[1]+1, 1):
        #     for row in self.grid3d[m]:
        #         for node in row:
        #             plt.subplot(1, mrange[1]+1, m+1)
        #             if node.up:
        #                 plt.plot([node.x, node.up.x], [node.y, node.up.y], color='black')
        #             if node.down:
        #                 plt.plot([node.x, node.down.x], [node.y, node.down.y], color='black')
        #             if node.left:
        #                 plt.plot([node.x, node.left.x], [node.y, node.left.y], color='black')
        #             if node.right:
        #                 plt.plot([node.x, node.right.x], [node.y, node.right.y], color='black')

        # plot the nets
        for net in nets:
            x = []
            y = []
            for pt in net:
                x.append(pt[0])
                y.append(pt[1])
            plt.subplot(1, mrange[1]+1, pt[2]+1)
            plt.scatter(x, y, marker='x', color='blue')

        # plot single path
        # if path is not None:
        #     x = []
        #     y = []
        #     for node in path:
        #         x.append(node.x)
        #         y.append(node.y)
        #     plt.plot(x, y, color='green')

        # plot multiple paths
        if paths is not None:
            for path in paths:
                x = []
                y = []
                z = []
                for node in path:
                    x.append(node.x)
                    y.append(node.y)
                    z.append(node.z)

                # this is for trim path
                # for node in path:
                #     x.append(int(node[0]))
                #     y.append(int(node[1]))
                #     z.append(int(node[2]))
                

                for i in range(len(z)-1):
                    cur = z[i]
                    nxt = z[i+1]

                    if cur > mrange[1] or nxt > mrange[1]:
                        continue
                    
                    if cur == nxt:
                        plt.subplot(1, mrange[1]+1, cur+1)
                        plt.plot(x[i:i+2], y[i:i+2], color='green', marker='x')
                    else:
                        plt.subplot(1, mrange[1]+1, cur+1)
                        plt.plot(x[i], y[i], color='magenta', marker='s')
                        plt.subplot(1, mrange[1]+1, nxt+1)
                        plt.plot(x[i], y[i], color='magenta', marker='s')

        plt.suptitle(title)
        plt.show()
        # plt.savefig(circuit.name+"_"+title+".png")

