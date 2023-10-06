import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import bpy

import os

from bpy.props import StringProperty, BoolProperty
from bpy_extras.io_utils import ImportHelper
from bpy.types import Operator

def main(context):
    
    #create paths and load data
    inputfile = bpy.data.objects["Empty"].point_cloud_visualizer.filepath
#    input_path="C:/Users/user_name/Desktop/"
#    dataname="TLS_kitchen.ply"

#    pcd = o3d.io.read_point_cloud(input_path+dataname)
    pcd = o3d.io.read_point_cloud(inputfile)

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    pcd.paint_uniform_color([0.6, 0.6, 0.6])

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    o3d.visualization.draw_geometries([pcd])

    segment_models={}
    segments={}
    max_plane_idx=10
    rest=pcd
    for i in range(max_plane_idx):
        colors = plt.get_cmap("tab20")(i)
        segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        segments[i]=rest.select_by_index(inliers)
        segments[i].paint_uniform_color(list(colors[:3]))
        rest = rest.select_by_index(inliers, invert=True)
        print("pass",i,"/",max_plane_idx,"done.")

    o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])

    segment_models={}
    segments={}
    max_plane_idx=20
    rest=pcd
    d_threshold=0.01
    for i in range(max_plane_idx):
        colors = plt.get_cmap("tab20")(i)
        segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        segments[i]=rest.select_by_index(inliers)
        labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*10, min_points=10))
        candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
        best_candidate=int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])
        print("the best candidate is: ", best_candidate)
        rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
        segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
        segments[i].paint_uniform_color(list(colors[:3]))
        print("pass",i+1,"/",max_plane_idx,"done.")

    labels = np.array(rest.cluster_dbscan(eps=0.05, min_points=5))
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    rest.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    #o3d.visualization.draw_geometries([rest])
#    print(segments.values())
    #o3d.visualization.draw_geometries([segments.values()])
    #o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])
#    final_p = segments
#    ply = o3d.geometry.PointCloud()
#    ply.points = o3d.utility.Vector3dVector(segments[i] for i in range(max_plane_idx))
#    o3d.io.write_point_cloud("C:/Users/gokug/Desktop/copy_of_fragment.ply", ply)
#    print("File Saved")


    for i in range (max_plane_idx):
        file_name = "G:\segment" + str(i) + ".ply"
        file_export =  o3d.geometry.PointCloud()
        file_export = segments[i]
        o3d.io.write_point_cloud(file_name,file_export,write_ascii = False)       
    
    o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest],zoom=0.3199,front=[0.30159062875123849, 0.94077325609922868, 0.15488309545553303],lookat=[-3.9559999108314514, -0.055000066757202148, -0.27599999308586121],up=[-0.044411423633999815, -0.138726419067636, 0.98753122516983349])
    
    

    # from mpl_toolkits import mplot3d

    # pc=np.asarray(pcd.points)
    # ax = plt.axes(projection='3d')
    # ax.scatter(pc[:,0], pc[:,1], pc[:,2], c = np.asarray(pcd.colors), s=0.01)
    # plt.show()
        
    
class Auto_Segment(bpy.types.Operator):
    bl_idname = "pcs.auto_segment"
    bl_label = "Auto Segmentor"
    
    def execute(self,context):
#        inputfile = OT_TestOpenFilebrowser.ply_file
#        main(inputfile,context)
        main(context)
        return {'FINISHED'}

class LayoutDemoPanel(bpy.types.Panel):
    """Creates a Panel in the scene context of the properties editor"""
    bl_label = "Auto Segmentor"
    bl_idname = "SCENE_PT_layout"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"
    def draw(self, context):
        layout = self.layout

        scene = context.scene
        
         # Big render button
        layout.label(text="Auto Segmentor")
        row = layout.row()
        row.scale_y = 1.0
        row.operator("pcs.auto_segment")
        
def register():
    bpy.utils.register_class(Auto_Segment)
    bpy.utils.register_class(LayoutDemoPanel)


def unregister():
    bpy.utils.unregister_class(Auto_Segment)
    bpy.utils.unregister_class(LayoutDemoPanel)


if __name__ == "__main__":
    register()
