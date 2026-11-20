import open3d as o3d
import glob
import os

mesh_folder = "../meshes"

for path in glob.glob(os.path.join(mesh_folder, "*.stl")):
    mesh = o3d.io.read_triangle_mesh(path)
    num_triangles = len(mesh.triangles)
    
    if num_triangles > 20000:
        print(f"Upraszczanie {os.path.basename(path)}: z {num_triangles} do 10 000 trójkątów...")
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=10000)
        # Obliczenie wektorów normalnych, aby siatka ładnie wyglądała
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(path, mesh)

print("Gotowe!")