#ifndef TSDF_PLUSPLUS_UTILS_CONVERSIONS_H_
#define TSDF_PLUSPLUS_UTILS_CONVERSIONS_H_

#include <voxblox/io/sdf_ply.h>

#include "tsdf_plusplus/core/common.h"

inline void convertMeshToPCLPointcloud(
    const voxblox::Mesh& mesh, pcl::PointCloud<PointTypeNormal>* pcl_cloud) {
  pcl_cloud->reserve(mesh.vertices.size());

  for (size_t vert_idx = 0u; vert_idx < mesh.vertices.size(); ++vert_idx) {
    PointTypeNormal point_pcl;

    const voxblox::Point& point = mesh.vertices[vert_idx];
    point_pcl.x = point[0];
    point_pcl.y = point[1];
    point_pcl.z = point[2];

    if (mesh.hasColors()) {
      const voxblox::Color& color = mesh.colors[vert_idx];
      point_pcl.r = static_cast<int>(color.r);
      point_pcl.g = static_cast<int>(color.g);
      point_pcl.b = static_cast<int>(color.b);
    }

    if (mesh.hasNormals()) {
      const voxblox::Point& normal = mesh.normals[vert_idx];
      point_pcl.normal_x = normal(0);
      point_pcl.normal_y = normal(1);
      point_pcl.normal_z = normal(2);
    }

    pcl_cloud->push_back(point_pcl);
  }

  pcl_cloud->is_dense = true;
  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1u;
}

inline void convertVoxelGridToPointCloud(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_voxels,
    const voxblox::MeshIntegratorConfig& mesh_config,
    pcl::PointCloud<PointTypeNormal>* pcl_cloud,
    const bool connected_mesh = true,
    const voxblox::FloatingPoint vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(pcl_cloud);

  voxblox::Mesh mesh;
  voxblox::io::convertLayerToMesh(tsdf_voxels, mesh_config, &mesh,
                                  connected_mesh, vertex_proximity_threshold);

  convertMeshToPCLPointcloud(mesh, pcl_cloud);
}

inline void convertMeshLayerToPCLPolygonMesh(
    const voxblox::MeshLayer& mesh_layer, pcl::PolygonMesh* polygon_mesh_ptr,
    const float vertex_proximity_threshold = 1e-10) {
  CHECK_NOTNULL(polygon_mesh_ptr);

  // Constructing the vertices pointcloud.
  pcl::PointCloud<PointTypeNormal> pcl_cloud;
  std::vector<pcl::Vertices> polygons;

  voxblox::Mesh mesh;
  mesh_layer.getMesh(&mesh);

  // Add points.
  convertMeshToPCLPointcloud(mesh, &pcl_cloud);

  // Add triangles.
  pcl::Vertices vertices_idx;
  polygons.reserve(mesh.indices.size() / 3);
  for (const voxblox::VertexIndex& idx : mesh.indices) {
    vertices_idx.vertices.push_back(idx);

    if (vertices_idx.vertices.size() == 3) {
      polygons.push_back(vertices_idx);
      vertices_idx.vertices.clear();
    }
  }

  // Converting to the pointcloud binary.
  pcl::PCLPointCloud2 pointcloud2;
  pcl::toPCLPointCloud2(pcl_cloud, pointcloud2);

  // Filling the mesh.
  polygon_mesh_ptr->cloud = pointcloud2;
  polygon_mesh_ptr->polygons = polygons;
}

#endif  // TSDF_PLUSPLUS_UTILS_CONVERSIONS_H_
