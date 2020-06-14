/*
    This file is part of Nori, a simple educational ray tracer
    Copyright (c) 2015 by Wenzel Jakob
    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.
    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <nori/mesh.h>
#include <Eigen/Geometry>
#include <chrono>  
using namespace std::chrono; 

NORI_NAMESPACE_BEGIN



void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
    //FactorialN start to change things here in Assignment2
    m_ocTree = new Octree(m_mesh, m_bbox);
}

Point3f *get_corners(BoundingBox3f m_bbox){

    Point3f min = m_bbox.min;
    Point3f max = m_bbox.max;
    Point3f *corners = new Point3f[8];
    corners[0]=Point3f(min.x(), min.y(), min.z());
    corners[1]=Point3f(min.x(), max.y(), min.z());
    corners[2]=Point3f(max.x(), min.y(), min.z());
    corners[3]=Point3f(min.x(), min.y(), max.z());
    corners[4]=Point3f(max.x(), max.y(), min.z());
    corners[5]=Point3f(max.x(), min.y(), max.z());
    corners[6]=Point3f(min.x(), max.y(), max.z());
    corners[7]=Point3f(max.x(), max.y(), max.z());
    return corners;


}

void build_aux(Octree *m_octree, uint32_t &root_aux, Mesh *m_mesh, BoundingBox3f m_bbox, uint32_t *tries, uint32_t num_tries, uint32_t depth, uint32_t *num_nodes){
    if(num_tries <= 10 || depth > 50){
        root_aux = ++m_octree->octree_index;
        m_octree->num_all_tries += num_tries;
        m_octree->children[m_octree->octree_index]=OctreeNode(tries, num_tries);
        num_nodes[1]++;
        return;
    }

    root_aux = ++m_octree->octree_index;
    m_octree->children[m_octree->octree_index]=OctreeNode(m_bbox);
    num_nodes[0] ++;


    Point3f center = m_octree->children[root_aux].bbox.getCenter();
    uint32_t *temp_list = new uint32_t[num_tries];
    bool *tries_added_flag = new bool[num_tries];
    bool empty_flag[8];
    int sub_tree_num_tries[8];
    for(uint32_t i = 0; i < 8; i++)
        sub_tree_num_tries[i] = 0;

    Point3f *corners = get_corners(m_bbox);

    for(uint32_t i = 0; i < num_tries; i++) 
        tries_added_flag[i] = false;
    for(uint32_t i = 0; i < 8; i++){
        BoundingBox3f result(center);
        result.expandBy(corners[i]);
        BoundingBox3f sub_bbox=result;
        // c[0]=0;
        for(uint32_t idx = 0; idx < num_tries; idx++){
            if(tries_added_flag[idx])continue;
            BoundingBox3f tri_bbox = m_mesh->getBoundingBox(tries[idx]);
            if(result.overlaps(tri_bbox)){
                empty_flag[i] = true;
                temp_list[sub_tree_num_tries[i]] = tries[idx];
                sub_tree_num_tries[i]++;
                sub_bbox.expandBy(tri_bbox);
                tries_added_flag[idx] = true;
            }
        }
        if(empty_flag[i]){
            uint32_t *sub_tries = new uint32_t[sub_tree_num_tries[i]];
            for(uint32_t idx = 0; idx < sub_tree_num_tries[i]; idx++) sub_tries[idx] = temp_list[idx];
            build_aux(m_octree, m_octree->children[root_aux].children[i], m_mesh, sub_bbox, sub_tries, sub_tree_num_tries[i], depth + 1, num_nodes);
        }
    }

}

bool rayIntersect_aux(Octree *m_octree, uint32_t root_aux, Ray3f &ray, Intersection &its, bool shadowRay, Mesh *m_mesh, uint32_t &f, bool &foundIntersection){
    std::pair<float,uint32_t>srt[10];
    uint32_t inTot=0;
    for(uint32_t i = 0; i < 8; i++){
        if(m_octree->children[root_aux].children[i] != 0){
            float near, far;
            bool intersect = m_octree->children[m_octree->children[root_aux].children[i]].bbox.rayIntersect(ray,near,far);
            if(intersect)srt[inTot++]=std::pair<float,uint32_t>(near,i);
        }
    }
    std::sort(srt,srt+inTot);
    for(uint32_t i = 0; i < inTot; i++)
        if(m_octree->children[root_aux].children[srt[i].second] != 0){
            if(ray.maxt < srt[i].first)break;
            bool tg = rayIntersect_aux(m_octree, m_octree->children[root_aux].children[srt[i].second], ray, its, shadowRay, m_mesh, f, foundIntersection);
            if(shadowRay&&tg)return true;
        }
    if(m_octree->children[root_aux].num_tries){
        for(uint32_t idx = 0; idx < m_octree->children[root_aux].num_tries; idx++){
            float u, v, t;
            if (m_mesh->rayIntersect(m_octree->children[root_aux].triangles[idx], ray, u, v, t)) {
                if (shadowRay)
                    return true;

                ray.maxt = its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_mesh;
                f = m_octree->children[root_aux].triangles[idx];
                foundIntersection = true;
            }
        }
    }
    return false;
}


void Accel::build() {
    uint32_t *num_nodes;
    num_nodes = new uint32_t[2];
    num_nodes[0]=0; num_nodes[1]=0;
    using namespace std::chrono; 
    auto start = high_resolution_clock::now();
    build_aux(m_ocTree, m_ocTree->root, m_mesh, m_mesh->getBoundingBox(), m_ocTree->triangles, m_mesh->getTriangleCount(), 1, num_nodes);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    
    cout << "Nomber of interior nodes: " << num_nodes[0] << "\n";
    cout << "Nomber of leaf nodes: " << num_nodes[1] << "\n";
    cout << duration.count() << " microseconds to bulid the octree\n";
}


bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Brute force search through all triangles */
    /*
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
             An intersection was found! Can terminate
               immediately if this is a shadow ray query 
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }*/
    // FactorialN: Add Octree using here.
    //cerr<<"F";


    bool intersect = rayIntersect_aux(m_ocTree, m_ocTree->root, ray, its, shadowRay, m_mesh, f, foundIntersection);
    if(intersect)return true;


    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.
           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END