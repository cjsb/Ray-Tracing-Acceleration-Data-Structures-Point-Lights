
/**
 * There are two classes in this file. One is for defing the octree base
 * node and another class for definig Octree data structure.
 * 
 * Auther: Hossein Souri - PhD studet at University of Marlyland
 * Department of Computer Scinece
 * email: hsouri@umiacs.umd.edu
 * 
 */

#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN


class OctreeNode{
public:

    OctreeNode(){
        num_tries = 1;
        triangles=new uint32_t[1];
        triangles[0]=1;
    }
    OctreeNode(const OctreeNode &octree_node){
        /* Copy Constructor */
        for(uint32_t index = 0; index < 8; index++)children[index] = octree_node.children[index];
        triangles = new uint32_t[octree_node.num_tries];
        for(uint32_t i = 0; i < octree_node.num_tries; i++)triangles[i] = octree_node.triangles[i];
        num_tries = octree_node.num_tries;
        bbox = octree_node.bbox;
    }
    OctreeNode(uint32_t *_triangles, const uint32_t &_num_tries){
        for(uint32_t i = 0; i < 8; i++)children[i] = 0;
        triangles = new uint32_t[_num_tries];
        for(uint32_t i = 0; i < _num_tries; i++)triangles[i] = _triangles[i];
        num_tries = _num_tries;
    }
    OctreeNode(BoundingBox3f &m_bbox){
        for(uint32_t index = 0; index < 8; index++)children[index] = 0;
        bbox = m_bbox;
        num_tries = 0;
    }
    ~OctreeNode(){
        
    }
    // Variables
    uint32_t children[8];
    BoundingBox3f bbox;
    uint32_t *triangles;
    uint32_t num_tries;
};


class Octree{
public: 
    Octree(Mesh *m_mesh, BoundingBox3f &m_bbox){

        octree_index=0;
        num_all_tries=0;
        uint32_t total_tris = m_mesh->getTriangleCount();
        triangles = new uint32_t[total_tris];
        for(uint32_t i=0; i<total_tris; i++)
            triangles[i]=i;
        children = new OctreeNode[total_tris];
    }
    ~Octree(){
    }

    OctreeNode *children;
    uint32_t octree_index;
    uint32_t num_all_tries;
    uint32_t root;
    uint32_t *triangles;
};

NORI_NAMESPACE_END
