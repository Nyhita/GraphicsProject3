#ifndef BONE_GEOMETRY_H
#define BONE_GEOMETRY_H

#include <ostream>
#include <vector>
#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <mmdadapter.h>

#define BONE_LINE_COUNT 48.0f
#define BONE_RADIUS 0.1f
#define HIGHLIGHT_RADIUS 0.2f

class Ray;

void print_vec(glm::vec3 vec, char* name);
void print_vec(glm::vec4 vec, char* name);
void print_matrix(glm::mat4 matrix);

struct BoundingBox {
	BoundingBox()
		: min(glm::vec3(-std::numeric_limits<float>::max())),
		max(glm::vec3(std::numeric_limits<float>::max())) {}
	glm::vec3 min;
	glm::vec3 max;
};

struct Bone {
private:
	int jid;
	float Length;
	
	glm::vec3 end_joint;
	glm::mat4 Axis;
	glm::mat4 T;
	glm::mat4 R;
	std::vector<Bone*> bone_children;
	Bone* parent_bone;

	int smallest_mag(glm::vec3 my_vector);

public:
	Bone(int _jid, glm::vec3& offset, Bone* parent, glm::mat4 Rs);
	~Bone();
	Bone* findBone(int _jid, glm::mat4& Rs);
	void addBone(int _jid, glm::vec3 offset, glm::mat4& Rs);
	int getJid() {return jid;}
	bool hasParent() {return (parent_bone != NULL);}
	
	void addJointVertices(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, std::string depth);
	
	void generateLine(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs);
	void generateCylinderLines(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, float radius);

	void findBoneIntersect(const Ray& ray, Bone* bone, float& t, glm::mat4 TRS, glm::mat4& TRs);
	void intersectRay(const Ray& ray, Bone* bone, float& t, glm::mat4 TRS, glm::mat4& TRs);

	void generateBoneTRs(glm::mat4& TRs);
};

struct Joint {
	// FIXME: Implement your Joint data structure.
	// Note: PMD represents weights on joints, but you need weights on
	//       bones to calculate the actual animation.
};


struct Skeleton {
	// FIXME: create skeleton and bone data structures
private:
	Bone* bone_root;
	std::vector<glm::vec4> skeleton_vertices;
	std::vector<glm::vec4> cylinder_vertices;
public:
	Skeleton();
	~Skeleton();
	void addBone(int _jid, glm::vec3 offset, int parent);
	void addRootBone(glm::vec3 offset);

	void generateVertices();
	void initCylinderVertices();

	const std::vector<glm::vec4>& getVertices() const;
	const std::vector<glm::vec4>& getCylinderVertices() const;

	std::vector<glm::vec4>& getVerticesVector();

	//void findBoneIntersect(const Ray* ray, Bone* bone, float& t);
	void highlightBones(const Ray& ray);
};

struct Mesh {
	Mesh();
	~Mesh();
	std::vector<glm::vec4> vertices;
	std::vector<glm::vec4> animated_vertices;
	std::vector<glm::uvec3> faces;
	std::vector<glm::vec4> vertex_normals;
	std::vector<glm::vec4> face_normals;
	std::vector<glm::vec2> uv_coordinates;
	std::vector<Material> materials;

	BoundingBox bounds;
	Skeleton skeleton;
	bool isDirty;
	bool show;

	void loadpmd(const std::string& fn);
	void updateAnimation();
	int getNumberOfBones() const 
	{ 
		return skeleton.getVertices().size()/2;
		// FIXME: return number of bones in skeleton
	}
	glm::vec3 getCenter() const { return 0.5f * glm::vec3(bounds.min + bounds.max); }
private:
	void computeBounds();
	void computeNormals();
};

class Ray
{
public:
	glm::vec3 p;
	glm::vec3 v;
};

#endif
