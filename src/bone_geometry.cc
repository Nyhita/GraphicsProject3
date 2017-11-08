#include "config.h"
#include "bone_geometry.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <glm/gtx/io.hpp>
#include <glm/gtx/transform.hpp>

void quadraticFormula(double a, double b, double c, double *totalResults);

void print_vec(glm::vec3 vec, char* name)
{
	std::cout << name << ": " << vec.x << " " << vec.y << " " << vec.z << "\n";
}

void print_vec(glm::vec4 vec, char* name)
{
	std::cout << name << ": " << vec.x << " " << vec.y << " " << vec.z << "\n";
}

void print_matrix(glm::mat4 matrix)
{
	std::cout << "|" << matrix[0].x << " " << matrix[1].x << " " << matrix[2].x << " " << matrix[3].x << "|\n";
	std::cout << "|" << matrix[0].y << " " << matrix[1].y << " " << matrix[2].y << " " << matrix[3].y << "|\n";
	std::cout << "|" << matrix[0].z << " " << matrix[1].z << " " << matrix[2].z << " " << matrix[3].z << "|\n";
	std::cout << "|" << matrix[0].w << " " << matrix[1].w << " " << matrix[2].w << " " << matrix[3].w << "|\n";
}

/*
 * For debugging purpose.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
	size_t count = std::min(v.size(), static_cast<size_t>(10));
	for (size_t i = 0; i < count; ++i) os << i << " " << v[i] << "\n";
	os << "size = " << v.size() << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& bounds)
{
	os << "min = " << bounds.min << " max = " << bounds.max;
	return os;
}

Skeleton::Skeleton(): bone_root(NULL) 
{

}

Skeleton::~Skeleton()
{
	delete bone_root;
}

void Skeleton::addRootBone(glm::vec3 offset)
{
	bone_root = new Bone(0, offset, NULL);
}

void Skeleton::generateVertices()
{
	skeleton_vertices.clear();
	bone_root->addJointVertices(skeleton_vertices, glm::mat4(1.0f), "\t");
	skeleton_vertices.push_back(glm::vec4(4.0f,8.0f,-1.0f,1.0f));
	skeleton_vertices.push_back(glm::vec4(6.0f,8.0f,-1.0f,1.0f));
}

void Skeleton::addBone(int _jid, glm::vec3 offset, int parent)
{
	glm::mat4 TRs(1.0f);
	Bone* parent_bone = bone_root->findBone(parent, TRs);
	parent_bone->addBone(_jid, offset);
}

const std::vector<glm::vec4>& Skeleton::getVertices() const
{
	return skeleton_vertices;
}

std::vector<glm::vec4>& Skeleton::getVerticesVector()
{
	return skeleton_vertices;
}

void Skeleton::initVertices(std::vector<glm::vec4>& vertices, int size)
{
	for(int i = 0; i < size; ++i)
		vertices.push_back(glm::vec4(0.0f,0.0f,0.0f,0.0f));	
}

void Skeleton::initCylinderVertices()
{
	initVertices(cylinder_vertices, BONE_LINE_COUNT*2);;
}

void Skeleton::initNormalVertices()
{
	initVertices(normal_vertices, 2);
}

void Skeleton::initBinormalVertices()
{
	initVertices(binormal_vertices, 2);
}


const std::vector<glm::vec4>& Skeleton::getCylinderVertices() const
{
	return cylinder_vertices;
}

std::vector<glm::vec4>& Skeleton::getCylinderVerticesVector()
{
	return cylinder_vertices;
}

const std::vector<glm::vec4>& Skeleton::getNormalVertices() const
{
	return normal_vertices;
}

const std::vector<glm::vec4>& Skeleton::getBinormalVertices() const
{
	return binormal_vertices;
}

void Skeleton::highlightBones(const Ray& ray)
{
	cylinder_vertices.clear();
	normal_vertices.clear();
	binormal_vertices.clear();

	glm::mat4 Trs = glm::mat4(1.0f);
	Bone* reference_bone = bone_root->findBone(TEST_BONE, Trs);
	Ray localRay;
	reference_bone->createLocalRay(localRay, ray, Trs);
	glm::vec3 q = localRay.p + 100000.0f * localRay.v;
	skeleton_vertices[skeleton_vertices.size()-2] = glm::vec4(localRay.p.x, localRay.p.y, localRay.p.z, 1.0f);
	skeleton_vertices[skeleton_vertices.size()-1] = glm::vec4(q.x, q.y, q.z, 1.0f);

	float t = std::numeric_limits<float>::max();
	Bone* highlight_bone = NULL;
	glm::mat4 TRs(1.0f);

	bone_root->findBoneIntersect(ray, highlight_bone, t, glm::mat4(1.0f), TRs, skeleton_vertices);
	std::cout << "highlight_bone is NULL" << (highlight_bone == NULL) << "\n";

	if(highlight_bone != NULL)
	{
		std::cout << "bone found: " << highlight_bone->getJid() << "\n";
		highlight_bone->boneScan();
		highlight_bone->generateCylinderLines(cylinder_vertices, TRs, HIGHLIGHT_RADIUS);
		//highlight_bone->generateAxis(cylinder_vertices, TRs, HIGHLIGHT_RADIUS);
		highlight_bone->generateNormalAxis(normal_vertices, TRs, AXIS_LENGTH);
		highlight_bone->generateBinormalAxis(binormal_vertices, TRs, AXIS_LENGTH);
	}
}

// Inspired from http://www.cplusplus.com/forum/beginner/9735/
bool quadraticFormula(float a, float b, float c, float& t0, float& t1)
{
	float numeratorResult = (b*b) - (4*a*c);
	if(numeratorResult <= 0.0f)
	{
		return false;
	}

	float numeratorSqrtResult = sqrt(numeratorResult);

	float denominatorResult = 2 * a;
	float addNegativeTo_b = 0 - b;

	if (denominatorResult == 0)
	{
		return false;
	}
	
	t0 = (addNegativeTo_b + numeratorSqrtResult)/denominatorResult;
	t1 = (addNegativeTo_b - numeratorSqrtResult)/denominatorResult;

	return true;
}

// Call this recursively from parent first
void Bone::generateBoneTRs(glm::mat4& TRs)
{
	TRs = T * R * TRs;
	if(parent_bone != NULL)
		parent_bone->generateBoneTRs(TRs);
}

void Bone::createLocalRay(Ray& localRay, const Ray& ray, glm::mat4 TRS)
{
	glm::vec3 q = ray.p + ray.v;

	glm::vec4 ray_p4 = glm::vec4(ray.p.x, ray.p.y, ray.p.z, 1.0f);
	//glm::vec4 ray_v4 = glm::vec4(ray.v.x, ray.v.y, ray.v.z, 1.0f);
	glm::vec4 ray_q4 = glm::vec4(q.x, q.y, q.z, 1.0f);

	glm::vec4 localRay_p4 = glm::inverse(TRS * T) * ray_p4;
	glm::vec4 localRay_q4 = glm::inverse(TRS * T) * ray_q4;
	
	//glm::vec4 localRay_v4 = glm::inverse(Axis) * ray_v4;
	glm::vec4 localRay_v4 = localRay_q4 - localRay_p4;

	localRay.p = glm::vec3(localRay_p4.x, localRay_p4.y, localRay_p4.z);
	localRay.v = glm::vec3(localRay_v4.x, localRay_v4.y, localRay_v4.z);
}

void Bone::intersectRay(const Ray& ray, Bone*& bone, float& t, glm::mat4 TRS, glm::mat4& TRs, std::vector<glm::vec4>& skeleton_vertices)
{
	// Convert ray to local coordinates
	Ray localRay;

	createLocalRay(localRay, ray, TRS);

	float tx0 = 0.0f;
	float tx1 = 0.0f;
	float tyz0 = 0.0f;
	float tyz1 = 0.0f;

	if(localRay.v.x == 0.0f)
		return;

	//std::cout << "5\n";
	
	// Solve equation 0 <= ray.x <= L or 0 <= r.p.x + t*r.v.x <= L
	tx0 = -1.0f * (localRay.p.x / localRay.v.x);
	tx1 = (Length - localRay.p.x) / localRay.v.x;

	//std::cout << "6\n";

	// d.y^2 + d.z^2 or (ray.v.y)^2 + (ray.v.z)^2
	float a = (localRay.v.y)*(localRay.v.y) + (localRay.v.z)*(localRay.v.z);
	
	// 2*p.y*d.y + 2*p.z*d.z or 2*ray.p.y*ray.v.y + 2*ray.p.z*ray.v.z
	float b = 2.0f*localRay.p.y*localRay.v.y + 2.0f*localRay.p.z*localRay.v.z;

	// p.y^2 + p.z^2 - R^2 or ray.p.y^2 + ray.p.z^2 - BONE_RADIUS^2
	float c = (localRay.p.y)*(localRay.p.y) + (localRay.p.z)*(localRay.p.z) - (BONE_RADIUS)*(BONE_RADIUS);

	//std::cout << "7\n";

	// Solve the equation y^2 + z^2 <= r^2 and find the range of t
	// Equation is also (ray.y^2 + ray.z^2 <= r^2)
	if(!quadraticFormula(a, b, c, tyz0, tyz1))
		return;
	//std::cout << "8\n";

	float tx_s = std::min(tx0, tx1);
	float tx_e = std::max(tx0, tx1);

	float tyz_s = std::min(tyz0, tyz1);
	float tyz_e = std::max(tyz0, tyz1);

	//std::cout << "9\n";

	if(tx_e < 0.0f || tyz_e < 0.0f)
		return;

	//std::cout << "10\n";

	tx_s = std::max(tx_s, 0.0f);
	tyz_s = std::max(tyz_s, 0.0f);

	float t1_start = 0.0f;
	float t1_end = 0.0f;
	float t2_start = 0.0f;
	float t2_end = 0.0f;

	//std::cout << "11\n";

	if(tx_s < tyz_s)
	{
		t1_start = tx_s;
		t1_end = tx_e;

		t2_start = tyz_s;
		t2_end = tyz_e;
	} 
	else 
	{
		t1_start = tyz_s;
		t1_end = tyz_e;

		t2_start = tx_s;
		t2_end = tx_e;
	}

	//std::cout << "12\n";

	// If there's no t that's in between both ranges to satisfy, return
	if(t2_start > t1_end)
		return;

	std::cout << "13: " << "t2_start" << t2_start << " t: " << t << "\n";

	// t2_start is now the t_start
	if(t2_start < t)
	{
		std::cout << "we in der" << "\n";
		t = t2_start;
		bone = (this);
		TRs = TRS;
		std::cout << "bone is NULL? " << (bone == NULL) << "\n";
	}

	//std::cout << "14\n";
}

void Bone::findBoneIntersect(const Ray& ray, Bone*& bone, float& t, glm::mat4 TRS, glm::mat4& TRs, std::vector<glm::vec4>& skeleton_vertices)
{
	if(jid != 0)
	{
		intersectRay(ray, bone, t, TRS, TRs, skeleton_vertices);
		//if(bone != NULL)
		//{
		//	std::cout << "Bone isn't null\n";
		//}
	}

	glm::mat4 new_TR = TRS * T * R;

	typedef std::vector<Bone*>::const_iterator iter;
	for(iter i = bone_children.begin(); i != bone_children.end(); ++i) 
	{
		Bone* bone_child = (*i);
		bone_child->findBoneIntersect(ray, bone, t, new_TR, TRs, skeleton_vertices);
	}	
}

void Bone::generateLine(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs)
{
	glm::vec4 start_joint = TRs * T * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 end_joint = TRs * T * R * glm::vec4(Length, 0.0f, 0.0f, 1.0f);

	skeleton_vertices.push_back(start_joint);
	skeleton_vertices.push_back(end_joint);
}

void Bone::generateCylinderLines(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, float radius)
{
	//float radius = 0.1f;
	float angle;
	for(angle = 0.0f; angle < (2.0f*M_PI); angle += 2.0f*M_PI/BONE_LINE_COUNT)
	{
		glm::vec4 start_joint = TRs * T * glm::vec4(0.0f, radius*glm::cos(angle), radius*glm::sin(angle), 1.0f);
		glm::vec4 end_joint = TRs * T * R * glm::vec4(Length, radius*glm::cos(angle), radius*glm::sin(angle), 1.0f);

		skeleton_vertices.push_back(start_joint);
		skeleton_vertices.push_back(end_joint);
	}
}

void Bone::generateCylinderLinesRaw(std::vector<glm::vec4>& skeleton_vertices, float radius)
{
	//float radius = 0.1f;
	float angle;
	for(angle = 0.0f; angle < (2.0f*M_PI); angle += 2.0f*M_PI/BONE_LINE_COUNT)
	{
		glm::vec4 start_joint = glm::vec4(0.0f, radius*glm::cos(angle), radius*glm::sin(angle), 1.0f);
		glm::vec4 end_joint = glm::vec4(Length, radius*glm::cos(angle), radius*glm::sin(angle), 1.0f);

		skeleton_vertices.push_back(start_joint);
		skeleton_vertices.push_back(end_joint);
	}
}

void Bone::generateAxis(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, float length)
{
	glm::vec4 origin = TRs * T * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 y_coord = TRs * T * R * glm::vec4(0.0f, 5.0f, 0.0f, 1.0f);
	glm::vec4 z_coord = TRs * T * R * glm::vec4(0.0f, 0.0f, 5.0f, 1.0f);

	skeleton_vertices.push_back(origin);
	skeleton_vertices.push_back(y_coord);
	skeleton_vertices.push_back(origin);
	skeleton_vertices.push_back(z_coord);
}

void Bone::generateNormalAxis(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, float length)
{
	glm::vec4 origin = TRs * T * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 y_coord = TRs * T * R * glm::vec4(0.0f, length, 0.0f, 1.0f);

	skeleton_vertices.push_back(origin);
	skeleton_vertices.push_back(y_coord);
}

void Bone::generateBinormalAxis(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, float length)
{
	glm::vec4 origin = TRs * T * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 z_coord = TRs * T * R * glm::vec4(0.0f, 0.0f, length, 1.0f);

	skeleton_vertices.push_back(origin);
	skeleton_vertices.push_back(z_coord);
}

void Bone::addJointVertices(std::vector<glm::vec4>& skeleton_vertices, glm::mat4 TRs, std::string depth)
{
	if(jid != 0)
	{
		//generateLine(skeleton_vertices, TRs);
		generateCylinderLines(skeleton_vertices, TRs, BONE_RADIUS);
	}

	glm::mat4 new_TR = TRs * T * R;

	typedef std::vector<Bone*>::const_iterator iter;
	for(iter i = bone_children.begin(); i != bone_children.end(); ++i) 
	{
		Bone* bone_child = (*i);
		bone_child->addJointVertices(skeleton_vertices, new_TR, depth + "\t");
	}

}

void Bone::addBone(int _jid, glm::vec3 offset)
{
	// std::cout << "Parent bone jid: " << (this)->jid << "\n";
	bone_children.push_back(new Bone(_jid, offset, this));
}

Bone* Bone::findBone(int _jid, glm::mat4& TRs)
{
	if(jid == _jid)
	{
		return (this);
	}

	glm::mat4 TRstart = TRs;
	TRs = TRstart * T * R;

	typedef std::vector<Bone*>::const_iterator iter;
	for(iter i = bone_children.begin(); i != bone_children.end(); ++i) 
	{
		Bone* bone_child = (*i);
		Bone* search_bone = bone_child->findBone(_jid, TRs);
		if(search_bone != NULL)
			return search_bone;
	}

	TRs = TRstart;

	return NULL;
}

int Bone::smallest_mag(glm::vec3 my_vector)
{
	float min_val = std::abs(my_vector[0]);
	int pos = 0;

	for(int i = 1; i < 3; ++i)
	{
		if(std::abs(my_vector[i]) < min_val)
		{
			min_val = my_vector[i];
			pos = i;
		}
	}

	return pos;
}

// Generate bones from a joint end and find the joint start from the parent
// For the imaginary root bone (from origin to 1st start point of real root), the joint start is the origin
Bone::Bone(int _jid, glm::vec3& offset, Bone* parent)
{
	// Set parent
	if(parent != NULL)
		parent_bone = parent;

	// Calculate the bone vector
	glm::vec3 bone_vector = offset;

	// Generate jid
	jid = _jid;

	// Generate length
	Length = glm::length(bone_vector);

	// Set up endpoint of parent ==> Use transforms to generate start point
	end_joint = offset; 

	// Generate Axis
	glm::vec3 t;
	glm::vec3 n;
	glm::vec3 b;

		// t-axis
	t = glm::normalize(bone_vector);

		// n-axis
	glm::vec3 v(0.0f, 0.0f, 0.0f);
	int axis = smallest_mag(t);
	v[axis] = 1.0f;
	n = glm::normalize(glm::cross(t, v));

		// b-axis
	b = glm::normalize(glm::cross(t, n));

	// Generate axis matrix
	Axis = glm::mat4(1.0f);

	if(parent != NULL)
	{
		Axis[0] = glm::vec4(t.x, t.y, t.z, 0.0f);
		Axis[1] = glm::vec4(n.x, n.y, n.z, 0.0f);
		Axis[2] = glm::vec4(b.x, b.y, b.z, 0.0f);	
	}

	// Generate T (relative to parent axis)
	T = glm::mat4(1.0f);

	// If we're based on world coordinates, the offset is the offset of the 1st joint
	if(parent != NULL)
	{
		if(parent->jid == 0)
		{
			T[3].x = parent->end_joint.x;
			T[3].y = parent->end_joint.y;
			T[3].z = parent->end_joint.z;
		} 
		else 
		{
			T[3].x = parent->Length;
			T[3].y = 0.0f;
			T[3].z = 0.0f;
		}
	}

	// Generate R which is R = (Rs)-1*Axis
	R = glm::mat4(1.0f);
	if(parent != NULL)
	{
		R = glm::inverse(parent->Axis)*Axis;
	}

}

Bone::~Bone()
{
	// [TODO] Use iterator to delete all bones
	typedef std::vector<Bone*>::const_iterator iter;
	for(iter i = bone_children.begin(); i != bone_children.end(); ++i) 
	{
		delete (*i);
	}
}

void Bone::boneScan()
{
	std::cout << "Bone: " << jid << "\n";
	std::cout << "Bone length: " << Length << "\n";
	std::cout << "T Matrix:\n";
	print_matrix(T);
	std::cout << "R Matrix:\n";
	print_matrix(R);
	std::cout << "Axis Matrix:\n";
	print_matrix(Axis);
}

// FIXME: Implement bone animation.


Mesh::Mesh()
{
	show = false;
}

Mesh::~Mesh()
{

}

void Mesh::loadpmd(const std::string& fn)
{
	MMDReader mr;
	mr.open(fn);
	mr.getMesh(vertices, faces, vertex_normals, uv_coordinates);
	computeBounds();
	mr.getMaterial(materials);

	// FIXME: load skeleton and blend weights from PMD file
	//        also initialize the skeleton as needed
	
	// Values to pass by reference
	glm::vec3 offset;
	int parent;

	// Set up the skeleton with the imaginary root bone
	mr.getJoint(0, offset, parent);
	skeleton.addRootBone(offset);

	int i = 1;

	// Use the information obtained from getJoint to add a bone
	
	while(mr.getJoint(i, offset, parent))
		skeleton.addBone(i++, offset, parent);

	skeleton.generateVertices();

	skeleton.initCylinderVertices();
	skeleton.initNormalVertices();
	skeleton.initBinormalVertices();

	// New way to draw bones as test
	//skeleton.getVerticesVector().clear();

	i = 1;
	//while(mr.getJoint(i, offset, parent))
	//{	
		glm::mat4 TRs = glm::mat4(1.0f);
		Bone* reference_bone = skeleton.getBoneRoot()->findBone(TEST_BONE, TRs);
		reference_bone->generateCylinderLinesRaw(testVertices, BONE_RADIUS);
		//reference_bone->generateCylinderLinesRaw(skeleton.getCylinderVerticesVector(), BONE_RADIUS);
	//}

	isDirty = true;
}

void Mesh::updateAnimation()
{
	animated_vertices = vertices;
	// FIXME: blend the vertices to animated_vertices, rather than copy
	//        the data directly.
}


void Mesh::computeBounds()
{
	bounds.min = glm::vec3(std::numeric_limits<float>::max());
	bounds.max = glm::vec3(-std::numeric_limits<float>::max());
	for (const auto& vert : vertices) {
		bounds.min = glm::min(glm::vec3(vert), bounds.min);
		bounds.max = glm::max(glm::vec3(vert), bounds.max);
	}
}

