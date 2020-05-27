#include "../3rdLibs/glm/glm/glm.hpp"
#include "../3rdLibs/glm/glm/gtc/matrix_transform.hpp"
#include "../3rdLibs/glm/glm/gtx/string_cast.hpp"
#include "../3rdLibs/glm/glm/gtx/norm.hpp"
#include <iostream>
#include <algorithm>

#define PI 3.14159
using namespace std;

/* A Simple Solver for Computer Graphics */


template <typename T> void String_cast(T vec);

void Vertex_in_Camera(glm::vec3 vertex_pos, glm::vec3 Camera_pos, glm::vec3 Target_pos);
void Model_Transform(glm::vec3 vertex_pos, glm::vec3 translation, glm::vec3 scale, glm::vec3 rotate_axis, float radius);
void Phong_Lighting(glm::vec3 View_dir, glm::vec3 Light_dir, glm::vec3 Normal, glm::vec3 Color);
void Solid_Angle(glm::vec3 vertex_pos, glm::vec3 center, float radius);
void Get_Volume(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4);
void Get_Area(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
glm::vec3 Reflection(glm::vec3 input_dir, glm::vec3 normal);
void Get_Circumcircle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
void convert_XYZ_to_rgb(glm::vec3 XYZ);
void convert_rgb_to_XYZ(glm::vec3 RGB);



template <typename T> void String_cast(T vec)
{
	cout << glm::to_string(vec) << std::endl;
}


class Ray_intersection
{
public:
	Ray_intersection(glm::vec3 origin, glm::vec3 end);
	~Ray_intersection();
	bool Triangle_Intersection(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3);
	bool Cylinder_Intersection(glm::vec3 center1, glm::vec3 center2, float r);
	bool Sphere_Intersection(glm::vec3 center, float r);

private:
	glm::vec3 origin;
	glm::vec3 end;
	glm::vec3 dir;
	//bool Torus_Intersection(glm::vec3 center, float R, float r);
};

Ray_intersection::Ray_intersection(glm::vec3 l_origin, glm::vec3 l_end)
{
	origin = l_origin;
	end = l_end;
	dir = glm::normalize(end - origin);
}

Ray_intersection::~Ray_intersection()
{

}


/* To see if the triangle has been intersected */
bool Ray_intersection::Triangle_Intersection(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
	glm::vec3 n = glm::cross(v2 - v1, v3 - v1);
	float d = glm::dot(origin - end, n);
	glm::vec3 e = glm::cross(origin - end, origin - v1);

	if (d == 0)
	{
		cout << " Parallel ! " << endl;
		return false;
	}

	float t = glm::dot(origin - v1, n) / d;
	if (t < 0) return false;

	float v = glm::dot(v3 - v1, e) / d;
	float w = -glm::dot(v2 - v1, e) / d;

	if (v < 0 && w < 0 && v > 1 && w > 1) return false;

	cout << "t:" << t << "intersection points : " << glm::to_string(origin + t * dir) << "end:" << glm::to_string(end) << endl;
	cout << "v" << v << "w" << w << endl;

	return true;
}

/* To see if the Cylinder has been intersected */
bool Ray_intersection::Cylinder_Intersection(glm::vec3 center1, glm::vec3 center2, float r)
{
	glm::vec3 d = center2 - center1;
	glm::vec3 m = origin - center1;
	glm::vec3 n = end - origin;

	if (glm::dot(m, d) < 0 || glm::dot(m + n, d) < 0 || glm::dot(m, d) > glm::dot(d, d) || glm::dot(m + n, d) > glm::dot(d, d))
	{
		cout << " out side " << endl;
		return false;
	}

	float a = glm::dot(glm::cross(d, n), glm::cross(d, n));
	float b = glm::dot(glm::cross(d, m), glm::cross(d, n));
	float c = glm::dot(glm::cross(d, m), glm::cross(d, m)) - pow(r, 2.0f)*glm::dot(d, d);

	float t1 = (-b + sqrtf(pow(b, 2.0f) - a * c)) / a;
	float t2 = (-b - sqrtf(pow(b, 2.0f) - a * c)) / a;

	cout << "t1:" << t1 << " " << "t2:" << t2 << endl;

	return true;
}


/* Check the Sphere */
bool Ray_intersection::Sphere_Intersection(glm::vec3 center, float r)
{
	glm::vec3 d = end - origin;
	glm::vec3 v = origin - center;

	glm::vec3 Vertical = glm::dot(v, d) / glm::dot(d, d) * d;
	if (glm::l2Norm(Vertical) > r)
	{
		cout << " No intersection " << endl;
		return false;
	}

	float a = glm::dot(d, d);
	float b = glm::dot(d, v);
	float c = glm::dot(v, v) - pow(r, 2.0f);


	float t1 = (b + sqrtf(a*c)) / a;
	float t2 = (b - sqrtf(a*c)) / a;

	cout << "t1:" << t1 << " " << "t2:" << t2 << endl;

	return true;
}



/* Given a vertex position in world coordinate, transfer it into camera coordinate system */
void Vertex_in_Camera(glm::vec3 vertex_pos, glm::vec3 Camera_pos, glm::vec3 Target_pos)
{
	glm::vec3 Camera_Dir = glm::normalize(Target_pos - Camera_pos);
	glm::vec3 up = glm::vec3(0, 1, 0);//glm::vec3(1.0f, 1.0f, 0.0f) - Camera_Dir;
	glm::vec3 Right = glm::normalize(glm::cross(up , Camera_Dir));

	//String_cast(Right);

	glm::vec3 Up_vec = glm::normalize(glm::cross(Camera_Dir, Right));
	glm::mat4 transformation = glm::mat4(glm::vec4(Right, 0.0f), glm::vec4(Up_vec, 0.0f), glm::vec4(Camera_Dir, 0.0f), glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
	transformation = glm::transpose(transformation);
	transformation = glm::translate(transformation, glm::vec3(-Camera_pos));
	//glm::mat4 perspective = glm::lookAt(Camera_pos, Target_pos, Up_vec); /* Camera pos, Look-at pos, up Vector */

	//String_cast(transformation);
	glm::vec3 P_cam = glm::vec3(transformation * glm::vec4(vertex_pos, 1));

	cout << "vertex in camera coordinates:";
	String_cast(P_cam);
}

/* Using Model Matrix to transform the vertex ( Radius instead of Degree ) */
void Model_Transform(glm::vec3 vertex_pos, glm::vec3 translation, glm::vec3 scale, glm::vec3 rotate_axis, float radius)
{
	glm::vec3 After_trans = vertex_pos;

	/* Decomposition Part */

	//if (glm::l2Norm(translation) != 0)
	//{
	//	/* Translation here */
	//	glm::mat4 trans = glm::mat4(1.0f);
	//	trans = glm::translate(trans, translation);
	//	After_trans = glm::vec3(trans * glm::vec4(After_trans, 1.0f));

	//	cout << " Translation " << endl;
	//	String_cast(After_trans);
	//}

	//if (glm::l2Norm(rotate_axis) != 0)
	//{
	//	/* Rotate here */
	//	glm::mat4 trans = glm::mat4(1.0f);
	//	//std::cout << radius << std::endl;
	//	trans = glm::rotate(trans, radius, rotate_axis);
	//	String_cast(trans);
	//	After_trans = glm::vec3(trans * glm::vec4(After_trans, 1.0f));

	//	cout << " Rotation " << endl;
	//	String_cast(After_trans);
	//}

	//if (glm::l2Norm(scale) != 0)
	//{
	//	/* Scaling here */
	//	glm::mat4 trans = glm::mat4(1.0f);
	//	trans = glm::scale(trans, scale);

	//	After_trans = glm::vec3(trans * glm::vec4(After_trans, 1.0f));

	//	cout << " Scale " << endl;
	//	String_cast(After_trans);
	//}

	glm::mat4 trans = glm::mat4(1.0f);
	trans = glm::scale(trans, scale);
	trans = glm::rotate(trans, radius, rotate_axis);
	trans = glm::translate(trans, translation);
	
	After_trans = glm::vec3(trans * glm::vec4(After_trans, 1.0f));
	
	//String_cast(trans);
	cout << "vertex in the world:";
	String_cast(After_trans);

	Vertex_in_Camera(After_trans, glm::vec3(8, 0, 0), glm::vec3(4, -3, 5));
}




/* Calculate the Phong lighting model especially focus on the calculation of reflect vector */
/* Light Dir should be Light to the object */
void Phong_Lighting(glm::vec3 View_dir, glm::vec3 Light_dir, glm::vec3 Normal, glm::vec3 Color)
{
	float shininess = 64.0f;
	glm::vec3 ambient_color = glm::vec3(0.1f);

	/* Calculate the diffuse coefficient */
	float diffuse = glm::dot(Normal, -Light_dir);
	glm::vec3 diffuse_color = Color * diffuse;

	/* Calculate the Reflection Vector of specular light */
	
	glm::vec3 reflect = glm::normalize(2 * glm::dot(-Light_dir, Normal) * Normal + Light_dir);
	float specular = pow(max(glm::dot(View_dir, reflect), 0.0f), shininess);
	cout << " Reflect: ";
	String_cast(reflect);

	glm::vec3 specular_color = Color * specular;

	cout << " Color: ";
	String_cast(ambient_color + diffuse + specular);

}

/* Calculate the solid angle for the center vertex , r takes 1 as its value usually*/
void Solid_Angle(glm::vec3 vertex_pos, glm::vec3 center, float r)
{
	float theta = asinf(r / glm::l2Norm(vertex_pos - center));

	/* After integration ... */

	float solid_angle = 2 * PI * (1 + cosf(theta));
	cout << "solid angle :" << solid_angle << endl;
}

/* Calculate the volume of a tetrahedron given 4 coordinates */
void Get_Volume(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4)
{
	glm::mat4 Matrix = glm::mat4(glm::vec4(1, v1), glm::vec4(1, v2), glm::vec4(1, v3), glm::vec4(1, v4));
	//String_cast(Matrix);

	float Volume = 1.0f / 6.0f * glm::determinant(Matrix);

	cout << "Volume is:" << Volume << endl;
}

/* Calculate the area of a triangle */
void Get_Area(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
	float theta = acosf(glm::dot(v2 - v1, v3 - v1) / ( glm::l2Norm(v2 - v1) * glm::l2Norm(v3 - v1)));

	
	float area = glm::l2Norm(v2 - v1) * glm::l2Norm(v3 - v1) * sinf(theta) * 0.5f;
	
	cout << "Aera is:" << area << endl;
}

/* Calculate the reflection vector */
glm::vec3 Reflection(glm::vec3 input_dir, glm::vec3 normal)
{
	glm::vec3 R = input_dir - 2 * (glm::dot(input_dir, normal)) * normal;
	return glm::normalize(R);
}

/* Get the center of circumcicle */
void Get_Circumcircle(glm::vec3 v1, glm::vec3 v2, glm::vec3 v3)
{
	glm::vec3 n, m;
	m = (v1 + v2) * 0.5f;
	n = (v1 + v3) * 0.5f;

	glm::vec3 normal = glm::cross(v1 - v2, v1 - v3);
	glm::vec3 d = glm::normalize(glm::cross(v1 - v2, normal));

	float t = glm::dot(n - m, v1 - v3) / glm::dot(d , v1 - v3);

	glm::vec3 center = m + t * d;

	cout << "The center is ";
	String_cast(center);

}

/* XYZ -> RGB */
void convert_XYZ_to_rgb(glm::vec3 XYZ)
{
	glm::mat3 Conversion = glm::mat3(glm::vec3(0.41847f, -0.091169f, 0.00092090f), glm::vec3(-0.15866f, 0.25243f, -0.0025498f), glm::vec3(-0.082835f, 0.015708f, 0.17860f));
	glm::vec3 RGB = Conversion * XYZ;

	cout << "RGB:";
	String_cast(RGB);
}

/* RGB->XYZ */
void convert_RGB_to_XYZ(glm::vec3 RGB)
{
	glm::mat3 Conversion = glm::mat3(glm::vec3(0.49000f, 0.17697f, 0.0000f), glm::vec3(0.31000f, 0.81240f, 0.010000f), glm::vec3(0.20000f, 0.010630f, 0.99000f));
	glm::vec3 XYZ = Conversion * RGB;
	cout << "XYZ";
	String_cast(XYZ);
}

int main()
{
	/* Given a vertex position in world coordinate, transfer it into camera coordinate system */

	glm::vec3 P , A , B;
	P = glm::vec3(1, 1, 2);
	
	//Vertex_in_Camera(P, glm::vec3(1, 0, 0), glm::vec3(3, 3, 3));
	
	A = glm::vec3(1, 1, 2);
	B = glm::vec3(0, 2, 1);


	/* Using Model Matrix to transform the vertex */
	Model_Transform(A, glm::vec3(1, -1, 1), glm::vec3(2.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f/6.0f * PI); /* Be aware that int / int gets int */
	//Model_Transform(B, glm::vec3(1, -1, 1), glm::vec3(2.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f / 6.0f * PI);
	/* Calculate the Phong lighting model especially focus on the calculation of reflect vector */
	//Phong_Lighting(glm::vec3(1, 0, 0), glm::vec3(0, 1, 1), glm::vec3(0, 0, 1), glm::vec3(1, 1, 1));

	Get_Volume(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	Get_Area(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), glm::vec3(0, 1, 0));

	Get_Circumcircle(glm::vec3(3, 2, 0), glm::vec3(5, 6, 0), glm::vec3(8, 7, 0));


	String_cast(glm::normalize(glm::cross(glm::vec3(0, 1, 0), glm::vec3(-4, -3, 5))));
	system("Pause");
}

