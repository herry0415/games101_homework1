//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <Eigen>


using namespace Eigen;
class Triangle{

public:
    Vector3f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
    /*Per vertex values*/ //每个顶点的值
    Vector3f color[3]; //color at each vertex;  颜色
    Vector2f tex_coords[3]; //texture u,v 纹理
    Vector3f normal[3]; //normal vector for each vertex 每个顶点的法向量

    //Texture *tex;
    Triangle();

    void setVertex(int ind, Vector3f ver); /*set i-th vertex coordinates  顶点坐标 */ 
    void setNormal(int ind, Vector3f n); /*set i-th vertex normal vector 顶点法向量 */
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color 顶点的颜色 */
    Vector3f getColor() const { return color[0]*255; } // Only one color per triangle.
    void setTexCoord(int ind, float s, float t); /*set i-th vertex texture coordinate 顶点纹理坐标 */
    std::array<Vector4f, 3> toVector4() const;
};






#endif //RASTERIZER_TRIANGLE_H
