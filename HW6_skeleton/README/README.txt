CS180/280 Homework 6
Ashley Wang
NetId: ashleyswang
Perms: 5339627

CMakeLists.txt - no changes

Bonus - not implemented

--- 

(output included in out.ppm)

void Renderer::Render(): 
first find position of the center of the current pixel (x,y) and get the direction vector that passes through it. 
then we pass in the dir vector into castRay() and store the color in the corresponding pixel of the framebuffer.


bool rayTriangleIntersect(): 
implemented Möller Trumbore Algorithm and updated tnear, u, and v only if ray intersect the triangle plane