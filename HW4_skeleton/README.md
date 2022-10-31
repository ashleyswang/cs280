## Homework 4 Report
Ashley Wang (Perm: 5339627)

---

### CMakeLists.txt & Bonus
No modifications to CMakeLists.txt or bonus questions implemented.

--- 

### `rasterize_triangle()`

* Found bounding box & iterated through each position using the same method as HW3. In the loop, we compute the Barycentric coordinates (alpha, beta, gamma) and calculate the interpolated z depth. Similar to HW3, we only update the depth buffer and interpolate the necessary attributes if the interpolated depth is closer to the eye.
* To interpolate the color, normal, textcoords, and shading coords we use the interpolate function using the alpha, beta, gamma calculated previously and `t.color`, `t.normal`, `t.text_coords`, and `view_pos` respectively. 
* Then we create the `fragment_shader_payload` and update the `view_pos` and pass the payload into the fragment sader to get the final color and set the final color in the frame buffer.

``` cpp
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
  auto v = t.toVector4();

  // find bounding box of current triangle
  float minx = std::min({v[0].x(), v[1].x(), v[2].x()});
  float maxx = std::max({v[0].x(), v[1].x(), v[2].x()});
  float miny = std::min({v[0].y(), v[1].y(), v[2].y()});
  float maxy = std::max({v[0].y(), v[1].y(), v[2].y()});

  // iterate through the pixel and find if the current pixel is inside the triangle
  for (int x = minx; x < maxx; ++x) {
    for (int y = miny; y < maxy; ++y) {
      if (!insideTriangle(x + 0.5, y + 0.5, t.v)) continue;

      // TODO: Inside your rasterization loop:
      //    * v[i].w() is the vertex view space depth value z.
      //    * Z is interpolated view space depth for the current pixel
      //    * zp is depth between zNear and zFar, used for z-buffer
      auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
      float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
      float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
      zp *= Z;

      // compare the interpolated z to the value from the depth buffer
      auto i = get_index(x, y);
      if (zp >= depth_buf[i]) continue;
      depth_buf[i] = zp;

      // TODO: Interpolate the attributes:
      auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], 
                                            t.color[1], t.color[2], 1);
      auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], 
                                             t.normal[1], t.normal[2], 1);
      auto interpolated_texcoords = interpolate(alpha, beta, gamma, 
                                                t.tex_coords[0], 
                                                t.tex_coords[1], 
                                                t.tex_coords[2], 1);
      auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, 
                                                    view_pos[0], view_pos[1], 
                                                    view_pos[2], 1);

      fragment_shader_payload payload(interpolated_color, 
                                      interpolated_normal.normalized(), 
                                      interpolated_texcoords, 
                                      texture ? &*texture : nullptr);
      payload.view_pos = interpolated_shadingcoords;
      // Use: Instead of passing the triangle's color directly to the frame 
      // buffer, pass the color to the shaders first to get the final color;
      Eigen::Vector2i pixel(x, y);
      auto pixel_color = fragment_shader(payload);
      set_pixel(pixel, pixel_color);
    }
  }
}
```

---

### `phong_fragment_shader()`

* For each light, we compute the ambient, diffuse, and specular light vectors in each dimension using Blinn-Phong's formula. We return the vector sum of all three terms.
  * Ambient term: $L_a = k_a * I_a$
  * Diffuse term: $L_d = k_d * (I/r^2) * \max(0, \mathbf{n}\cdot\mathbf{l})$
  * Specular term: $L_s = k_s * (I/r^2) * \max(0, \mathbf{n}\cdot\mathbf{h})^p$ where $\mathbf{h} = \frac{\mathbf{v} + \mathbf{l}}{||\mathbf{v} + \mathbf{l}||}$

```cpp
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    // Given code
    ...

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the 
        // *ambient*, *diffuse*, and *specular* components are. Then, 
        // accumulate that result on the *result_color* object.

        Eigen::Vector3f ambient;
        Eigen::Vector3f diffuse;
        Eigen::Vector3f specular;

        auto view_dir = (eye_pos - point).normalized();
        auto light_dir = (light.position - point).normalized();
        auto h = (view_dir + light_dir).normalized();
        auto r2 = (light.position - point).squaredNorm();

        for (int i = 0; i < 3; ++i) {
            ambient[i] = ka[i] * amb_light_intensity[i];
            diffuse[i] = kd[i] * (light.intensity[i] / r2) * std::max(0.0f, normal.dot(light_dir));
            specular[i] = ks[i] * (light.intensity[i] / r2) * std::pow(std::max(0.0f, normal.dot(h)), p);
        }

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}
```

---

### `texture_fragment_shader()`

* If the payload texture is given, update the texture color and use as the diffuse light coefficient $k_d$. 
* Calculate the light vector using Blinn-Phong's formula (same as `phong_fragment_shader()`).

```cpp
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f texture_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the 
        // current fragment
        texture_color = payload.texture->getColor(payload.tex_coords.x(), 
                                                  payload.tex_coords.y());
    }

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    // Given code
    ...
    
    // Same code as phong_fragment_shader
    ...
    return result_color * 255.f;
}
```
