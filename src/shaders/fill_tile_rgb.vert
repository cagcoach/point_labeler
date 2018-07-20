#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 2) in uint  in_label;
layout (location = 4) in vec3  in_color;

uniform uint scan;

uniform mat4 pose;

uniform float minRange;
uniform float maxRange;

uniform vec2 tilePos;
uniform float tileSize;
uniform float tileBoundary;


out POINT
{
  bool valid;
  vec3 color;
  uint label;
} vs_out;


void main()
{
  float range = length(in_vertex);
  
  vec4 v_global = pose * vec4(in_vertex, 1.0);
  vec2 v = v_global.xy - tilePos;
  
  bool inside_tile = (abs(v.x) < 0.5 * (tileSize + tileBoundary) && abs(v.y) < 0.5 * (tileSize + tileBoundary));
  bool out_of_range = range < minRange || range > maxRange;
   
  vs_out.valid = false;
  
  if(inside_tile && !out_of_range)
  {
    vs_out.valid = true;
    vs_out.color = in_color;
    vs_out.label = in_label;
  }
}